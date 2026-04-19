//! Translated from `c/silk/decode_core.c` (RFC 6716).
//!
//! Core decoder that runs the inverse NSQ: expands the pulse signal
//! into an excitation, applies LTP re-whitening / prediction on voiced
//! subframes, runs LPC synthesis with per-subframe gain matching, and
//! writes the reconstructed speech into `xq`.

use super::lpc_analysis_filter::silk_LPC_analysis_filter;
use super::macros::{
    silk_add32_ovflw, silk_add_lshift32, silk_div32_varq, silk_inverse32_varq, silk_lshift, silk_rshift,
    silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb, silk_smulww,
};
use super::structs::{
    LTP_ORDER, MAX_FRAME_LENGTH, MAX_LPC_ORDER, MAX_NB_SUBFR, MAX_SUB_FRAME_LENGTH, SilkDecoderControl, SilkDecoderState,
};
use super::tables_other::silk_Quantization_Offsets_Q10;

const QUANT_LEVEL_ADJUST_Q10: i32 = 80;
const TYPE_VOICED: i32 = 2;

/// `silk_decode_core` — inverse NSQ (LTP + LPC).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_decode_core(
    ps_dec: *mut SilkDecoderState,
    ps_dec_ctrl: *mut SilkDecoderControl,
    xq: *mut i16,
    pulses: *const i32,
) {
    unsafe {
        let mut a_q12_tmp = [0i16; MAX_LPC_ORDER];
        let mut s_ltp = [0i16; MAX_FRAME_LENGTH];
        let mut s_ltp_q15 = [0i32; 2 * MAX_FRAME_LENGTH];
        let mut res_q14 = [0i32; MAX_SUB_FRAME_LENGTH];
        let mut s_lpc_q14 = [0i32; MAX_SUB_FRAME_LENGTH + MAX_LPC_ORDER];

        let offset_q10 = silk_Quantization_Offsets_Q10[((*ps_dec).indices.signal_type as usize) >> 1]
            [(*ps_dec).indices.quant_offset_type as usize] as i32;

        let nlsf_interpolation_flag: i32 = if ((*ps_dec).indices.nlsf_interp_coef_q2 as i32) < (1 << 2) { 1 } else { 0 };

        /* Decode excitation */
        let mut rand_seed = (*ps_dec).indices.seed as i32;
        let mut i = 0i32;
        while i < (*ps_dec).frame_length {
            rand_seed = 907633515i32.wrapping_add(rand_seed.wrapping_mul(196314165));
            let mut v = silk_lshift(*pulses.offset(i as isize), 14);
            if v > 0 {
                v -= QUANT_LEVEL_ADJUST_Q10 << 4;
            } else if v < 0 {
                v += QUANT_LEVEL_ADJUST_Q10 << 4;
            }
            v += offset_q10 << 4;
            if rand_seed < 0 {
                v = -v;
            }
            (*ps_dec).exc_q14[i as usize] = v;
            rand_seed = silk_add32_ovflw(rand_seed, *pulses.offset(i as isize));
            i += 1;
        }

        /* Copy LPC state */
        core::ptr::copy_nonoverlapping(
            (*ps_dec).s_lpc_q14_buf.as_ptr(),
            s_lpc_q14.as_mut_ptr(),
            MAX_LPC_ORDER,
        );

        let mut pexc_off = 0i32;
        let mut pxq_off = 0i32;
        let mut s_ltp_buf_idx = (*ps_dec).ltp_mem_length;
        let mut lag = 0i32;
        /* Loop over subframes */
        let mut k = 0i32;
        while k < (*ps_dec).nb_subfr {
            let a_q12 = (*ps_dec_ctrl).pred_coef_q12[(k >> 1) as usize].as_ptr();

            /* Preload LPC coeficients to array on stack */
            core::ptr::copy_nonoverlapping(a_q12, a_q12_tmp.as_mut_ptr(), (*ps_dec).lpc_order as usize);
            let b_q14 = (*ps_dec_ctrl).ltp_coef_q14.as_mut_ptr().offset((k * LTP_ORDER as i32) as isize);
            let mut signal_type = (*ps_dec).indices.signal_type as i32;

            let gain_q10 = silk_rshift((*ps_dec_ctrl).gains_q16[k as usize], 6);
            let mut inv_gain_q31 = silk_inverse32_varq((*ps_dec_ctrl).gains_q16[k as usize], 47);

            /* Calculate gain adjustment factor */
            let gain_adj_q16 = if (*ps_dec_ctrl).gains_q16[k as usize] != (*ps_dec).prev_gain_q16 {
                let adj = silk_div32_varq((*ps_dec).prev_gain_q16, (*ps_dec_ctrl).gains_q16[k as usize], 16);
                let mut i = 0;
                while i < MAX_LPC_ORDER {
                    s_lpc_q14[i] = silk_smulww(adj, s_lpc_q14[i]);
                    i += 1;
                }
                adj
            } else {
                1 << 16
            };

            /* Save inv_gain */
            (*ps_dec).prev_gain_q16 = (*ps_dec_ctrl).gains_q16[k as usize];

            /* Avoid abrupt transition from voiced PLC to unvoiced normal decoding */
            if (*ps_dec).loss_cnt != 0
                && (*ps_dec).prev_signal_type == TYPE_VOICED
                && (*ps_dec).indices.signal_type as i32 != TYPE_VOICED
                && k < MAX_NB_SUBFR as i32 / 2
            {
                core::ptr::write_bytes(b_q14, 0, LTP_ORDER);
                *b_q14.offset((LTP_ORDER / 2) as isize) = (0.25 * (1u32 << 14) as f64 + 0.5) as i16;

                signal_type = TYPE_VOICED;
                (*ps_dec_ctrl).pitch_l[k as usize] = (*ps_dec).lag_prev;
            }

            if signal_type == TYPE_VOICED {
                /* Voiced */
                lag = (*ps_dec_ctrl).pitch_l[k as usize];

                /* Re-whitening */
                if k == 0 || (k == 2 && nlsf_interpolation_flag != 0) {
                    /* Rewhiten with new A coefs */
                    let start_idx = (*ps_dec).ltp_mem_length - lag - (*ps_dec).lpc_order - LTP_ORDER as i32 / 2;

                    if k == 2 {
                        core::ptr::copy_nonoverlapping(
                            xq,
                            (*ps_dec).out_buf.as_mut_ptr().offset((*ps_dec).ltp_mem_length as isize),
                            (2 * (*ps_dec).subfr_length) as usize,
                        );
                    }

                    silk_LPC_analysis_filter(
                        s_ltp.as_mut_ptr().offset(start_idx as isize),
                        (*ps_dec).out_buf.as_ptr().offset((start_idx + k * (*ps_dec).subfr_length) as isize),
                        a_q12,
                        (*ps_dec).ltp_mem_length - start_idx,
                        (*ps_dec).lpc_order,
                    );

                    /* After rewhitening the LTP state is unscaled */
                    if k == 0 {
                        /* Do LTP downscaling to reduce inter-packet dependency */
                        inv_gain_q31 = silk_lshift(silk_smulwb(inv_gain_q31, (*ps_dec_ctrl).ltp_scale_q14), 2);
                    }
                    let mut i = 0i32;
                    while i < lag + LTP_ORDER as i32 / 2 {
                        s_ltp_q15[(s_ltp_buf_idx - i - 1) as usize] =
                            silk_smulwb(inv_gain_q31, s_ltp[((*ps_dec).ltp_mem_length - i - 1) as usize] as i32);
                        i += 1;
                    }
                } else {
                    /* Update LTP state when Gain changes */
                    if gain_adj_q16 != 1 << 16 {
                        let mut i = 0i32;
                        while i < lag + LTP_ORDER as i32 / 2 {
                            s_ltp_q15[(s_ltp_buf_idx - i - 1) as usize] =
                                silk_smulww(gain_adj_q16, s_ltp_q15[(s_ltp_buf_idx - i - 1) as usize]);
                            i += 1;
                        }
                    }
                }
            }

            /* Long-term prediction */
            let pres_is_exc;
            if signal_type == TYPE_VOICED {
                let mut pred_lag_off = (s_ltp_buf_idx - lag + LTP_ORDER as i32 / 2) as isize;
                let mut i = 0i32;
                while i < (*ps_dec).subfr_length {
                    /* Unrolled loop */
                    /* Avoids introducing a bias because silk_SMLAWB() always rounds to -inf */
                    let mut ltp_pred_q13 = 2i32;
                    ltp_pred_q13 = silk_smlawb(ltp_pred_q13, s_ltp_q15[pred_lag_off as usize], *b_q14.offset(0) as i32);
                    ltp_pred_q13 =
                        silk_smlawb(ltp_pred_q13, s_ltp_q15[(pred_lag_off - 1) as usize], *b_q14.offset(1) as i32);
                    ltp_pred_q13 =
                        silk_smlawb(ltp_pred_q13, s_ltp_q15[(pred_lag_off - 2) as usize], *b_q14.offset(2) as i32);
                    ltp_pred_q13 =
                        silk_smlawb(ltp_pred_q13, s_ltp_q15[(pred_lag_off - 3) as usize], *b_q14.offset(3) as i32);
                    ltp_pred_q13 =
                        silk_smlawb(ltp_pred_q13, s_ltp_q15[(pred_lag_off - 4) as usize], *b_q14.offset(4) as i32);
                    pred_lag_off += 1;

                    /* Generate LPC excitation */
                    res_q14[i as usize] =
                        silk_add_lshift32((*ps_dec).exc_q14[(pexc_off + i) as usize], ltp_pred_q13, 1);

                    /* Update states */
                    s_ltp_q15[s_ltp_buf_idx as usize] = silk_lshift(res_q14[i as usize], 1);
                    s_ltp_buf_idx += 1;
                    i += 1;
                }
                pres_is_exc = false;
            } else {
                pres_is_exc = true;
            }

            let mut i = 0i32;
            while i < (*ps_dec).subfr_length {
                let pres = if pres_is_exc {
                    (*ps_dec).exc_q14[(pexc_off + i) as usize]
                } else {
                    res_q14[i as usize]
                };

                /* Short-term prediction */
                /* Avoids introducing a bias because silk_SMLAWB() always rounds to -inf */
                let mut lpc_pred_q10 = silk_rshift((*ps_dec).lpc_order, 1);
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 1], a_q12_tmp[0] as i32);
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 2], a_q12_tmp[1] as i32);
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 3], a_q12_tmp[2] as i32);
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 4], a_q12_tmp[3] as i32);
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 5], a_q12_tmp[4] as i32);
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 6], a_q12_tmp[5] as i32);
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 7], a_q12_tmp[6] as i32);
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 8], a_q12_tmp[7] as i32);
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 9], a_q12_tmp[8] as i32);
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 10], a_q12_tmp[9] as i32);
                if (*ps_dec).lpc_order == 16 {
                    lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 11], a_q12_tmp[10] as i32);
                    lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 12], a_q12_tmp[11] as i32);
                    lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 13], a_q12_tmp[12] as i32);
                    lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 14], a_q12_tmp[13] as i32);
                    lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 15], a_q12_tmp[14] as i32);
                    lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_lpc_q14[MAX_LPC_ORDER + i as usize - 16], a_q12_tmp[15] as i32);
                }

                /* Add prediction to LPC excitation */
                s_lpc_q14[MAX_LPC_ORDER + i as usize] = silk_add_lshift32(pres, lpc_pred_q10, 4);

                /* Scale with gain */
                *xq.offset((pxq_off + i) as isize) =
                    silk_sat16(silk_rshift_round(silk_smulww(s_lpc_q14[MAX_LPC_ORDER + i as usize], gain_q10), 8)) as i16;
                i += 1;
            }

            /* Update LPC filter state */
            core::ptr::copy(
                s_lpc_q14.as_ptr().add((*ps_dec).subfr_length as usize),
                s_lpc_q14.as_mut_ptr(),
                MAX_LPC_ORDER,
            );
            pexc_off += (*ps_dec).subfr_length;
            pxq_off += (*ps_dec).subfr_length;
            k += 1;
        }

        /* Save LPC state */
        core::ptr::copy_nonoverlapping(
            s_lpc_q14.as_ptr(),
            (*ps_dec).s_lpc_q14_buf.as_mut_ptr(),
            MAX_LPC_ORDER,
        );

    }
}
