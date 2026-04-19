//! Translated from `c/silk/PLC.c` (RFC 6716).
//!
//! Packet-loss concealment: keeps a rolling snapshot of the last good
//! frame's parameters (LTP coefs, pitch lag, gains, LPC coefs, LTP
//! scale) and, on loss, synthesises a replacement signal by LTP/LPC
//! filtering a rewhitened copy of the previous excitation with
//! progressively attenuated gains. `silk_PLC_glue_frames` crossfades
//! into the first good frame after loss.

use super::bwexpander::silk_bwexpander;
use super::lpc_analysis_filter::silk_LPC_analysis_filter;
use super::lpc_inv_pred_gain::silk_LPC_inverse_pred_gain;
use super::macros::{
    silk_add_lshift32, silk_clz32, silk_div32, silk_div32_16, silk_inverse32_varq, silk_lshift, silk_lshift32, silk_max_16,
    silk_max_32, silk_max_int, silk_min, silk_min_32, silk_min_int, silk_mla_ovflw, silk_rshift, silk_rshift_round, silk_sat16,
    silk_smlawb, silk_smulbb, silk_smulwb, silk_smulww,
};
use super::sqrt_approx::silk_sqrt_approx;
use super::structs::{
    LTP_ORDER, MAX_FRAME_LENGTH, MAX_LPC_ORDER, MAX_NB_SUBFR, MAX_SUB_FRAME_LENGTH, SilkDecoderControl, SilkDecoderState,
};
use super::sum_sqr_shift::silk_sum_sqr_shift;

const NB_ATT: usize = 2;
const HARM_ATT_Q15: [i16; NB_ATT] = [32440, 31130]; /* 0.99, 0.95 */
const PLC_RAND_ATTENUATE_V_Q15: [i16; NB_ATT] = [31130, 26214]; /* 0.95, 0.8 */
const PLC_RAND_ATTENUATE_UV_Q15: [i16; NB_ATT] = [32440, 29491]; /* 0.99, 0.9 */

const TYPE_VOICED: i32 = 2;
const BWE_COEF_Q16: i32 = (0.99 * (1u32 << 16) as f64 + 0.5) as i32; /* SILK_FIX_CONST(BWE_COEF, 16) */
const V_PITCH_GAIN_START_MIN_Q14: i32 = 11469;
const V_PITCH_GAIN_START_MAX_Q14: i32 = 15565;
const MAX_PITCH_LAG_MS: i32 = 18;
const RAND_BUF_SIZE: i32 = 128;
const RAND_BUF_MASK: i32 = RAND_BUF_SIZE - 1;
const LOG2_INV_LPC_GAIN_HIGH_THRES: i32 = 3;
const LOG2_INV_LPC_GAIN_LOW_THRES: i32 = 8;
const PITCH_DRIFT_FAC_Q16: i32 = 655;

/// `silk_PLC_Reset` — initialize PLC state after an fs_kHz change.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_PLC_Reset(ps_dec: *mut SilkDecoderState) {
    unsafe {
        (*ps_dec).s_plc.pitch_l_q8 = silk_lshift((*ps_dec).frame_length, 8 - 1);
        (*ps_dec).s_plc.prev_gain_q16[0] = 1 << 16;
        (*ps_dec).s_plc.prev_gain_q16[1] = 1 << 16;
        (*ps_dec).s_plc.subfr_length = 20;
        (*ps_dec).s_plc.nb_subfr = 2;
    }
}

/// `silk_PLC` — PLC control function.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_PLC(
    ps_dec: *mut SilkDecoderState,
    ps_dec_ctrl: *mut SilkDecoderControl,
    frame: *mut i16,
    lost: i32,
) {
    unsafe {
        if (*ps_dec).fs_khz != (*ps_dec).s_plc.fs_khz {
            silk_PLC_Reset(ps_dec);
            (*ps_dec).s_plc.fs_khz = (*ps_dec).fs_khz;
        }

        if lost != 0 {
            silk_PLC_conceal(ps_dec, ps_dec_ctrl, frame);
            (*ps_dec).loss_cnt += 1;
        } else {
            silk_PLC_update(ps_dec, ps_dec_ctrl);
        }
    }
}

/// Update state of PLC.
unsafe fn silk_PLC_update(ps_dec: *mut SilkDecoderState, ps_dec_ctrl: *mut SilkDecoderControl) {
    unsafe {
        let ps_plc = &raw mut (*ps_dec).s_plc;

        /* Update parameters used in case of packet loss */
        (*ps_dec).prev_signal_type = (*ps_dec).indices.signal_type as i32;
        let mut ltp_gain_q14 = 0i32;
        if (*ps_dec).indices.signal_type as i32 == TYPE_VOICED {
            /* Find the parameters for the last subframe which contains a pitch pulse */
            let mut j = 0i32;
            while j * (*ps_dec).subfr_length < (*ps_dec_ctrl).pitch_l[((*ps_dec).nb_subfr - 1) as usize] {
                if j == (*ps_dec).nb_subfr {
                    break;
                }
                let mut temp_ltp_gain_q14 = 0i32;
                let mut i = 0i32;
                while i < LTP_ORDER as i32 {
                    temp_ltp_gain_q14 +=
                        (*ps_dec_ctrl).ltp_coef_q14[(((*ps_dec).nb_subfr - 1 - j) * LTP_ORDER as i32 + i) as usize] as i32;
                    i += 1;
                }
                if temp_ltp_gain_q14 > ltp_gain_q14 {
                    ltp_gain_q14 = temp_ltp_gain_q14;
                    core::ptr::copy_nonoverlapping(
                        (*ps_dec_ctrl)
                            .ltp_coef_q14
                            .as_ptr()
                            .add(silk_smulbb((*ps_dec).nb_subfr - 1 - j, LTP_ORDER as i32) as usize),
                        (*ps_plc).ltp_coef_q14.as_mut_ptr(),
                        LTP_ORDER,
                    );

                    (*ps_plc).pitch_l_q8 = silk_lshift((*ps_dec_ctrl).pitch_l[((*ps_dec).nb_subfr - 1 - j) as usize], 8);
                }
                j += 1;
            }

            core::ptr::write_bytes((*ps_plc).ltp_coef_q14.as_mut_ptr(), 0, LTP_ORDER);
            (*ps_plc).ltp_coef_q14[LTP_ORDER / 2] = ltp_gain_q14 as i16;

            /* Limit LT coefs */
            if ltp_gain_q14 < V_PITCH_GAIN_START_MIN_Q14 {
                let tmp = silk_lshift(V_PITCH_GAIN_START_MIN_Q14, 10);
                let scale_q10 = silk_div32(tmp, silk_max_32(ltp_gain_q14, 1));
                let mut i = 0;
                while i < LTP_ORDER {
                    (*ps_plc).ltp_coef_q14[i] =
                        silk_rshift(silk_smulbb((*ps_plc).ltp_coef_q14[i] as i32, scale_q10), 10) as i16;
                    i += 1;
                }
            } else if ltp_gain_q14 > V_PITCH_GAIN_START_MAX_Q14 {
                let tmp = silk_lshift(V_PITCH_GAIN_START_MAX_Q14, 14);
                let scale_q14 = silk_div32(tmp, silk_max_32(ltp_gain_q14, 1));
                let mut i = 0;
                while i < LTP_ORDER {
                    (*ps_plc).ltp_coef_q14[i] =
                        silk_rshift(silk_smulbb((*ps_plc).ltp_coef_q14[i] as i32, scale_q14), 14) as i16;
                    i += 1;
                }
            }
        } else {
            (*ps_plc).pitch_l_q8 = silk_lshift(silk_smulbb((*ps_dec).fs_khz, 18), 8);
            core::ptr::write_bytes((*ps_plc).ltp_coef_q14.as_mut_ptr(), 0, LTP_ORDER);
        }

        /* Save LPC coeficients */
        core::ptr::copy_nonoverlapping(
            (*ps_dec_ctrl).pred_coef_q12[1].as_ptr(),
            (*ps_plc).prev_lpc_q12.as_mut_ptr(),
            (*ps_dec).lpc_order as usize,
        );
        (*ps_plc).prev_ltp_scale_q14 = (*ps_dec_ctrl).ltp_scale_q14 as i16;

        /* Save last two gains */
        core::ptr::copy_nonoverlapping(
            (*ps_dec_ctrl).gains_q16.as_ptr().add(((*ps_dec).nb_subfr - 2) as usize),
            (*ps_plc).prev_gain_q16.as_mut_ptr(),
            2,
        );

        (*ps_plc).subfr_length = (*ps_dec).subfr_length;
        (*ps_plc).nb_subfr = (*ps_dec).nb_subfr;
    }
}

unsafe fn silk_PLC_conceal(ps_dec: *mut SilkDecoderState, ps_dec_ctrl: *mut SilkDecoderControl, frame: *mut i16) {
    unsafe {
        let mut exc_buf = [0i16; 2 * MAX_SUB_FRAME_LENGTH];
        let mut a_q12 = [0i16; MAX_LPC_ORDER];
        let mut s_ltp = [0i16; MAX_FRAME_LENGTH];
        let mut s_ltp_q14 = [0i32; 2 * MAX_FRAME_LENGTH];
        let ps_plc = &raw mut (*ps_dec).s_plc;
        let mut prev_gain_q10 = [0i32; 2];

        prev_gain_q10[0] = silk_rshift((*ps_plc).prev_gain_q16[0], 6);
        prev_gain_q10[1] = silk_rshift((*ps_plc).prev_gain_q16[1], 6);

        if (*ps_dec).first_frame_after_reset != 0 {
            core::ptr::write_bytes((*ps_plc).prev_lpc_q12.as_mut_ptr(), 0, MAX_LPC_ORDER);
        }

        /* Find random noise component */
        /* Scale previous excitation signal */
        let subfr_length = (*ps_plc).subfr_length;
        let nb_subfr = (*ps_plc).nb_subfr;
        let mut exc_buf_off = 0usize;
        let mut k = 0i32;
        while k < 2 {
            let mut i = 0i32;
            while i < subfr_length {
                exc_buf[exc_buf_off + i as usize] = silk_sat16(silk_rshift(
                    silk_smulww((*ps_dec).exc_q14[(i + (k + nb_subfr - 2) * subfr_length) as usize], prev_gain_q10[k as usize]),
                    8,
                )) as i16;
                i += 1;
            }
            exc_buf_off += subfr_length as usize;
            k += 1;
        }
        /* Find the subframe with lowest energy of the last two and use that as random noise generator */
        let mut energy1 = 0i32;
        let mut shift1 = 0i32;
        let mut energy2 = 0i32;
        let mut shift2 = 0i32;
        silk_sum_sqr_shift(&mut energy1, &mut shift1, exc_buf.as_ptr(), subfr_length);
        silk_sum_sqr_shift(&mut energy2, &mut shift2, exc_buf.as_ptr().add(subfr_length as usize), subfr_length);

        let rand_ptr: *const i32 = if silk_rshift(energy1, shift2) < silk_rshift(energy2, shift1) {
            /* First sub-frame has lowest energy */
            (*ps_dec).exc_q14.as_ptr().add(silk_max_int(0, (nb_subfr - 1) * subfr_length - RAND_BUF_SIZE) as usize)
        } else {
            /* Second sub-frame has lowest energy */
            (*ps_dec).exc_q14.as_ptr().add(silk_max_int(0, nb_subfr * subfr_length - RAND_BUF_SIZE) as usize)
        };

        /* Set up Gain to random noise component */
        let b_q14 = (*ps_plc).ltp_coef_q14.as_mut_ptr();
        let mut rand_scale_q14 = (*ps_plc).rand_scale_q14;

        /* Set up attenuation gains */
        let harm_gain_q15 = HARM_ATT_Q15[silk_min_int(NB_ATT as i32 - 1, (*ps_dec).loss_cnt) as usize] as i32;
        let mut rand_gain_q15 = if (*ps_dec).prev_signal_type == TYPE_VOICED {
            PLC_RAND_ATTENUATE_V_Q15[silk_min_int(NB_ATT as i32 - 1, (*ps_dec).loss_cnt) as usize] as i32
        } else {
            PLC_RAND_ATTENUATE_UV_Q15[silk_min_int(NB_ATT as i32 - 1, (*ps_dec).loss_cnt) as usize] as i32
        };

        /* LPC concealment. Apply BWE to previous LPC */
        silk_bwexpander((*ps_plc).prev_lpc_q12.as_mut_ptr(), (*ps_dec).lpc_order, BWE_COEF_Q16);

        /* Preload LPC coeficients to array on stack. Gives small performance gain */
        core::ptr::copy_nonoverlapping((*ps_plc).prev_lpc_q12.as_ptr(), a_q12.as_mut_ptr(), (*ps_dec).lpc_order as usize);

        /* First Lost frame */
        if (*ps_dec).loss_cnt == 0 {
            rand_scale_q14 = 1 << 14;

            /* Reduce random noise Gain for voiced frames */
            if (*ps_dec).prev_signal_type == TYPE_VOICED {
                let mut i = 0i32;
                while i < LTP_ORDER as i32 {
                    rand_scale_q14 -= *b_q14.offset(i as isize);
                    i += 1;
                }
                rand_scale_q14 = silk_max_16(3277, rand_scale_q14 as i32) as i16;
                rand_scale_q14 =
                    silk_rshift(silk_smulbb(rand_scale_q14 as i32, (*ps_plc).prev_ltp_scale_q14 as i32), 14) as i16;
            } else {
                /* Reduce random noise for unvoiced frames with high LPC gain */
                let inv_gain_q30 = silk_LPC_inverse_pred_gain((*ps_plc).prev_lpc_q12.as_ptr(), (*ps_dec).lpc_order);

                let mut down_scale_q30 = silk_min_32(silk_rshift(1 << 30, LOG2_INV_LPC_GAIN_HIGH_THRES), inv_gain_q30);
                down_scale_q30 = silk_max_32(silk_rshift(1 << 30, LOG2_INV_LPC_GAIN_LOW_THRES), down_scale_q30);
                down_scale_q30 = silk_lshift(down_scale_q30, LOG2_INV_LPC_GAIN_HIGH_THRES);

                rand_gain_q15 = silk_rshift(silk_smulwb(down_scale_q30, rand_gain_q15), 14);
            }
        }

        let mut rand_seed = (*ps_plc).rand_seed;
        let mut lag = silk_rshift_round((*ps_plc).pitch_l_q8, 8);
        let mut s_ltp_buf_idx = (*ps_dec).ltp_mem_length;

        /* Rewhiten LTP state */
        let idx = (*ps_dec).ltp_mem_length - lag - (*ps_dec).lpc_order - LTP_ORDER as i32 / 2;
        silk_LPC_analysis_filter(
            s_ltp.as_mut_ptr().offset(idx as isize),
            (*ps_dec).out_buf.as_ptr().offset(idx as isize),
            a_q12.as_ptr(),
            (*ps_dec).ltp_mem_length - idx,
            (*ps_dec).lpc_order,
        );
        /* Scale LTP state */
        let mut inv_gain_q30 = silk_inverse32_varq((*ps_plc).prev_gain_q16[1], 46);
        inv_gain_q30 = silk_min(inv_gain_q30, i32::MAX >> 1);
        let mut i = idx + (*ps_dec).lpc_order;
        while i < (*ps_dec).ltp_mem_length {
            s_ltp_q14[i as usize] = silk_smulwb(inv_gain_q30, s_ltp[i as usize] as i32);
            i += 1;
        }

        /***************************/
        /* LTP synthesis filtering */
        /***************************/
        let mut k = 0i32;
        while k < (*ps_dec).nb_subfr {
            /* Set up pointer */
            let mut pred_lag_off = (s_ltp_buf_idx - lag + LTP_ORDER as i32 / 2) as isize;
            let mut i = 0i32;
            while i < (*ps_dec).subfr_length {
                /* Unrolled loop */
                /* Avoids introducing a bias because silk_SMLAWB() always rounds to -inf */
                let mut ltp_pred_q12 = 2i32;
                ltp_pred_q12 = silk_smlawb(ltp_pred_q12, s_ltp_q14[pred_lag_off as usize], *b_q14.offset(0) as i32);
                ltp_pred_q12 = silk_smlawb(ltp_pred_q12, s_ltp_q14[(pred_lag_off - 1) as usize], *b_q14.offset(1) as i32);
                ltp_pred_q12 = silk_smlawb(ltp_pred_q12, s_ltp_q14[(pred_lag_off - 2) as usize], *b_q14.offset(2) as i32);
                ltp_pred_q12 = silk_smlawb(ltp_pred_q12, s_ltp_q14[(pred_lag_off - 3) as usize], *b_q14.offset(3) as i32);
                ltp_pred_q12 = silk_smlawb(ltp_pred_q12, s_ltp_q14[(pred_lag_off - 4) as usize], *b_q14.offset(4) as i32);
                pred_lag_off += 1;

                /* Generate LPC excitation */
                rand_seed = silk_mla_ovflw(907633515, rand_seed, 196314165);
                let ridx = silk_rshift(rand_seed, 25) & RAND_BUF_MASK;
                s_ltp_q14[s_ltp_buf_idx as usize] =
                    silk_lshift32(silk_smlawb(ltp_pred_q12, *rand_ptr.offset(ridx as isize), rand_scale_q14 as i32), 2);
                s_ltp_buf_idx += 1;
                i += 1;
            }

            /* Gradually reduce LTP gain */
            let mut j = 0i32;
            while j < LTP_ORDER as i32 {
                *b_q14.offset(j as isize) =
                    silk_rshift(silk_smulbb(harm_gain_q15, *b_q14.offset(j as isize) as i32), 15) as i16;
                j += 1;
            }
            /* Gradually reduce excitation gain */
            rand_scale_q14 = silk_rshift(silk_smulbb(rand_scale_q14 as i32, rand_gain_q15), 15) as i16;

            /* Slowly increase pitch lag */
            (*ps_plc).pitch_l_q8 = silk_smlawb((*ps_plc).pitch_l_q8, (*ps_plc).pitch_l_q8, PITCH_DRIFT_FAC_Q16);
            (*ps_plc).pitch_l_q8 =
                silk_min_32((*ps_plc).pitch_l_q8, silk_lshift(silk_smulbb(MAX_PITCH_LAG_MS, (*ps_dec).fs_khz), 8));
            lag = silk_rshift_round((*ps_plc).pitch_l_q8, 8);
            k += 1;
        }

        /***************************/
        /* LPC synthesis filtering */
        /***************************/
        let s_lpc_off = ((*ps_dec).ltp_mem_length - MAX_LPC_ORDER as i32) as usize;

        /* Copy LPC state */
        core::ptr::copy_nonoverlapping((*ps_dec).s_lpc_q14_buf.as_ptr(), s_ltp_q14.as_mut_ptr().add(s_lpc_off), MAX_LPC_ORDER);

        let mut i = 0i32;
        while i < (*ps_dec).frame_length {
            /* partly unrolled */
            /* Avoids introducing a bias because silk_SMLAWB() always rounds to -inf */
            let base = s_lpc_off + MAX_LPC_ORDER + i as usize;
            let mut lpc_pred_q10 = silk_rshift((*ps_dec).lpc_order, 1);
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - 1], a_q12[0] as i32);
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - 2], a_q12[1] as i32);
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - 3], a_q12[2] as i32);
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - 4], a_q12[3] as i32);
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - 5], a_q12[4] as i32);
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - 6], a_q12[5] as i32);
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - 7], a_q12[6] as i32);
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - 8], a_q12[7] as i32);
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - 9], a_q12[8] as i32);
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - 10], a_q12[9] as i32);
            let mut j = 10i32;
            while j < (*ps_dec).lpc_order {
                lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - (j as usize) - 1], a_q12[j as usize] as i32);
                j += 1;
            }

            /* Add prediction to LPC excitation */
            s_ltp_q14[base] = silk_add_lshift32(s_ltp_q14[base], lpc_pred_q10, 4);

            /* Scale with Gain */
            *frame.offset(i as isize) =
                silk_sat16(silk_sat16(silk_rshift_round(silk_smulww(s_ltp_q14[base], prev_gain_q10[1]), 8))) as i16;
            i += 1;
        }

        /* Save LPC state */
        core::ptr::copy_nonoverlapping(
            s_ltp_q14.as_ptr().add(s_lpc_off + (*ps_dec).frame_length as usize),
            (*ps_dec).s_lpc_q14_buf.as_mut_ptr(),
            MAX_LPC_ORDER,
        );

        /**************************************/
        /* Update states                      */
        /**************************************/
        (*ps_plc).rand_seed = rand_seed;
        (*ps_plc).rand_scale_q14 = rand_scale_q14;
        let mut i = 0;
        while i < MAX_NB_SUBFR {
            (*ps_dec_ctrl).pitch_l[i] = lag;
            i += 1;
        }
    }
}

/// `silk_PLC_glue_frames` — glue concealed frames with the next good frame.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_PLC_glue_frames(ps_dec: *mut SilkDecoderState, frame: *mut i16, length: i32) {
    unsafe {
        let ps_plc = &raw mut (*ps_dec).s_plc;

        if (*ps_dec).loss_cnt != 0 {
            /* Calculate energy in concealed residual */
            silk_sum_sqr_shift(&raw mut (*ps_plc).conc_energy, &raw mut (*ps_plc).conc_energy_shift, frame, length);
            (*ps_plc).last_frame_lost = 1;
        } else {
            if (*ps_plc).last_frame_lost != 0 {
                /* Calculate residual in decoded signal if last frame was lost */
                let mut energy = 0i32;
                let mut energy_shift = 0i32;
                silk_sum_sqr_shift(&mut energy, &mut energy_shift, frame, length);

                /* Normalize energies */
                if energy_shift > (*ps_plc).conc_energy_shift {
                    (*ps_plc).conc_energy = silk_rshift((*ps_plc).conc_energy, energy_shift - (*ps_plc).conc_energy_shift);
                } else if energy_shift < (*ps_plc).conc_energy_shift {
                    energy = silk_rshift(energy, (*ps_plc).conc_energy_shift - energy_shift);
                }

                /* Fade in the energy difference */
                if energy > (*ps_plc).conc_energy {
                    let mut lz = silk_clz32((*ps_plc).conc_energy);
                    lz -= 1;
                    (*ps_plc).conc_energy = silk_lshift((*ps_plc).conc_energy, lz);
                    energy = silk_rshift(energy, silk_max_32(24 - lz, 0));

                    let frac_q24 = silk_div32((*ps_plc).conc_energy, silk_max_32(energy, 1));

                    let mut gain_q16 = silk_lshift(silk_sqrt_approx(frac_q24), 4);
                    let mut slope_q16 = silk_div32_16((1 << 16) - gain_q16, length);
                    /* Make slope 4x steeper to avoid missing onsets after DTX */
                    slope_q16 = silk_lshift(slope_q16, 2);

                    let mut i = 0i32;
                    while i < length {
                        *frame.offset(i as isize) = silk_smulwb(gain_q16, *frame.offset(i as isize) as i32) as i16;
                        gain_q16 += slope_q16;
                        if gain_q16 > 1 << 16 {
                            break;
                        }
                        i += 1;
                    }
                }
            }
            (*ps_plc).last_frame_lost = 0;
        }
    }
}
