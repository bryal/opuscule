//! Translated from `c/silk/PLC.c` (RFC 6716).
//!
//! Packet-loss concealment: keeps a rolling snapshot of the last good
//! frame's parameters (LTP coefs, pitch lag, gains, LPC coefs, LTP
//! scale) and, on loss, synthesises a replacement signal by LTP/LPC
//! filtering a rewhitened copy of the previous excitation with
//! progressively attenuated gains. `silk_PLC_glue_frames` crossfades
//! into the first good frame after loss.

#![allow(clippy::indexing_slicing)] // dense SILK kernels; voice path is deprioritized vs CELT

use super::bwexpander::silk_bwexpander;
use super::lpc_analysis_filter::silk_lpc_analysis_filter;
use super::lpc_inv_pred_gain::silk_lpc_inverse_pred_gain;
use super::macros::{
    silk_clz32, silk_inverse32_varq, silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulbb, silk_smulwb,
    silk_smulww,
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
pub fn silk_plc_reset(ps_dec: &mut SilkDecoderState) {
    ps_dec.s_plc.pitch_l_q8 = silk_lshift(ps_dec.frame_length, 8 - 1);
    ps_dec.s_plc.prev_gain_q16[0] = 1 << 16;
    ps_dec.s_plc.prev_gain_q16[1] = 1 << 16;
    ps_dec.s_plc.subfr_length = 20;
    ps_dec.s_plc.nb_subfr = 2;
}

/// `silk_PLC` — PLC control function.
pub fn silk_plc(ps_dec: &mut SilkDecoderState, ps_dec_ctrl: &mut SilkDecoderControl, frame: &mut [i16], lost: i32) {
    if ps_dec.fs_khz != ps_dec.s_plc.fs_khz {
        silk_plc_reset(ps_dec);
        ps_dec.s_plc.fs_khz = ps_dec.fs_khz;
    }

    if lost != 0 {
        silk_plc_conceal(ps_dec, ps_dec_ctrl, frame);
        ps_dec.loss_cnt += 1;
    } else {
        silk_plc_update(ps_dec, ps_dec_ctrl);
    }
}

/// Update state of PLC.
fn silk_plc_update(ps_dec: &mut SilkDecoderState, ps_dec_ctrl: &SilkDecoderControl) {
    /* Update parameters used in case of packet loss */
    ps_dec.prev_signal_type = ps_dec.indices.signal_type as i32;
    let mut ltp_gain_q14 = 0i32;
    if ps_dec.indices.signal_type as i32 == TYPE_VOICED {
        /* Find the parameters for the last subframe which contains a pitch pulse */
        let mut j = 0i32;
        while j * ps_dec.subfr_length < ps_dec_ctrl.pitch_l[(ps_dec.nb_subfr - 1) as usize] {
            if j == ps_dec.nb_subfr {
                break;
            }
            let mut temp_ltp_gain_q14 = 0i32;
            let row = ((ps_dec.nb_subfr - 1 - j) * LTP_ORDER as i32) as usize;
            for i in 0..LTP_ORDER {
                temp_ltp_gain_q14 += ps_dec_ctrl.ltp_coef_q14[row + i] as i32;
            }
            if temp_ltp_gain_q14 > ltp_gain_q14 {
                ltp_gain_q14 = temp_ltp_gain_q14;
                ps_dec.s_plc.ltp_coef_q14.copy_from_slice(&ps_dec_ctrl.ltp_coef_q14[row..row + LTP_ORDER]);

                ps_dec.s_plc.pitch_l_q8 = silk_lshift(ps_dec_ctrl.pitch_l[(ps_dec.nb_subfr - 1 - j) as usize], 8);
            }
            j += 1;
        }

        ps_dec.s_plc.ltp_coef_q14.fill(0);
        ps_dec.s_plc.ltp_coef_q14[LTP_ORDER / 2] = ltp_gain_q14 as i16;

        /* Limit LT coefs */
        if ltp_gain_q14 < V_PITCH_GAIN_START_MIN_Q14 {
            let tmp = silk_lshift(V_PITCH_GAIN_START_MIN_Q14, 10);
            let scale_q10 = tmp / ltp_gain_q14.max(1);
            for i in 0..LTP_ORDER {
                ps_dec.s_plc.ltp_coef_q14[i] = (silk_smulbb(ps_dec.s_plc.ltp_coef_q14[i] as i32, scale_q10) >> 10) as i16;
            }
        } else if ltp_gain_q14 > V_PITCH_GAIN_START_MAX_Q14 {
            let tmp = silk_lshift(V_PITCH_GAIN_START_MAX_Q14, 14);
            let scale_q14 = tmp / ltp_gain_q14.max(1);
            for i in 0..LTP_ORDER {
                ps_dec.s_plc.ltp_coef_q14[i] = (silk_smulbb(ps_dec.s_plc.ltp_coef_q14[i] as i32, scale_q14) >> 14) as i16;
            }
        }
    } else {
        ps_dec.s_plc.pitch_l_q8 = silk_lshift(silk_smulbb(ps_dec.fs_khz, 18), 8);
        ps_dec.s_plc.ltp_coef_q14.fill(0);
    }

    /* Save LPC coeficients */
    let lpc_order = ps_dec.lpc_order as usize;
    ps_dec.s_plc.prev_lpc_q12[..lpc_order].copy_from_slice(&ps_dec_ctrl.pred_coef_q12[1][..lpc_order]);
    ps_dec.s_plc.prev_ltp_scale_q14 = ps_dec_ctrl.ltp_scale_q14 as i16;

    /* Save last two gains */
    let nb = ps_dec.nb_subfr as usize;
    ps_dec.s_plc.prev_gain_q16.copy_from_slice(&ps_dec_ctrl.gains_q16[nb - 2..nb]);

    ps_dec.s_plc.subfr_length = ps_dec.subfr_length;
    ps_dec.s_plc.nb_subfr = ps_dec.nb_subfr;
}

fn silk_plc_conceal(ps_dec: &mut SilkDecoderState, ps_dec_ctrl: &mut SilkDecoderControl, frame: &mut [i16]) {
    let mut exc_buf = [0i16; 2 * MAX_SUB_FRAME_LENGTH];
    let mut a_q12 = [0i16; MAX_LPC_ORDER];
    let mut s_ltp = [0i16; MAX_FRAME_LENGTH];
    let mut s_ltp_q14 = [0i32; 2 * MAX_FRAME_LENGTH];
    let prev_gain_q10 = [ps_dec.s_plc.prev_gain_q16[0] >> 6, ps_dec.s_plc.prev_gain_q16[1] >> 6];

    if ps_dec.first_frame_after_reset != 0 {
        ps_dec.s_plc.prev_lpc_q12.fill(0);
    }

    /* Find random noise component */
    /* Scale previous excitation signal */
    let subfr_length = ps_dec.s_plc.subfr_length;
    let nb_subfr = ps_dec.s_plc.nb_subfr;
    let mut exc_buf_off = 0usize;
    for k in 0..2 {
        for i in 0..subfr_length as usize {
            exc_buf[exc_buf_off + i] = silk_sat16(
                silk_smulww(ps_dec.exc_q14[i + ((k + nb_subfr - 2) * subfr_length) as usize], prev_gain_q10[k as usize]) >> 8,
            ) as i16;
        }
        exc_buf_off += subfr_length as usize;
    }
    /* Find the subframe with lowest energy of the last two and use that as random noise generator */
    let mut energy1 = 0i32;
    let mut shift1 = 0i32;
    let mut energy2 = 0i32;
    let mut shift2 = 0i32;
    silk_sum_sqr_shift(&mut energy1, &mut shift1, &exc_buf[..subfr_length as usize]);
    silk_sum_sqr_shift(&mut energy2, &mut shift2, &exc_buf[subfr_length as usize..2 * subfr_length as usize]);

    let rand_off = if energy1 >> shift2 < energy2 >> shift1 {
        /* First sub-frame has lowest energy */
        ((nb_subfr - 1) * subfr_length - RAND_BUF_SIZE).max(0) as usize
    } else {
        /* Second sub-frame has lowest energy */
        (nb_subfr * subfr_length - RAND_BUF_SIZE).max(0) as usize
    };

    /* Set up Gain to random noise component */
    let mut rand_scale_q14 = ps_dec.s_plc.rand_scale_q14;

    /* Set up attenuation gains */
    let harm_gain_q15 = HARM_ATT_Q15[(NB_ATT as i32 - 1).min(ps_dec.loss_cnt) as usize] as i32;
    let mut rand_gain_q15 = if ps_dec.prev_signal_type == TYPE_VOICED {
        PLC_RAND_ATTENUATE_V_Q15[(NB_ATT as i32 - 1).min(ps_dec.loss_cnt) as usize] as i32
    } else {
        PLC_RAND_ATTENUATE_UV_Q15[(NB_ATT as i32 - 1).min(ps_dec.loss_cnt) as usize] as i32
    };

    /* LPC concealment. Apply BWE to previous LPC */
    let lpc_order = ps_dec.lpc_order as usize;
    silk_bwexpander(&mut ps_dec.s_plc.prev_lpc_q12[..lpc_order], BWE_COEF_Q16);

    /* Preload LPC coeficients to array on stack. Gives small performance gain */
    a_q12[..lpc_order].copy_from_slice(&ps_dec.s_plc.prev_lpc_q12[..lpc_order]);

    /* First Lost frame */
    if ps_dec.loss_cnt == 0 {
        rand_scale_q14 = 1 << 14;

        /* Reduce random noise Gain for voiced frames */
        if ps_dec.prev_signal_type == TYPE_VOICED {
            for i in 0..LTP_ORDER {
                rand_scale_q14 -= ps_dec.s_plc.ltp_coef_q14[i];
            }
            rand_scale_q14 = 3277i32.max(rand_scale_q14 as i32) as i16;
            rand_scale_q14 = (silk_smulbb(rand_scale_q14 as i32, ps_dec.s_plc.prev_ltp_scale_q14 as i32) >> 14) as i16;
        } else {
            /* Reduce random noise for unvoiced frames with high LPC gain */
            let inv_gain_q30 = silk_lpc_inverse_pred_gain(&ps_dec.s_plc.prev_lpc_q12[..lpc_order], lpc_order as i32);

            let mut down_scale_q30 = ((1 << 30) >> LOG2_INV_LPC_GAIN_HIGH_THRES).min(inv_gain_q30);
            down_scale_q30 = ((1 << 30) >> LOG2_INV_LPC_GAIN_LOW_THRES).max(down_scale_q30);
            down_scale_q30 = silk_lshift(down_scale_q30, LOG2_INV_LPC_GAIN_HIGH_THRES);

            rand_gain_q15 = silk_smulwb(down_scale_q30, rand_gain_q15) >> 14;
        }
    }

    let mut rand_seed = ps_dec.s_plc.rand_seed;
    let mut lag = silk_rshift_round(ps_dec.s_plc.pitch_l_q8, 8);
    let mut s_ltp_buf_idx = ps_dec.ltp_mem_length;

    /* Rewhiten LTP state */
    let idx = ps_dec.ltp_mem_length - lag - ps_dec.lpc_order - LTP_ORDER as i32 / 2;
    let len = ps_dec.ltp_mem_length - idx;
    silk_lpc_analysis_filter(
        &mut s_ltp[idx as usize..idx as usize + len as usize],
        &ps_dec.out_buf[idx as usize..idx as usize + len as usize],
        &a_q12[..lpc_order],
        len,
        ps_dec.lpc_order,
    );
    /* Scale LTP state */
    let mut inv_gain_q30 = silk_inverse32_varq(ps_dec.s_plc.prev_gain_q16[1], 46);
    inv_gain_q30 = inv_gain_q30.min(i32::MAX >> 1);
    let mut i = idx + ps_dec.lpc_order;
    while i < ps_dec.ltp_mem_length {
        s_ltp_q14[i as usize] = silk_smulwb(inv_gain_q30, s_ltp[i as usize] as i32);
        i += 1;
    }

    /***************************/
    /* LTP synthesis filtering */
    /***************************/
    let mut k = 0i32;
    while k < ps_dec.nb_subfr {
        /* Set up pointer */
        let mut pred_lag_off = (s_ltp_buf_idx - lag + LTP_ORDER as i32 / 2) as usize;
        for _ in 0..ps_dec.subfr_length {
            /* Unrolled loop */
            /* Avoids introducing a bias because silk_SMLAWB() always rounds to -inf */
            let mut ltp_pred_q12 = 2i32;
            ltp_pred_q12 = silk_smlawb(ltp_pred_q12, s_ltp_q14[pred_lag_off], ps_dec.s_plc.ltp_coef_q14[0] as i32);
            ltp_pred_q12 = silk_smlawb(ltp_pred_q12, s_ltp_q14[pred_lag_off - 1], ps_dec.s_plc.ltp_coef_q14[1] as i32);
            ltp_pred_q12 = silk_smlawb(ltp_pred_q12, s_ltp_q14[pred_lag_off - 2], ps_dec.s_plc.ltp_coef_q14[2] as i32);
            ltp_pred_q12 = silk_smlawb(ltp_pred_q12, s_ltp_q14[pred_lag_off - 3], ps_dec.s_plc.ltp_coef_q14[3] as i32);
            ltp_pred_q12 = silk_smlawb(ltp_pred_q12, s_ltp_q14[pred_lag_off - 4], ps_dec.s_plc.ltp_coef_q14[4] as i32);
            pred_lag_off += 1;

            /* Generate LPC excitation */
            rand_seed = 907633515i32.wrapping_add(rand_seed.wrapping_mul(196314165));
            let ridx = ((rand_seed >> 25) & RAND_BUF_MASK) as usize;
            s_ltp_q14[s_ltp_buf_idx as usize] =
                silk_lshift(silk_smlawb(ltp_pred_q12, ps_dec.exc_q14[rand_off + ridx], rand_scale_q14 as i32), 2);
            s_ltp_buf_idx += 1;
        }

        /* Gradually reduce LTP gain */
        for j in 0..LTP_ORDER {
            ps_dec.s_plc.ltp_coef_q14[j] = (silk_smulbb(harm_gain_q15, ps_dec.s_plc.ltp_coef_q14[j] as i32) >> 15) as i16;
        }
        /* Gradually reduce excitation gain */
        rand_scale_q14 = (silk_smulbb(rand_scale_q14 as i32, rand_gain_q15) >> 15) as i16;

        /* Slowly increase pitch lag */
        ps_dec.s_plc.pitch_l_q8 = silk_smlawb(ps_dec.s_plc.pitch_l_q8, ps_dec.s_plc.pitch_l_q8, PITCH_DRIFT_FAC_Q16);
        ps_dec.s_plc.pitch_l_q8 = ps_dec.s_plc.pitch_l_q8.min(silk_lshift(silk_smulbb(MAX_PITCH_LAG_MS, ps_dec.fs_khz), 8));
        lag = silk_rshift_round(ps_dec.s_plc.pitch_l_q8, 8);
        k += 1;
    }

    /***************************/
    /* LPC synthesis filtering */
    /***************************/
    let s_lpc_off = (ps_dec.ltp_mem_length - MAX_LPC_ORDER as i32) as usize;

    /* Copy LPC state */
    s_ltp_q14[s_lpc_off..s_lpc_off + MAX_LPC_ORDER].copy_from_slice(&ps_dec.s_lpc_q14_buf);

    assert_eq!(frame.len(), ps_dec.frame_length as usize);
    for (i, out) in frame.iter_mut().enumerate() {
        /* partly unrolled */
        /* Avoids introducing a bias because silk_SMLAWB() always rounds to -inf */
        let base = s_lpc_off + MAX_LPC_ORDER + i;
        let mut lpc_pred_q10 = ps_dec.lpc_order >> 1;
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
        for j in 10..ps_dec.lpc_order as usize {
            lpc_pred_q10 = silk_smlawb(lpc_pred_q10, s_ltp_q14[base - j - 1], a_q12[j] as i32);
        }

        /* Add prediction to LPC excitation */
        s_ltp_q14[base] += silk_lshift(lpc_pred_q10, 4);

        /* Scale with Gain */
        *out = silk_sat16(silk_sat16(silk_rshift_round(silk_smulww(s_ltp_q14[base], prev_gain_q10[1]), 8))) as i16;
    }

    /* Save LPC state */
    let frame_len = ps_dec.frame_length as usize;
    ps_dec.s_lpc_q14_buf.copy_from_slice(&s_ltp_q14[s_lpc_off + frame_len..s_lpc_off + frame_len + MAX_LPC_ORDER]);

    /**************************************/
    /* Update states                      */
    /**************************************/
    ps_dec.s_plc.rand_seed = rand_seed;
    ps_dec.s_plc.rand_scale_q14 = rand_scale_q14;
    for i in 0..MAX_NB_SUBFR {
        ps_dec_ctrl.pitch_l[i] = lag;
    }
}

/// `silk_PLC_glue_frames` — glue concealed frames with the next good frame.
pub fn silk_plc_glue_frames(ps_dec: &mut SilkDecoderState, frame: &mut [i16]) {
    let length = frame.len() as i32;
    let ps_plc = &mut ps_dec.s_plc;

    if ps_dec.loss_cnt != 0 {
        /* Calculate energy in concealed residual */
        let mut energy = 0i32;
        let mut energy_shift = 0i32;
        silk_sum_sqr_shift(&mut energy, &mut energy_shift, frame);
        ps_plc.conc_energy = energy;
        ps_plc.conc_energy_shift = energy_shift;
        ps_plc.last_frame_lost = 1;
    } else {
        if ps_plc.last_frame_lost != 0 {
            /* Calculate residual in decoded signal if last frame was lost */
            let mut energy = 0i32;
            let mut energy_shift = 0i32;
            silk_sum_sqr_shift(&mut energy, &mut energy_shift, frame);

            /* Normalize energies */
            if energy_shift > ps_plc.conc_energy_shift {
                ps_plc.conc_energy >>= energy_shift - ps_plc.conc_energy_shift;
            } else if energy_shift < ps_plc.conc_energy_shift {
                energy >>= ps_plc.conc_energy_shift - energy_shift;
            }

            /* Fade in the energy difference */
            if energy > ps_plc.conc_energy {
                let mut lz = silk_clz32(ps_plc.conc_energy);
                lz -= 1;
                ps_plc.conc_energy = silk_lshift(ps_plc.conc_energy, lz);
                energy >>= (24 - lz).max(0);

                let frac_q24 = ps_plc.conc_energy / energy.max(1);

                let mut gain_q16 = silk_lshift(silk_sqrt_approx(frac_q24), 4);
                let mut slope_q16 = ((1 << 16) - gain_q16) / length;
                /* Make slope 4x steeper to avoid missing onsets after DTX */
                slope_q16 = silk_lshift(slope_q16, 2);

                for slot in frame.iter_mut() {
                    *slot = silk_smulwb(gain_q16, *slot as i32) as i16;
                    gain_q16 += slope_q16;
                    if gain_q16 > 1 << 16 {
                        break;
                    }
                }
            }
        }
        ps_plc.last_frame_lost = 0;
    }
}
