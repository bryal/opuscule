//! Translated from `c/silk/CNG.c` (RFC 6716).
//!
//! Comfort-noise generation: smooths NLSF and gain estimates from the
//! most recent unvoiced frame and, during packet loss, synthesizes a
//! noise signal that is mixed into the decoder output.

use super::NLSF2A::silk_nlsf2a;
use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb, silk_smulww};
use super::structs::{MAX_FRAME_LENGTH, MAX_LPC_ORDER, SilkDecoderControl, SilkDecoderState};

const CNG_BUF_MASK_MAX: i32 = 255;
const CNG_GAIN_SMTH_Q16: i32 = 4634;
const CNG_NLSF_SMTH_Q16: i32 = 16348;
const TYPE_NO_VOICE_ACTIVITY: i32 = 0;
const SILK_INT16_MAX: i32 = i16::MAX as i32;

/// `silk_CNG_exc` — generate excitation for CNG LPC synthesis.
fn silk_cng_exc(residual_q10: &mut [i32], exc_buf_q14: &[i32], gain_q16: i32, rand_seed: &mut i32) {
    let length = residual_q10.len() as i32;
    let mut exc_mask = CNG_BUF_MASK_MAX;
    while exc_mask > length {
        exc_mask >>= 1;
    }

    let mut seed = *rand_seed;
    for slot in residual_q10.iter_mut() {
        seed = 907633515i32.wrapping_add(seed.wrapping_mul(196314165));
        let idx = ((seed >> 24) & exc_mask) as usize;
        *slot = silk_sat16(silk_smulww(exc_buf_q14[idx], gain_q16 >> 4)) as i16 as i32;
    }
    *rand_seed = seed;
}

/// `silk_CNG_Reset` — reset CNG state.
pub fn silk_cng_reset(ps_dec: &mut SilkDecoderState) {
    let nlsf_step_q15 = SILK_INT16_MAX / (ps_dec.lpc_order + 1);
    let mut nlsf_acc_q15 = 0i32;
    let mut i = 0i32;
    while i < ps_dec.lpc_order {
        nlsf_acc_q15 += nlsf_step_q15;
        ps_dec.s_cng.cng_smth_nlsf_q15[i as usize] = nlsf_acc_q15 as i16;
        i += 1;
    }
    ps_dec.s_cng.cng_smth_gain_q16 = 0;
    ps_dec.s_cng.rand_seed = 3176576;
}

/// `silk_CNG` — update CNG estimate, apply CNG when packet was lost.
pub fn silk_cng(ps_dec: &mut SilkDecoderState, ps_dec_ctrl: &SilkDecoderControl, frame: &mut [i16]) {
    let length = frame.len() as i32;
    let mut a_q12 = [0i16; MAX_LPC_ORDER];
    let mut cng_sig_q10 = [0i32; MAX_FRAME_LENGTH + MAX_LPC_ORDER];

    if ps_dec.fs_khz != ps_dec.s_cng.fs_khz {
        /* Reset state */
        silk_cng_reset(ps_dec);
        ps_dec.s_cng.fs_khz = ps_dec.fs_khz;
    }
    if ps_dec.loss_cnt == 0 && ps_dec.prev_signal_type == TYPE_NO_VOICE_ACTIVITY {
        /* Update CNG parameters */

        /* Smoothing of LSF's */
        let lpc_order = ps_dec.lpc_order;
        for i in 0..lpc_order as usize {
            ps_dec.s_cng.cng_smth_nlsf_q15[i] = (ps_dec.s_cng.cng_smth_nlsf_q15[i] as i32
                + silk_smulwb(ps_dec.prev_nlsf_q15[i] as i32 - ps_dec.s_cng.cng_smth_nlsf_q15[i] as i32, CNG_NLSF_SMTH_Q16))
                as i16;
        }
        /* Find the subframe with the highest gain */
        let mut max_gain_q16 = 0i32;
        let mut subfr = 0usize;
        for i in 0..ps_dec.nb_subfr as usize {
            if ps_dec_ctrl.gains_q16[i] > max_gain_q16 {
                max_gain_q16 = ps_dec_ctrl.gains_q16[i];
                subfr = i;
            }
        }
        /* Update CNG excitation buffer with excitation from this subframe */
        let subfr_len = ps_dec.subfr_length as usize;
        let nb = ps_dec.nb_subfr as usize;
        // In bounds: dest end subfr_len + (nb - 1)*subfr_len == nb*subfr_len == frame_length
        // <= MAX_FRAME_LENGTH == cng_exc_buf_q14.len().
        ps_dec.s_cng.cng_exc_buf_q14.copy_within(0..(nb - 1) * subfr_len, subfr_len);
        ps_dec.s_cng.cng_exc_buf_q14[..subfr_len].copy_from_slice(&ps_dec.exc_q14[subfr * subfr_len..(subfr + 1) * subfr_len]);

        /* Smooth gains */
        for i in 0..ps_dec.nb_subfr as usize {
            ps_dec.s_cng.cng_smth_gain_q16 +=
                silk_smulwb(ps_dec_ctrl.gains_q16[i] - ps_dec.s_cng.cng_smth_gain_q16, CNG_GAIN_SMTH_Q16);
        }
    }

    /* Add CNG when packet is lost or during DTX */
    if ps_dec.loss_cnt != 0 {
        /* Generate CNG excitation */
        silk_cng_exc(
            &mut cng_sig_q10[MAX_LPC_ORDER..MAX_LPC_ORDER + length as usize],
            &ps_dec.s_cng.cng_exc_buf_q14,
            ps_dec.s_cng.cng_smth_gain_q16,
            &mut ps_dec.s_cng.rand_seed,
        );

        /* Convert CNG NLSF to filter representation */
        silk_nlsf2a(&mut a_q12, &ps_dec.s_cng.cng_smth_nlsf_q15, ps_dec.lpc_order);

        /* Generate CNG signal, by synthesis filtering */
        cng_sig_q10[..MAX_LPC_ORDER].copy_from_slice(&ps_dec.s_cng.cng_synth_state);
        for i in 0..length as usize {
            /* Avoids introducing a bias because silk_SMLAWB() always rounds to -inf */
            let mut sum_q6 = ps_dec.lpc_order >> 1;
            sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 1], a_q12[0] as i32);
            sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 2], a_q12[1] as i32);
            sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 3], a_q12[2] as i32);
            sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 4], a_q12[3] as i32);
            sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 5], a_q12[4] as i32);
            sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 6], a_q12[5] as i32);
            sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 7], a_q12[6] as i32);
            sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 8], a_q12[7] as i32);
            sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 9], a_q12[8] as i32);
            sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 10], a_q12[9] as i32);
            if ps_dec.lpc_order == 16 {
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 11], a_q12[10] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 12], a_q12[11] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 13], a_q12[12] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 14], a_q12[13] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 15], a_q12[14] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i - 16], a_q12[15] as i32);
            }

            /* Update states */
            cng_sig_q10[MAX_LPC_ORDER + i] += silk_lshift(sum_q6, 4);

            frame[i] = frame[i].saturating_add(silk_rshift_round(sum_q6, 6) as i16);
        }
        ps_dec.s_cng.cng_synth_state.copy_from_slice(&cng_sig_q10[length as usize..length as usize + MAX_LPC_ORDER]);
    } else {
        ps_dec.s_cng.cng_synth_state[..ps_dec.lpc_order as usize].fill(0);
    }
}
