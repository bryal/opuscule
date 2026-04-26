//! Translated from `c/silk/CNG.c` (RFC 6716).
//!
//! Comfort-noise generation: smooths NLSF and gain estimates from the
//! most recent unvoiced frame and, during packet loss, synthesizes a
//! noise signal that is mixed into the decoder output.

use super::NLSF2A::silk_NLSF2A;
use super::macros::{
    silk_add_lshift, silk_div32_16, silk_mla_ovflw, silk_rshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb,
    silk_smulww,
};
use super::structs::{MAX_FRAME_LENGTH, MAX_LPC_ORDER, SilkDecoderControl, SilkDecoderState};

const CNG_BUF_MASK_MAX: i32 = 255;
const CNG_GAIN_SMTH_Q16: i32 = 4634;
const CNG_NLSF_SMTH_Q16: i32 = 16348;
const TYPE_NO_VOICE_ACTIVITY: i32 = 0;
const SILK_INT16_MAX: i32 = i16::MAX as i32;

/// `silk_CNG_exc` — generate excitation for CNG LPC synthesis.
unsafe fn silk_CNG_exc(residual_q10: *mut i32, exc_buf_q14: *const i32, gain_q16: i32, length: i32, rand_seed: *mut i32) {
    unsafe {
        let mut exc_mask = CNG_BUF_MASK_MAX;
        while exc_mask > length {
            exc_mask = silk_rshift(exc_mask, 1);
        }

        let mut seed = *rand_seed;
        let mut i = 0i32;
        while i < length {
            seed = silk_mla_ovflw(907633515, seed, 196314165);
            let idx = silk_rshift(seed, 24) & exc_mask;
            *residual_q10.offset(i as isize) =
                silk_sat16(silk_smulww(*exc_buf_q14.offset(idx as isize), gain_q16 >> 4)) as i16 as i32;
            i += 1;
        }
        *rand_seed = seed;
    }
}

/// `silk_CNG_Reset` — reset CNG state.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_CNG_Reset(ps_dec: *mut SilkDecoderState) {
    unsafe {
        let nlsf_step_q15 = silk_div32_16(SILK_INT16_MAX, (*ps_dec).lpc_order + 1);
        let mut nlsf_acc_q15 = 0i32;
        let mut i = 0i32;
        while i < (*ps_dec).lpc_order {
            nlsf_acc_q15 += nlsf_step_q15;
            (*ps_dec).s_cng.cng_smth_nlsf_q15[i as usize] = nlsf_acc_q15 as i16;
            i += 1;
        }
        (*ps_dec).s_cng.cng_smth_gain_q16 = 0;
        (*ps_dec).s_cng.rand_seed = 3176576;
    }
}

/// `silk_CNG` — update CNG estimate, apply CNG when packet was lost.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_CNG(
    ps_dec: *mut SilkDecoderState,
    ps_dec_ctrl: *mut SilkDecoderControl,
    frame: *mut i16,
    length: i32,
) {
    unsafe {
        let mut a_q12 = [0i16; MAX_LPC_ORDER];
        let mut cng_sig_q10 = [0i32; MAX_FRAME_LENGTH + MAX_LPC_ORDER];
        let ps_cng = &raw mut (*ps_dec).s_cng;

        if (*ps_dec).fs_khz != (*ps_cng).fs_khz {
            /* Reset state */
            silk_CNG_Reset(ps_dec);
            (*ps_cng).fs_khz = (*ps_dec).fs_khz;
        }
        if (*ps_dec).loss_cnt == 0 && (*ps_dec).prev_signal_type == TYPE_NO_VOICE_ACTIVITY {
            /* Update CNG parameters */

            /* Smoothing of LSF's */
            let mut i = 0i32;
            while i < (*ps_dec).lpc_order {
                (*ps_cng).cng_smth_nlsf_q15[i as usize] = ((*ps_cng).cng_smth_nlsf_q15[i as usize] as i32
                    + silk_smulwb(
                        (*ps_dec).prev_nlsf_q15[i as usize] as i32 - (*ps_cng).cng_smth_nlsf_q15[i as usize] as i32,
                        CNG_NLSF_SMTH_Q16,
                    )) as i16;
                i += 1;
            }
            /* Find the subframe with the highest gain */
            let mut max_gain_q16 = 0i32;
            let mut subfr = 0i32;
            let mut i = 0i32;
            while i < (*ps_dec).nb_subfr {
                if (*ps_dec_ctrl).gains_q16[i as usize] > max_gain_q16 {
                    max_gain_q16 = (*ps_dec_ctrl).gains_q16[i as usize];
                    subfr = i;
                }
                i += 1;
            }
            /* Update CNG excitation buffer with excitation from this subframe */
            let subfr_len = (*ps_dec).subfr_length as usize;
            let nb = (*ps_dec).nb_subfr as usize;
            core::ptr::copy(
                (*ps_cng).cng_exc_buf_q14.as_ptr(),
                (*ps_cng).cng_exc_buf_q14.as_mut_ptr().add(subfr_len),
                (nb - 1) * subfr_len,
            );
            core::ptr::copy_nonoverlapping(
                (*ps_dec).exc_q14.as_ptr().add((subfr as usize) * subfr_len),
                (*ps_cng).cng_exc_buf_q14.as_mut_ptr(),
                subfr_len,
            );

            /* Smooth gains */
            let mut i = 0i32;
            while i < (*ps_dec).nb_subfr {
                (*ps_cng).cng_smth_gain_q16 +=
                    silk_smulwb((*ps_dec_ctrl).gains_q16[i as usize] - (*ps_cng).cng_smth_gain_q16, CNG_GAIN_SMTH_Q16);
                i += 1;
            }
        }

        /* Add CNG when packet is lost or during DTX */
        if (*ps_dec).loss_cnt != 0 {
            /* Generate CNG excitation */
            silk_CNG_exc(
                cng_sig_q10.as_mut_ptr().add(MAX_LPC_ORDER),
                (*ps_cng).cng_exc_buf_q14.as_ptr(),
                (*ps_cng).cng_smth_gain_q16,
                length,
                &raw mut (*ps_cng).rand_seed,
            );

            /* Convert CNG NLSF to filter representation */
            silk_NLSF2A(a_q12.as_mut_ptr(), (*ps_cng).cng_smth_nlsf_q15.as_ptr(), (*ps_dec).lpc_order);

            /* Generate CNG signal, by synthesis filtering */
            core::ptr::copy_nonoverlapping((*ps_cng).cng_synth_state.as_ptr(), cng_sig_q10.as_mut_ptr(), MAX_LPC_ORDER);
            let mut i = 0i32;
            while i < length {
                /* Avoids introducing a bias because silk_SMLAWB() always rounds to -inf */
                let mut sum_q6 = silk_rshift((*ps_dec).lpc_order, 1);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 1], a_q12[0] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 2], a_q12[1] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 3], a_q12[2] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 4], a_q12[3] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 5], a_q12[4] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 6], a_q12[5] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 7], a_q12[6] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 8], a_q12[7] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 9], a_q12[8] as i32);
                sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 10], a_q12[9] as i32);
                if (*ps_dec).lpc_order == 16 {
                    sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 11], a_q12[10] as i32);
                    sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 12], a_q12[11] as i32);
                    sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 13], a_q12[12] as i32);
                    sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 14], a_q12[13] as i32);
                    sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 15], a_q12[14] as i32);
                    sum_q6 = silk_smlawb(sum_q6, cng_sig_q10[MAX_LPC_ORDER + i as usize - 16], a_q12[15] as i32);
                }

                /* Update states */
                cng_sig_q10[MAX_LPC_ORDER + i as usize] = silk_add_lshift(cng_sig_q10[MAX_LPC_ORDER + i as usize], sum_q6, 4);

                *frame.offset(i as isize) = (*frame.offset(i as isize)).saturating_add(silk_rshift_round(sum_q6, 6) as i16);
                i += 1;
            }
            core::ptr::copy_nonoverlapping(
                cng_sig_q10.as_ptr().add(length as usize),
                (*ps_cng).cng_synth_state.as_mut_ptr(),
                MAX_LPC_ORDER,
            );
        } else {
            core::ptr::write_bytes((*ps_cng).cng_synth_state.as_mut_ptr(), 0, (*ps_dec).lpc_order as usize);
        }
    }
}
