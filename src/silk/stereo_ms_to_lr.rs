//! Translated from `c/silk/stereo_MS_to_LR.c` (RFC 6716).
//!
//! Converts adaptive Mid/Side representation to Left/Right stereo signal.

#![allow(clippy::indexing_slicing)] // dense SILK kernels; voice path is deprioritized vs CELT

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulbb};
use super::structs::StereoDecState;

const STEREO_INTERP_LEN_MS: i32 = 8;

/// `silk_stereo_MS_to_LR` — convert mid/side to left/right stereo.
pub fn silk_stereo_ms_to_lr(
    state: &mut StereoDecState,
    x1: &mut [i16],
    x2: &mut [i16],
    pred_q13: &[i32],
    fs_khz: i32,
    frame_length: i32,
) {
    let frame_length = frame_length as usize;

    /* Buffering */
    x1[..2].copy_from_slice(&state.s_mid);
    x2[..2].copy_from_slice(&state.s_side);
    state.s_mid.copy_from_slice(&x1[frame_length..frame_length + 2]);
    state.s_side.copy_from_slice(&x2[frame_length..frame_length + 2]);

    /* Interpolate predictors and add prediction to side channel */
    let mut pred0_q13 = state.pred_prev_q13[0] as i32;
    let mut pred1_q13 = state.pred_prev_q13[1] as i32;
    let denom_q16 = (1 << 16) / (STEREO_INTERP_LEN_MS * fs_khz);
    let delta0_q13 = silk_rshift_round(silk_smulbb(pred_q13[0] - state.pred_prev_q13[0] as i32, denom_q16), 16);
    let delta1_q13 = silk_rshift_round(silk_smulbb(pred_q13[1] - state.pred_prev_q13[1] as i32, denom_q16), 16);
    let mut n = 0usize;
    while n < (STEREO_INTERP_LEN_MS * fs_khz) as usize {
        pred0_q13 += delta0_q13;
        pred1_q13 += delta1_q13;
        let sum = silk_lshift(x1[n] as i32 + x1[n + 2] as i32 + silk_lshift(x1[n + 1] as i32, 1), 9); /* Q11 */
        let sum = silk_smlawb(silk_lshift(x2[n + 1] as i32, 8), sum, pred0_q13); /* Q8 */
        let sum = silk_smlawb(sum, silk_lshift(x1[n + 1] as i32, 11), pred1_q13); /* Q8 */
        x2[n + 1] = silk_sat16(silk_rshift_round(sum, 8)) as i16;
        n += 1;
    }
    let pred0_q13 = pred_q13[0];
    let pred1_q13 = pred_q13[1];
    while n < frame_length {
        let sum = silk_lshift(x1[n] as i32 + x1[n + 2] as i32 + silk_lshift(x1[n + 1] as i32, 1), 9); /* Q11 */
        let sum = silk_smlawb(silk_lshift(x2[n + 1] as i32, 8), sum, pred0_q13); /* Q8 */
        let sum = silk_smlawb(sum, silk_lshift(x1[n + 1] as i32, 11), pred1_q13); /* Q8 */
        x2[n + 1] = silk_sat16(silk_rshift_round(sum, 8)) as i16;
        n += 1;
    }
    state.pred_prev_q13[0] = pred_q13[0] as i16;
    state.pred_prev_q13[1] = pred_q13[1] as i16;

    /* Convert to left/right signals */
    let mut n = 0usize;
    while n < frame_length {
        let sum = x1[n + 1] as i32 + x2[n + 1] as i32;
        let diff = x1[n + 1] as i32 - x2[n + 1] as i32;
        x1[n + 1] = silk_sat16(sum) as i16;
        x2[n + 1] = silk_sat16(diff) as i16;
        n += 1;
    }
}
