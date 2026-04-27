//! Translated from `c/silk/stereo_MS_to_LR.c` (RFC 6716).
//!
//! Converts adaptive Mid/Side representation to Left/Right stereo signal.

use super::macros::{silk_add_lshift32, silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulbb};
use super::structs::StereoDecState;

const STEREO_INTERP_LEN_MS: i32 = 8;

/// `silk_stereo_MS_to_LR` — convert mid/side to left/right stereo.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_stereo_MS_to_LR(
    state: *mut StereoDecState,
    x1: *mut i16,
    x2: *mut i16,
    pred_q13: *const i32,
    fs_khz: i32,
    frame_length: i32,
) {
    unsafe {
        /* Buffering */
        core::ptr::copy_nonoverlapping((*state).s_mid.as_ptr(), x1, 2);
        core::ptr::copy_nonoverlapping((*state).s_side.as_ptr(), x2, 2);
        core::ptr::copy_nonoverlapping(x1.offset(frame_length as isize), (*state).s_mid.as_mut_ptr(), 2);
        core::ptr::copy_nonoverlapping(x2.offset(frame_length as isize), (*state).s_side.as_mut_ptr(), 2);

        /* Interpolate predictors and add prediction to side channel */
        let mut pred0_q13 = (*state).pred_prev_q13[0] as i32;
        let mut pred1_q13 = (*state).pred_prev_q13[1] as i32;
        let denom_q16 = (1 << 16) / (STEREO_INTERP_LEN_MS * fs_khz);
        let delta0_q13 = silk_rshift_round(silk_smulbb(*pred_q13.offset(0) - (*state).pred_prev_q13[0] as i32, denom_q16), 16);
        let delta1_q13 = silk_rshift_round(silk_smulbb(*pred_q13.offset(1) - (*state).pred_prev_q13[1] as i32, denom_q16), 16);
        let mut n = 0;
        while n < STEREO_INTERP_LEN_MS * fs_khz {
            pred0_q13 += delta0_q13;
            pred1_q13 += delta1_q13;
            let sum = silk_lshift(
                silk_add_lshift32(
                    *x1.offset(n as isize) as i32 + *x1.offset((n + 2) as isize) as i32,
                    *x1.offset((n + 1) as isize) as i32,
                    1,
                ),
                9,
            ); /* Q11 */
            let sum = silk_smlawb(silk_lshift(*x2.offset((n + 1) as isize) as i32, 8), sum, pred0_q13); /* Q8 */
            let sum = silk_smlawb(sum, silk_lshift(*x1.offset((n + 1) as isize) as i32, 11), pred1_q13); /* Q8 */
            *x2.offset((n + 1) as isize) = silk_sat16(silk_rshift_round(sum, 8)) as i16;
            n += 1;
        }
        let pred0_q13 = *pred_q13.offset(0);
        let pred1_q13 = *pred_q13.offset(1);
        while n < frame_length {
            let sum = silk_lshift(
                silk_add_lshift32(
                    *x1.offset(n as isize) as i32 + *x1.offset((n + 2) as isize) as i32,
                    *x1.offset((n + 1) as isize) as i32,
                    1,
                ),
                9,
            ); /* Q11 */
            let sum = silk_smlawb(silk_lshift(*x2.offset((n + 1) as isize) as i32, 8), sum, pred0_q13); /* Q8 */
            let sum = silk_smlawb(sum, silk_lshift(*x1.offset((n + 1) as isize) as i32, 11), pred1_q13); /* Q8 */
            *x2.offset((n + 1) as isize) = silk_sat16(silk_rshift_round(sum, 8)) as i16;
            n += 1;
        }
        (*state).pred_prev_q13[0] = *pred_q13.offset(0) as i16;
        (*state).pred_prev_q13[1] = *pred_q13.offset(1) as i16;

        /* Convert to left/right signals */
        let mut n = 0;
        while n < frame_length {
            let sum = *x1.offset((n + 1) as isize) as i32 + *x2.offset((n + 1) as isize) as i32;
            let diff = *x1.offset((n + 1) as isize) as i32 - *x2.offset((n + 1) as isize) as i32;
            *x1.offset((n + 1) as isize) = silk_sat16(sum) as i16;
            *x2.offset((n + 1) as isize) = silk_sat16(diff) as i16;
            n += 1;
        }
    }
}
