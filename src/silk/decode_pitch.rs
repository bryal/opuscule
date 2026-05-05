//! Translated from `c/silk/decode_pitch.c` (RFC 6716).
//!
//! Reconstructs per-subframe pitch lags from the coded lag index and
//! contour index using the pitch estimation codebook tables.

use super::pitch_est_tables::{SILK_CB_LAGS_STAGE2, SILK_CB_LAGS_STAGE2_10_MS, SILK_CB_LAGS_STAGE3, SILK_CB_LAGS_STAGE3_10_MS};

// Constants from pitch_est_defines.h
const PE_MAX_NB_SUBFR: i32 = 4;
const PE_NB_CBKS_STAGE2_EXT: i32 = 11;
const PE_NB_CBKS_STAGE2_10MS: i32 = 3;
const PE_NB_CBKS_STAGE3_MAX: i32 = 34;
const PE_NB_CBKS_STAGE3_10MS: i32 = 12;
const PE_MIN_LAG_MS: i32 = 2;
const PE_MAX_LAG_MS: i32 = 18;

/// `silk_decode_pitch` — decode pitch lags from lag index + contour index.
pub unsafe fn silk_decode_pitch(lag_index: i16, contour_index: i8, pitch_lags: *mut i32, fs_khz: i32, nb_subfr: i32) {
    unsafe {
        let (lag_cb_ptr, cbk_size): (*const i8, i32) = if fs_khz == 8 {
            if nb_subfr == PE_MAX_NB_SUBFR {
                (SILK_CB_LAGS_STAGE2.as_ptr() as *const i8, PE_NB_CBKS_STAGE2_EXT)
            } else {
                (SILK_CB_LAGS_STAGE2_10_MS.as_ptr() as *const i8, PE_NB_CBKS_STAGE2_10MS)
            }
        } else {
            if nb_subfr == PE_MAX_NB_SUBFR {
                (SILK_CB_LAGS_STAGE3.as_ptr() as *const i8, PE_NB_CBKS_STAGE3_MAX)
            } else {
                (SILK_CB_LAGS_STAGE3_10_MS.as_ptr() as *const i8, PE_NB_CBKS_STAGE3_10MS)
            }
        };

        let min_lag = PE_MIN_LAG_MS * fs_khz;
        let max_lag = PE_MAX_LAG_MS * fs_khz;
        let lag = min_lag + lag_index as i32;

        let mut k = 0;
        while k < nb_subfr {
            /* matrix_ptr(Lag_CB_ptr, k, contourIndex, cbk_size) */
            let cb_val = *lag_cb_ptr.offset((k * cbk_size + contour_index as i32) as isize);
            let mut pl = lag + cb_val as i32;
            if pl < min_lag {
                pl = min_lag;
            } else if pl > max_lag {
                pl = max_lag;
            }
            *pitch_lags.offset(k as isize) = pl;
            k += 1;
        }
    }
}
