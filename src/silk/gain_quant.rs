//! Translated from `c/silk/gain_quant.c` (RFC 6716).
//!
//! Gain scalar quantization / dequantization on a log scale with
//! hysteresis and delta coding.

use super::lin2log::silk_lin2log;
use super::log2lin::silk_log2lin;
use super::macros::{silk_add_lshift32, silk_limit_int, silk_lshift, silk_max_int, silk_min_32, silk_rshift, silk_smulwb};

// -- Constants from c/silk/define.h --

const MIN_QGAIN_DB: i32 = 2;
const MAX_QGAIN_DB: i32 = 88;
const N_LEVELS_QGAIN: i32 = 64;
const MAX_DELTA_GAIN_QUANT: i32 = 36;
const MIN_DELTA_GAIN_QUANT: i32 = -4;

// -- Derived constants matching the C #defines --

const OFFSET: i32 = (MIN_QGAIN_DB * 128) / 6 + 16 * 128;
const SCALE_Q16: i32 = (65536 * (N_LEVELS_QGAIN - 1)) / (((MAX_QGAIN_DB - MIN_QGAIN_DB) * 128) / 6);
const INV_SCALE_Q16: i32 = (65536 * (((MAX_QGAIN_DB - MIN_QGAIN_DB) * 128) / 6)) / (N_LEVELS_QGAIN - 1);

/// `silk_gains_quant` — quantize gain values (encoder-side, but also used in
/// the decoder for gain parameter handling).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_gains_quant(
    ind: *mut i8,
    gain_q16: *mut i32,
    prev_ind: *mut i8,
    conditional: i32,
    nb_subfr: i32,
) {
    unsafe {
        let mut k = 0;
        while k < nb_subfr {
            /* Convert to log scale, scale, floor() */
            *ind.offset(k as isize) = silk_smulwb(SCALE_Q16, silk_lin2log(*gain_q16.offset(k as isize)) - OFFSET) as i8;

            /* Round towards previous quantized gain (hysteresis) */
            if (*ind.offset(k as isize) as i32) < *prev_ind as i32 {
                *ind.offset(k as isize) = (*ind.offset(k as isize)).wrapping_add(1);
            }
            *ind.offset(k as isize) = silk_limit_int(*ind.offset(k as isize) as i32, 0, N_LEVELS_QGAIN - 1) as i8;

            /* Compute delta indices and limit */
            if k == 0 && conditional == 0 {
                /* Full index */
                *ind.offset(k as isize) =
                    silk_limit_int(*ind.offset(k as isize) as i32, *prev_ind as i32 + MIN_DELTA_GAIN_QUANT, N_LEVELS_QGAIN - 1)
                        as i8;
                *prev_ind = *ind.offset(k as isize);
            } else {
                /* Delta index */
                *ind.offset(k as isize) = (*ind.offset(k as isize) as i32 - *prev_ind as i32) as i8;

                /* Double the quantization step size for large gain increases, so that the max gain level can be reached */
                let double_step_size_threshold = 2 * MAX_DELTA_GAIN_QUANT - N_LEVELS_QGAIN + *prev_ind as i32;
                if (*ind.offset(k as isize) as i32) > double_step_size_threshold {
                    *ind.offset(k as isize) = (double_step_size_threshold
                        + silk_rshift(*ind.offset(k as isize) as i32 - double_step_size_threshold + 1, 1))
                        as i8;
                }

                *ind.offset(k as isize) =
                    silk_limit_int(*ind.offset(k as isize) as i32, MIN_DELTA_GAIN_QUANT, MAX_DELTA_GAIN_QUANT) as i8;

                /* Accumulate deltas */
                if (*ind.offset(k as isize) as i32) > double_step_size_threshold {
                    *prev_ind =
                        (*prev_ind as i32 + silk_lshift(*ind.offset(k as isize) as i32, 1) - double_step_size_threshold) as i8;
                } else {
                    *prev_ind = (*prev_ind as i32 + *ind.offset(k as isize) as i32) as i8;
                }

                /* Shift to make non-negative */
                *ind.offset(k as isize) = (*ind.offset(k as isize) as i32 - MIN_DELTA_GAIN_QUANT) as i8;
            }

            /* Scale and convert to linear scale */
            *gain_q16.offset(k as isize) =
                silk_log2lin(silk_min_32(silk_smulwb(INV_SCALE_Q16, *prev_ind as i32) + OFFSET, 3967)); /* 3967 = 31 in Q7 */
            k += 1;
        }
    }
}

/// `silk_gains_dequant` — dequantize gain indices back to linear Q16 gains.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_gains_dequant(
    gain_q16: *mut i32,
    ind: *const i8,
    prev_ind: *mut i8,
    conditional: i32,
    nb_subfr: i32,
) {
    unsafe {
        let mut k = 0;
        while k < nb_subfr {
            if k == 0 && conditional == 0 {
                /* Gain index is not allowed to go down more than 16 steps (~21.8 dB) */
                *prev_ind = silk_max_int(*ind.offset(k as isize) as i32, *prev_ind as i32 - 16) as i8;
            } else {
                /* Delta index */
                let ind_tmp = *ind.offset(k as isize) as i32 + MIN_DELTA_GAIN_QUANT;

                /* Accumulate deltas */
                let double_step_size_threshold = 2 * MAX_DELTA_GAIN_QUANT - N_LEVELS_QGAIN + *prev_ind as i32;
                if ind_tmp > double_step_size_threshold {
                    *prev_ind = (*prev_ind as i32 + silk_lshift(ind_tmp, 1) - double_step_size_threshold) as i8;
                } else {
                    *prev_ind = (*prev_ind as i32 + ind_tmp) as i8;
                }
            }
            *prev_ind = silk_limit_int(*prev_ind as i32, 0, N_LEVELS_QGAIN - 1) as i8;

            /* Scale and convert to linear scale */
            *gain_q16.offset(k as isize) =
                silk_log2lin(silk_min_32(silk_smulwb(INV_SCALE_Q16, *prev_ind as i32) + OFFSET, 3967)); /* 3967 = 31 in Q7 */
            k += 1;
        }
    }
}

/// `silk_gains_ID` — compute a unique identifier for a gain index vector.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_gains_ID(ind: *const i8, nb_subfr: i32) -> i32 {
    unsafe {
        let mut gains_id: i32 = 0;
        let mut k = 0;
        while k < nb_subfr {
            gains_id = silk_add_lshift32(*ind.offset(k as isize) as i32, gains_id, 8);
            k += 1;
        }
        gains_id
    }
}
