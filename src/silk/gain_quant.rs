//! Translated from `c/silk/gain_quant.c` (RFC 6716).
//!
//! Gain scalar quantization / dequantization on a log scale with
//! hysteresis and delta coding.

use super::log2lin::silk_log2lin;
use super::macros::{silk_limit_int, silk_lshift, silk_smulwb};

// -- Constants from c/silk/define.h --

const MIN_QGAIN_DB: i32 = 2;
const MAX_QGAIN_DB: i32 = 88;
const N_LEVELS_QGAIN: i32 = 64;
const MAX_DELTA_GAIN_QUANT: i32 = 36;
const MIN_DELTA_GAIN_QUANT: i32 = -4;

// -- Derived constants matching the C #defines --

const OFFSET: i32 = (MIN_QGAIN_DB * 128) / 6 + 16 * 128;
const INV_SCALE_Q16: i32 = (65536 * (((MAX_QGAIN_DB - MIN_QGAIN_DB) * 128) / 6)) / (N_LEVELS_QGAIN - 1);

/// `silk_gains_dequant` — dequantize gain indices back to linear Q16 gains.
pub fn silk_gains_dequant(gain_q16: &mut [i32], ind: &[i8], prev_ind: &mut i8, conditional: i32, nb_subfr: i32) {
    for k in 0..nb_subfr as usize {
        if k == 0 && conditional == 0 {
            /* Gain index is not allowed to go down more than 16 steps (~21.8 dB) */
            *prev_ind = (ind[k] as i32).max(*prev_ind as i32 - 16) as i8;
        } else {
            /* Delta index */
            let ind_tmp = ind[k] as i32 + MIN_DELTA_GAIN_QUANT;

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
        gain_q16[k] = silk_log2lin((silk_smulwb(INV_SCALE_Q16, *prev_ind as i32) + OFFSET).min(3967)); /* 3967 = 31 in Q7 */
    }
}
