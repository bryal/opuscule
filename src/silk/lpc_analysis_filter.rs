//! Translated from `c/silk/LPC_analysis_filter.c` (RFC 6716).
//!
//! FIR prediction-error filter: computes `out = in - predicted(in)`.
//! The first `d` output samples are set to zero (filter warm-up).

#![allow(clippy::indexing_slicing)] // dense SILK kernels; voice path is deprioritized vs CELT

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlabb_ovflw, silk_smulbb};

/// `silk_LPC_analysis_filter` — apply an LPC analysis filter of order `d`
/// (must be even and ≥ 6) to `in_[0..len]`, writing the residual to
/// `out[0..len]`. `b` is the LPC coefficient vector (Q12) of length `d`.
pub fn silk_lpc_analysis_filter(out: &mut [i16], in_: &[i16], b: &[i16], len: i32, d: i32) {
    for ix in d..len {
        let centre = (ix - 1) as usize;

        let mut out32_q12 = silk_smulbb(in_[centre] as i32, b[0] as i32);
        /* Allowing wrap around so that two wraps can cancel each other. The rare
        cases where the result wraps around can only be triggered by invalid streams*/
        out32_q12 = silk_smlabb_ovflw(out32_q12, in_[centre - 1] as i32, b[1] as i32);
        out32_q12 = silk_smlabb_ovflw(out32_q12, in_[centre - 2] as i32, b[2] as i32);
        out32_q12 = silk_smlabb_ovflw(out32_q12, in_[centre - 3] as i32, b[3] as i32);
        out32_q12 = silk_smlabb_ovflw(out32_q12, in_[centre - 4] as i32, b[4] as i32);
        out32_q12 = silk_smlabb_ovflw(out32_q12, in_[centre - 5] as i32, b[5] as i32);
        let mut j = 6;
        while j < d {
            out32_q12 = silk_smlabb_ovflw(out32_q12, in_[centre - j as usize] as i32, b[j as usize] as i32);
            out32_q12 = silk_smlabb_ovflw(out32_q12, in_[centre - j as usize - 1] as i32, b[j as usize + 1] as i32);
            j += 2;
        }

        /* Subtract prediction */
        out32_q12 = silk_lshift(in_[centre + 1] as i32, 12).wrapping_sub(out32_q12);

        /* Scale to Q0 */
        let out32 = silk_rshift_round(out32_q12, 12);

        /* Saturate output */
        out[ix as usize] = silk_sat16(out32) as i16;
    }

    /* Set first d output samples to zero */
    out[..d as usize].fill(0);
}
