//! Translated from `c/silk/lin2log.c` (RFC 6716, section 4.2 / silk).
//!
//! Approximation of `128 * log2(inLin)` — the inverse of [`silk_log2lin`].

use super::macros::{silk_clz_frac, silk_lshift, silk_mul, silk_smlawb};

/// `silk_lin2log` — convert a linear-scale value to a `log2`-scale value
/// in Q7. Returns `128 * log2(inLin)` rounded to the nearest integer (well,
/// to the resolution of a piece-wise parabolic approximation).
///
/// Marked `extern "C"` and `no_mangle` so that any remaining C SILK code
/// links against this Rust implementation rather than the deleted
/// `silk/lin2log.c`.
#[unsafe(no_mangle)]
pub extern "C" fn silk_lin2log(in_lin: i32) -> i32 {
    let mut lz: i32 = 0;
    let mut frac_q7: i32 = 0;

    silk_clz_frac(in_lin, &mut lz, &mut frac_q7);

    /* Piece-wise parabolic approximation */
    silk_lshift(31 - lz, 7) + silk_smlawb(frac_q7, silk_mul(frac_q7, 128 - frac_q7), 179)
}
