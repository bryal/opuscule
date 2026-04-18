//! Translated from `silk_SQRT_APPROX` in `c/silk/Inlines.h` (RFC 6716).
//!
//! Cheap square-root approximation used by the NLSF decoder and PLC.
//! Accuracy: < ±10% for outputs > 15, < ±2.5% for outputs > 120.

use super::macros::{silk_clz_frac, silk_rshift, silk_smlawb, silk_smulbb};

/// `silk_SQRT_APPROX` — approximate `sqrt(x)`.
#[inline]
pub fn silk_sqrt_approx(x: i32) -> i32 {
    if x <= 0 {
        return 0;
    }

    let mut lz: i32 = 0;
    let mut frac_q7: i32 = 0;
    silk_clz_frac(x, &mut lz, &mut frac_q7);

    let mut y = if lz & 1 != 0 {
        32768
    } else {
        46214 /* 46214 = sqrt(2) * 32768 */
    };

    /* get scaling right */
    y >>= silk_rshift(lz, 1);

    /* increment using fractional part of input */
    y = silk_smlawb(y, y, silk_smulbb(213, frac_q7));

    y
}
