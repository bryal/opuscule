//! Translated from `c/silk/interpolate.c` (RFC 6716).
//!
//! Linear interpolation of two `i16` coefficient vectors.

use super::macros::{silk_add_rshift, silk_smulbb};

/// `silk_interpolate` — interpolate two `i16` vectors with a Q2 blend factor.
///
/// `xi[i] = x0[i] + ((x1[i] - x0[i]) * ifact_Q2) >> 2`, for `i` in `0..d`.
/// `ifact_Q2` is expected to be in `[0, 4]` (0 → all `x0`, 4 → all `x1`).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_interpolate(xi: *mut i16, x0: *const i16, x1: *const i16, ifact_q2: i32, d: i32) {
    unsafe {
        let mut i = 0;
        while i < d {
            *xi.offset(i as isize) = silk_add_rshift(
                *x0.offset(i as isize) as i32,
                silk_smulbb(*x1.offset(i as isize) as i32 - *x0.offset(i as isize) as i32, ifact_q2),
                2,
            ) as i16;
            i += 1;
        }
    }
}
