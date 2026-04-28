//! Translated from `c/silk/resampler_private_AR2.c` (RFC 6716).
//!
//! Second-order AR filter with single delay elements, used internally by
//! the resampler.

use super::macros::{silk_lshift, silk_smlawb, silk_smulwb};

/// `silk_resampler_private_AR2` — second-order AR section.
///
/// `S` is a 2-element state vector (Q?), `out_Q8` receives `len` Q8
/// outputs, `in` is the i16 input, `A_Q14` is a 2-element AR coefficient
/// vector in Q14.
pub unsafe fn silk_resampler_private_AR2(
    s: *mut i32,
    out_q8: *mut i32,
    in_: *const i16,
    a_q14: *const i16,
    len: i32,
) {
    unsafe {
        let mut k = 0;
        while k < len {
            let mut out32 = *s.offset(0) + silk_lshift(*in_.offset(k as isize) as i32, 8);
            *out_q8.offset(k as isize) = out32;
            out32 = silk_lshift(out32, 2);
            *s.offset(0) = silk_smlawb(*s.offset(1), out32, *a_q14.offset(0) as i32);
            *s.offset(1) = silk_smulwb(out32, *a_q14.offset(1) as i32);
            k += 1;
        }
    }
}
