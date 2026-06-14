//! Translated from `c/silk/resampler_private_AR2.c` (RFC 6716).
//!
//! Second-order AR filter with single delay elements, used internally by
//! the resampler.

#![allow(clippy::indexing_slicing)] // dense SILK kernels; voice path is deprioritized vs CELT

use super::macros::{silk_lshift, silk_smlawb, silk_smulwb};

/// `silk_resampler_private_AR2` — second-order AR section.
///
/// `s` is a 2-element state vector (Q?), `out_q8` receives `len` Q8
/// outputs, `in_` is the i16 input, `a_q14` is a 2-element AR coefficient
/// vector in Q14.
pub fn silk_resampler_private_ar2(s: &mut [i32], out_q8: &mut [i32], in_: &[i16], a_q14: &[i16], len: i32) {
    let mut k = 0;
    while k < len as usize {
        let mut out32 = s[0] + silk_lshift(in_[k] as i32, 8);
        out_q8[k] = out32;
        out32 = silk_lshift(out32, 2);
        s[0] = silk_smlawb(s[1], out32, a_q14[0] as i32);
        s[1] = silk_smulwb(out32, a_q14[1] as i32);
        k += 1;
    }
}
