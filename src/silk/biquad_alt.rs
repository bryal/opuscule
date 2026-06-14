//! Translated from `c/silk/biquad_alt.c` (RFC 6716).
//!
//! Second-order ARMA filter (direct form II transposed).

#![allow(clippy::indexing_slicing)] // dense SILK kernels; voice path is deprioritized vs CELT

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb};

/// `silk_biquad_alt` — second-order ARMA filter, alternative implementation.
///
/// Operates on interleaved data when `stride > 1`. The MA coefficients
/// `b_q28[3]` and AR coefficients `a_q28[2]` are Q28 fixed-point. The
/// state vector `s[2]` is Q12.
pub fn silk_biquad_alt(in_: &[i16], b_q28: &[i32], a_q28: &[i32], s: &mut [i32], out: &mut [i16], len: i32, stride: i32) {
    /* DIRECT FORM II TRANSPOSED (uses 2 element state vector) */

    /* Negate A_Q28 values and split in two parts */
    let a0_l_q28 = (-a_q28[0]) & 0x00003FFF; /* lower part */
    let a0_u_q28 = -a_q28[0] >> 14; /* upper part */
    let a1_l_q28 = (-a_q28[1]) & 0x00003FFF; /* lower part */
    let a1_u_q28 = -a_q28[1] >> 14; /* upper part */

    let mut k = 0;
    while k < len {
        /* S[ 0 ], S[ 1 ]: Q12 */
        let inval = in_[(k * stride) as usize] as i32;
        let out32_q14 = silk_lshift(silk_smlawb(s[0], b_q28[0], inval), 2);

        s[0] = s[1] + silk_rshift_round(silk_smulwb(out32_q14, a0_l_q28), 14);
        s[0] = silk_smlawb(s[0], out32_q14, a0_u_q28);
        s[0] = silk_smlawb(s[0], b_q28[1], inval);

        s[1] = silk_rshift_round(silk_smulwb(out32_q14, a1_l_q28), 14);
        s[1] = silk_smlawb(s[1], out32_q14, a1_u_q28);
        s[1] = silk_smlawb(s[1], b_q28[2], inval);

        /* Scale back to Q0 and saturate */
        out[(k * stride) as usize] = silk_sat16((out32_q14 + (1 << 14) - 1) >> 14) as i16;
        k += 1;
    }
}
