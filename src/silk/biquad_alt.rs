//! Translated from `c/silk/biquad_alt.c` (RFC 6716).
//!
//! Second-order ARMA filter (direct form II transposed).

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb};

/// `silk_biquad_alt` — second-order ARMA filter, alternative implementation.
///
/// Operates on interleaved data when `stride > 1`. The MA coefficients
/// `B_Q28[3]` and AR coefficients `A_Q28[2]` are Q28 fixed-point. The
/// state vector `S[2]` is Q12.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_biquad_alt(
    in_: *const i16,
    b_q28: *const i32,
    a_q28: *const i32,
    s: *mut i32,
    out: *mut i16,
    len: i32,
    stride: i32,
) {
    unsafe {
        /* DIRECT FORM II TRANSPOSED (uses 2 element state vector) */

        /* Negate A_Q28 values and split in two parts */
        let a0_l_q28 = (-*a_q28.offset(0)) & 0x00003FFF; /* lower part */
        let a0_u_q28 = -*a_q28.offset(0) >> 14; /* upper part */
        let a1_l_q28 = (-*a_q28.offset(1)) & 0x00003FFF; /* lower part */
        let a1_u_q28 = -*a_q28.offset(1) >> 14; /* upper part */

        let mut k = 0;
        while k < len {
            /* S[ 0 ], S[ 1 ]: Q12 */
            let inval = *in_.offset((k * stride) as isize) as i32;
            let out32_q14 = silk_lshift(silk_smlawb(*s.offset(0), *b_q28.offset(0), inval), 2);

            *s.offset(0) = *s.offset(1) + silk_rshift_round(silk_smulwb(out32_q14, a0_l_q28), 14);
            *s.offset(0) = silk_smlawb(*s.offset(0), out32_q14, a0_u_q28);
            *s.offset(0) = silk_smlawb(*s.offset(0), *b_q28.offset(1), inval);

            *s.offset(1) = silk_rshift_round(silk_smulwb(out32_q14, a1_l_q28), 14);
            *s.offset(1) = silk_smlawb(*s.offset(1), out32_q14, a1_u_q28);
            *s.offset(1) = silk_smlawb(*s.offset(1), *b_q28.offset(2), inval);

            /* Scale back to Q0 and saturate */
            *out.offset((k * stride) as isize) = silk_sat16((out32_q14 + (1 << 14) - 1) >> 14) as i16;
            k += 1;
        }
    }
}
