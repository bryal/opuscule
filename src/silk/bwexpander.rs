//! Translated from `c/silk/bwexpander.c` (RFC 6716).
//!
//! Chirp (bandwidth expansion) of an LP AR filter stored as Q12 `i16`
//! coefficients.

use super::macros::silk_rshift_round;

/// `silk_bwexpander` — bandwidth-expand an AR filter by applying a
/// geometrically decaying chirp factor.
///
/// `ar[i]` is scaled by `chirp_Q16^(i+1)` (approximate), where the chirp
/// factor itself decays each iteration. The NB comment in the C source
/// warns against using `silk_SMULWB` here because its rounding bias can
/// make the filter unstable; `silk_RSHIFT_ROUND(silk_MUL(…), 16)` is used
/// instead.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_bwexpander(ar: *mut i16, d: i32, mut chirp_q16: i32) {
    unsafe {
        let chirp_minus_one_q16 = chirp_q16 - 65536;

        /* NB: Dont use silk_SMULWB, instead of silk_RSHIFT_ROUND( silk_MUL(), 16 ), below.  */
        /* Bias in silk_SMULWB can lead to unstable filters                                */
        let mut i = 0;
        while i < d - 1 {
            *ar.offset(i as isize) = silk_rshift_round(chirp_q16 * *ar.offset(i as isize) as i32, 16) as i16;
            chirp_q16 += silk_rshift_round(chirp_q16 * chirp_minus_one_q16, 16);
            i += 1;
        }
        *ar.offset((d - 1) as isize) = silk_rshift_round(chirp_q16 * *ar.offset((d - 1) as isize) as i32, 16) as i16;
    }
}
