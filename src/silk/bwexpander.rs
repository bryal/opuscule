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
pub fn silk_bwexpander(ar: &mut [i16], mut chirp_q16: i32) {
    let chirp_minus_one_q16 = chirp_q16 - 65536;
    let d = ar.len();

    /* NB: Dont use silk_SMULWB, instead of silk_RSHIFT_ROUND( silk_MUL(), 16 ), below.  */
    /* Bias in silk_SMULWB can lead to unstable filters                                */
    for i in 0..d - 1 {
        ar[i] = silk_rshift_round(chirp_q16 * ar[i] as i32, 16) as i16;
        chirp_q16 += silk_rshift_round(chirp_q16 * chirp_minus_one_q16, 16);
    }
    ar[d - 1] = silk_rshift_round(chirp_q16 * ar[d - 1] as i32, 16) as i16;
}
