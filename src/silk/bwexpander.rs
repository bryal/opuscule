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
    let chirp_minus_one_q16 = chirp_q16 - (1i32 << 16);

    /* NB: Dont use silk_SMULWB, instead of silk_RSHIFT_ROUND( silk_MUL(), 16 ), below.  */
    /* Bias in silk_SMULWB can lead to unstable filters                                */
    if let Some((last, init)) = ar.split_last_mut() {
        for coeff in init {
            *coeff = silk_rshift_round(chirp_q16 * *coeff as i32, 16) as i16;
            chirp_q16 += silk_rshift_round(chirp_q16 * chirp_minus_one_q16, 16);
        }
        *last = silk_rshift_round(chirp_q16 * *last as i32, 16) as i16;
    }
}
