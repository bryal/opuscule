//! Translated from `c/silk/bwexpander_32.c` (RFC 6716).
//!
//! Chirp (bandwidth expansion) of an LP AR filter stored as `i32`
//! coefficients. Same idea as [`super::bwexpander`] but operates on a
//! wider representation and uses `silk_SMULWW` instead of the
//! RSHIFT_ROUND(MUL) workaround.

use super::macros::{silk_rshift_round, silk_smulww};

/// `silk_bwexpander_32` — bandwidth-expand an AR filter (32-bit coefficients).
pub fn silk_bwexpander_32(ar: &mut [i32], mut chirp_q16: i32) {
    let chirp_minus_one_q16 = chirp_q16 - 65536;
    let d = ar.len();

    for i in 0..d - 1 {
        ar[i] = silk_smulww(chirp_q16, ar[i]);
        chirp_q16 += silk_rshift_round(chirp_q16 * chirp_minus_one_q16, 16);
    }
    ar[d - 1] = silk_smulww(chirp_q16, ar[d - 1]);
}
