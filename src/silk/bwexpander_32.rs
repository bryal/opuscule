//! Translated from `c/silk/bwexpander_32.c` (RFC 6716).
//!
//! Chirp (bandwidth expansion) of an LP AR filter stored as `i32`
//! coefficients. Same idea as [`super::bwexpander`] but operates on a
//! wider representation and uses `silk_SMULWW` instead of the
//! RSHIFT_ROUND(MUL) workaround.

use super::macros::{silk_rshift_round, silk_smulww};

/// `silk_bwexpander_32` — bandwidth-expand an AR filter (32-bit coefficients).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_bwexpander_32(ar: *mut i32, d: i32, mut chirp_q16: i32) {
    unsafe {
        let chirp_minus_one_q16 = chirp_q16 - 65536;

        let mut i = 0;
        while i < d - 1 {
            *ar.offset(i as isize) = silk_smulww(chirp_q16, *ar.offset(i as isize));
            chirp_q16 += silk_rshift_round(chirp_q16 * chirp_minus_one_q16, 16);
            i += 1;
        }
        *ar.offset((d - 1) as isize) = silk_smulww(chirp_q16, *ar.offset((d - 1) as isize));
    }
}
