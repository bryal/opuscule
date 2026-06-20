//! Translated from `c/silk/log2lin.c` (RFC 6716, section 4.2 / silk).
//!
//! Approximation of `2^(inLog_Q7 / 128)` - the inverse of the (encoder-only,
//! not translated) `silk_lin2log`.

use super::macros::{silk_lshift, silk_smlawb, silk_smulbb};

/// `silk_log2lin` — convert a Q7 log-scale value back to a linear value.
/// Returns 0 for negative input. The C does not bound the upper end:
/// `silk_LSHIFT(1, inLog_Q7 >> 7)` is allowed to overflow, and the macro
/// is therefore expected to wrap (we use [`silk_lshift`]'s wrapping shift
/// to mirror that behaviour).
pub fn silk_log2lin(in_log_q7: i32) -> i32 {
    if in_log_q7 < 0 {
        return 0;
    }

    let mut out = silk_lshift(1, in_log_q7 >> 7);
    let frac_q7 = in_log_q7 & 0x7F;
    if in_log_q7 < 2048 {
        /* Piece-wise parabolic approximation */
        out += out * silk_smlawb(frac_q7, silk_smulbb(frac_q7, 128 - frac_q7), -174) >> 7;
    } else {
        /* Piece-wise parabolic approximation */
        out += (out >> 7) * silk_smlawb(frac_q7, silk_smulbb(frac_q7, 128 - frac_q7), -174);
    }
    out
}
