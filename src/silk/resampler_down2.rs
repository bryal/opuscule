//! Translated from `c/silk/resampler_down2.c` (RFC 6716).
//!
//! Two-section all-pass-based 2× downsampler. Each output sample is the
//! sum of the responses of an even-tap and an odd-tap all-pass section
//! whose coefficients live in [`silk_resampler_down2_0`] and
//! [`silk_resampler_down2_1`].

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb};
use super::resampler_rom::{SILK_RESAMPLER_DOWN2_0, SILK_RESAMPLER_DOWN2_1};

/// `silk_resampler_down2` — downsample by a factor 2.
pub fn silk_resampler_down2(s: &mut [i32], out: &mut [i16], in_: &[i16], in_len: i32) {
    let len2 = (in_len >> 1) as usize;

    /* silk_assert(silk_resampler_down2_0 > 0); */
    /* silk_assert(silk_resampler_down2_1 < 0); */

    /* Internal variables and state are in Q10 format */
    let mut k = 0;
    while k < len2 {
        /* Convert to Q10 */
        let in32 = silk_lshift(in_[2 * k] as i32, 10);

        /* All-pass section for even input sample */
        let y = in32 - s[0];
        let x = silk_smlawb(y, y, SILK_RESAMPLER_DOWN2_1 as i32);
        let mut out32 = s[0] + x;
        s[0] = in32 + x;

        /* Convert to Q10 */
        let in32 = silk_lshift(in_[2 * k + 1] as i32, 10);

        /* All-pass section for odd input sample, and add to output of previous section */
        let y = in32 - s[1];
        let x = silk_smulwb(y, SILK_RESAMPLER_DOWN2_0 as i32);
        out32 += s[1];
        out32 += x;
        s[1] = in32 + x;

        /* Add, convert back to int16 and store to output */
        out[k] = silk_sat16(silk_rshift_round(out32, 11)) as i16;
        k += 1;
    }
}
