//! Translated from `c/silk/resampler_private_up2_HQ.c` (RFC 6716).
//!
//! High-quality 2× upsampler. Three cascaded all-pass sections per
//! polyphase branch, followed (externally) by a notch filter just above
//! the original Nyquist.

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb};
use super::resampler_rom::{SILK_RESAMPLER_UP2_HQ_0, SILK_RESAMPLER_UP2_HQ_1};
use super::structs::SilkResamplerStateStruct;

/// `silk_resampler_private_up2_HQ` — high-quality 2× upsampler.
///
/// `s` is a 6-element resampler state vector (Q10).
pub fn silk_resampler_private_up2_hq(s: &mut [i32], out: &mut [i16], in_: &[i16], len: i32) {
    /* silk_assert(silk_resampler_up2_hq_0[0] > 0); */
    /* silk_assert(silk_resampler_up2_hq_0[1] > 0); */
    /* silk_assert(silk_resampler_up2_hq_0[2] < 0); */
    /* silk_assert(silk_resampler_up2_hq_1[0] > 0); */
    /* silk_assert(silk_resampler_up2_hq_1[1] > 0); */
    /* silk_assert(silk_resampler_up2_hq_1[2] < 0); */

    /* Internal variables and state are in Q10 format */
    let mut k = 0;
    while k < len as usize {
        /* Convert to Q10 */
        let in32 = silk_lshift(in_[k] as i32, 10);

        /* First all-pass section for even output sample */
        let y = in32 - s[0];
        let x = silk_smulwb(y, SILK_RESAMPLER_UP2_HQ_0[0] as i32);
        let mut out32_1 = s[0] + x;
        s[0] = in32 + x;

        /* Second all-pass section for even output sample */
        let y = out32_1 - s[1];
        let x = silk_smulwb(y, SILK_RESAMPLER_UP2_HQ_0[1] as i32);
        let out32_2 = s[1] + x;
        s[1] = out32_1 + x;

        /* Third all-pass section for even output sample */
        let y = out32_2 - s[2];
        let x = silk_smlawb(y, y, SILK_RESAMPLER_UP2_HQ_0[2] as i32);
        out32_1 = s[2] + x;
        s[2] = out32_2 + x;

        /* Apply gain in Q15, convert back to int16 and store to output */
        out[2 * k] = silk_sat16(silk_rshift_round(out32_1, 10)) as i16;

        /* First all-pass section for odd output sample */
        let y = in32 - s[3];
        let x = silk_smulwb(y, SILK_RESAMPLER_UP2_HQ_1[0] as i32);
        let mut out32_1 = s[3] + x;
        s[3] = in32 + x;

        /* Second all-pass section for odd output sample */
        let y = out32_1 - s[4];
        let x = silk_smulwb(y, SILK_RESAMPLER_UP2_HQ_1[1] as i32);
        let out32_2 = s[4] + x;
        s[4] = out32_1 + x;

        /* Third all-pass section for odd output sample */
        let y = out32_2 - s[5];
        let x = silk_smlawb(y, y, SILK_RESAMPLER_UP2_HQ_1[2] as i32);
        out32_1 = s[5] + x;
        s[5] = out32_2 + x;

        /* Apply gain in Q15, convert back to int16 and store to output */
        out[2 * k + 1] = silk_sat16(silk_rshift_round(out32_1, 10)) as i16;
        k += 1;
    }
}

/// `silk_resampler_private_up2_HQ_wrapper` — adapts the resampler dispatch
/// signature to the typed entry above.
pub fn silk_resampler_private_up2_hq_wrapper(s: &mut SilkResamplerStateStruct, out: &mut [i16], in_: &[i16], len: i32) {
    silk_resampler_private_up2_hq(&mut s.s_iir, out, in_, len);
}
