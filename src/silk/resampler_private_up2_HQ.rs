//! Translated from `c/silk/resampler_private_up2_HQ.c` (RFC 6716).
//!
//! High-quality 2× upsampler. Three cascaded all-pass sections per
//! polyphase branch, followed (externally) by a notch filter just above
//! the original Nyquist.

use core::ffi::c_void;

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb};
use super::resampler_rom::{silk_resampler_up2_hq_0, silk_resampler_up2_hq_1};

/// `silk_resampler_private_up2_HQ` — high-quality 2× upsampler.
///
/// `S` is a 6-element resampler state vector (Q10).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_resampler_private_up2_HQ(s: *mut i32, out: *mut i16, in_: *const i16, len: i32) {
    unsafe {
        /* silk_assert(silk_resampler_up2_hq_0[0] > 0); */
        /* silk_assert(silk_resampler_up2_hq_0[1] > 0); */
        /* silk_assert(silk_resampler_up2_hq_0[2] < 0); */
        /* silk_assert(silk_resampler_up2_hq_1[0] > 0); */
        /* silk_assert(silk_resampler_up2_hq_1[1] > 0); */
        /* silk_assert(silk_resampler_up2_hq_1[2] < 0); */

        /* Internal variables and state are in Q10 format */
        let mut k = 0;
        while k < len {
            /* Convert to Q10 */
            let in32 = silk_lshift(*in_.offset(k as isize) as i32, 10);

            /* First all-pass section for even output sample */
            let y = in32 - *s.offset(0);
            let x = silk_smulwb(y, silk_resampler_up2_hq_0[0] as i32);
            let mut out32_1 = *s.offset(0) + x;
            *s.offset(0) = in32 + x;

            /* Second all-pass section for even output sample */
            let y = out32_1 - *s.offset(1);
            let x = silk_smulwb(y, silk_resampler_up2_hq_0[1] as i32);
            let out32_2 = *s.offset(1) + x;
            *s.offset(1) = out32_1 + x;

            /* Third all-pass section for even output sample */
            let y = out32_2 - *s.offset(2);
            let x = silk_smlawb(y, y, silk_resampler_up2_hq_0[2] as i32);
            out32_1 = *s.offset(2) + x;
            *s.offset(2) = out32_2 + x;

            /* Apply gain in Q15, convert back to int16 and store to output */
            *out.offset((2 * k) as isize) = silk_sat16(silk_rshift_round(out32_1, 10)) as i16;

            /* First all-pass section for odd output sample */
            let y = in32 - *s.offset(3);
            let x = silk_smulwb(y, silk_resampler_up2_hq_1[0] as i32);
            let mut out32_1 = *s.offset(3) + x;
            *s.offset(3) = in32 + x;

            /* Second all-pass section for odd output sample */
            let y = out32_1 - *s.offset(4);
            let x = silk_smulwb(y, silk_resampler_up2_hq_1[1] as i32);
            let out32_2 = *s.offset(4) + x;
            *s.offset(4) = out32_1 + x;

            /* Third all-pass section for odd output sample */
            let y = out32_2 - *s.offset(5);
            let x = silk_smlawb(y, y, silk_resampler_up2_hq_1[2] as i32);
            out32_1 = *s.offset(5) + x;
            *s.offset(5) = out32_2 + x;

            /* Apply gain in Q15, convert back to int16 and store to output */
            *out.offset((2 * k + 1) as isize) = silk_sat16(silk_rshift_round(out32_1, 10)) as i16;
            k += 1;
        }
    }
}

/// `silk_resampler_private_up2_HQ_wrapper` — adapts the resampler dispatch
/// signature to the typed entry above.
///
/// The C cast is `(silk_resampler_state_struct *)SS; ...; S->sIIR`. Because
/// `sIIR[6]` is the first member of the C struct (and the header comment
/// flags this as load-bearing), the cast plus member access is byte-for-byte
/// equivalent to treating `SS` as `*mut i32`. We rely on the same invariant
/// here until the resampler state struct is itself translated.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_resampler_private_up2_HQ_wrapper(ss: *mut c_void, out: *mut i16, in_: *const i16, len: i32) {
    unsafe {
        silk_resampler_private_up2_HQ(ss as *mut i32, out, in_, len);
    }
}
