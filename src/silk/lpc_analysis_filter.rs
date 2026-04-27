//! Translated from `c/silk/LPC_analysis_filter.c` (RFC 6716).
//!
//! FIR prediction-error filter: computes `out = in - predicted(in)`.
//! The first `d` output samples are set to zero (filter warm-up).

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlabb_ovflw, silk_smulbb};

/// `silk_LPC_analysis_filter` — apply an LPC analysis filter of order `d`
/// (must be even and ≥ 6) to `in_[0..len]`, writing the residual to
/// `out[0..len]`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_LPC_analysis_filter(out: *mut i16, in_: *const i16, b: *const i16, len: i32, d: i32) {
    unsafe {
        let mut ix = d;
        while ix < len {
            let in_ptr = in_.offset((ix - 1) as isize);

            let mut out32_q12 = silk_smulbb(*in_ptr.offset(0) as i32, *b.offset(0) as i32);
            /* Allowing wrap around so that two wraps can cancel each other. The rare
            cases where the result wraps around can only be triggered by invalid streams*/
            out32_q12 = silk_smlabb_ovflw(out32_q12, *in_ptr.offset(-1) as i32, *b.offset(1) as i32);
            out32_q12 = silk_smlabb_ovflw(out32_q12, *in_ptr.offset(-2) as i32, *b.offset(2) as i32);
            out32_q12 = silk_smlabb_ovflw(out32_q12, *in_ptr.offset(-3) as i32, *b.offset(3) as i32);
            out32_q12 = silk_smlabb_ovflw(out32_q12, *in_ptr.offset(-4) as i32, *b.offset(4) as i32);
            out32_q12 = silk_smlabb_ovflw(out32_q12, *in_ptr.offset(-5) as i32, *b.offset(5) as i32);
            let mut j = 6;
            while j < d {
                out32_q12 = silk_smlabb_ovflw(out32_q12, *in_ptr.offset(-j as isize) as i32, *b.offset(j as isize) as i32);
                out32_q12 =
                    silk_smlabb_ovflw(out32_q12, *in_ptr.offset((-j - 1) as isize) as i32, *b.offset((j + 1) as isize) as i32);
                j += 2;
            }

            /* Subtract prediction */
            out32_q12 = silk_lshift(*in_ptr.offset(1) as i32, 12).wrapping_sub(out32_q12);

            /* Scale to Q0 */
            let out32 = silk_rshift_round(out32_q12, 12);

            /* Saturate output */
            *out.offset(ix as isize) = silk_sat16(out32) as i16;
            ix += 1;
        }

        /* Set first d output samples to zero */
        std::ptr::write_bytes(out, 0, d as usize);
    }
}
