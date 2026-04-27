//! Translated from `c/silk/inner_prod_aligned.c` (RFC 6716).
//!
//! Scaled inner product of two `i16` vectors.

use super::macros::silk_smulbb;

/// `silk_inner_prod_aligned_scale` — compute the inner product of two
/// `i16` vectors, right-shifting each partial product by `scale` bits
/// before accumulation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_inner_prod_aligned_scale(in_vec1: *const i16, in_vec2: *const i16, scale: i32, len: i32) -> i32 {
    unsafe {
        let mut sum: i32 = 0;
        let mut i = 0;
        while i < len {
            sum += silk_smulbb(*in_vec1.offset(i as isize) as i32, *in_vec2.offset(i as isize) as i32) >> scale;
            i += 1;
        }
        sum
    }
}
