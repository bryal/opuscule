//! Translated from `c/silk/sum_sqr_shift.c` (RFC 6716).
//!
//! Computes the sum-of-squares energy of an `i16` vector, auto-scaling
//! the accumulator by powers of two to prevent overflow.

use super::macros::{silk_smlabb_ovflw, silk_smulbb};

/// `silk_sum_sqr_shift` — compute the energy (sum of squares) of `x[0..len]`,
/// returning both the energy value and the number of right-shift bits applied
/// to keep the result in 32 bits with at least two leading zeros.
pub unsafe fn silk_sum_sqr_shift(energy: *mut i32, shift: *mut i32, x: *const i16, len: i32) {
    unsafe {
        let mut nrg: i32 = 0;
        let mut shft: i32 = 0;
        let len = len - 1;

        let mut i = 0;
        while i < len {
            nrg = silk_smlabb_ovflw(nrg, *x.offset(i as isize) as i32, *x.offset(i as isize) as i32);
            nrg = silk_smlabb_ovflw(nrg, *x.offset((i + 1) as isize) as i32, *x.offset((i + 1) as isize) as i32);
            if nrg < 0 {
                /* Scale down */
                nrg = (nrg as u32 >> 2) as i32;
                shft = 2;
                i += 2;
                break;
            }
            i += 2;
        }
        while i < len {
            let mut nrg_tmp = silk_smulbb(*x.offset(i as isize) as i32, *x.offset(i as isize) as i32);
            nrg_tmp = silk_smlabb_ovflw(nrg_tmp, *x.offset((i + 1) as isize) as i32, *x.offset((i + 1) as isize) as i32);
            nrg = (nrg as u32 + (nrg_tmp as u32 >> shft)) as i32;
            if nrg < 0 {
                /* Scale down */
                nrg = (nrg as u32 >> 2) as i32;
                shft += 2;
            }
            i += 2;
        }
        if i == len {
            /* One sample left to process */
            let nrg_tmp = silk_smulbb(*x.offset(i as isize) as i32, *x.offset(i as isize) as i32);
            nrg = (nrg as u32 + (nrg_tmp as u32 >> shft)) as i32;
        }

        /* Make sure to have at least one extra leading zero (two leading zeros in total) */
        if nrg & (0xC0000000u32 as i32) != 0 {
            nrg = (nrg as u32 >> 2) as i32;
            shft += 2;
        }

        /* Output arguments */
        *shift = shft;
        *energy = nrg;
    }
}
