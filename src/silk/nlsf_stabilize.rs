//! Translated from `c/silk/NLSF_stabilize.c` (RFC 6716).
//!
//! Stabilizer for Normalized Line Spectral Frequencies: enforces minimum
//! spacing between coefficients and pushes them away from boundaries.

use super::macros::{silk_limit_int, silk_rshift_round};
use super::sort::silk_insertion_sort_increasing_all_values_int16;

const MAX_LOOPS: i32 = 20;

/// `silk_NLSF_stabilize` — stabilize an NLSF vector by enforcing minimum
/// spacing. High effort to minimise Euclidean distance from the input.
pub unsafe fn silk_NLSF_stabilize(nlsf_q15: *mut i16, n_delta_min_q15: *const i16, l: i32) {
    unsafe {
        let mut i_: i32 = 0;
        let mut loops = 0;
        while loops < MAX_LOOPS {
            /**************************/
            /* Find smallest distance */
            /**************************/
            /* First element */
            let mut min_diff_q15 = *nlsf_q15.offset(0) as i32 - *n_delta_min_q15.offset(0) as i32;
            i_ = 0;
            /* Middle elements */
            let mut i = 1;
            while i <= l - 1 {
                let diff_q15 = *nlsf_q15.offset(i as isize) as i32
                    - (*nlsf_q15.offset((i - 1) as isize) as i32 + *n_delta_min_q15.offset(i as isize) as i32);
                if diff_q15 < min_diff_q15 {
                    min_diff_q15 = diff_q15;
                    i_ = i;
                }
                i += 1;
            }
            /* Last element */
            let diff_q15 = (1 << 15) - (*nlsf_q15.offset((l - 1) as isize) as i32 + *n_delta_min_q15.offset(l as isize) as i32);
            if diff_q15 < min_diff_q15 {
                min_diff_q15 = diff_q15;
                i_ = l;
            }

            /***************************************************/
            /* Now check if the smallest distance non-negative */
            /***************************************************/
            if min_diff_q15 >= 0 {
                return;
            }

            if i_ == 0 {
                /* Move away from lower limit */
                *nlsf_q15.offset(0) = *n_delta_min_q15.offset(0);
            } else if i_ == l {
                /* Move away from higher limit */
                *nlsf_q15.offset((l - 1) as isize) = ((1 << 15) - *n_delta_min_q15.offset(l as isize) as i32) as i16;
            } else {
                /* Find the lower extreme for the location of the current center frequency */
                let mut min_center_q15: i32 = 0;
                let mut k = 0;
                while k < i_ {
                    min_center_q15 += *n_delta_min_q15.offset(k as isize) as i32;
                    k += 1;
                }
                min_center_q15 += *n_delta_min_q15.offset(i_ as isize) as i32 >> 1;

                /* Find the upper extreme for the location of the current center frequency */
                let mut max_center_q15: i32 = 1 << 15;
                let mut k = l;
                while k > i_ {
                    max_center_q15 -= *n_delta_min_q15.offset(k as isize) as i32;
                    k -= 1;
                }
                max_center_q15 -= *n_delta_min_q15.offset(i_ as isize) as i32 >> 1;

                /* Move apart, sorted by value, keeping the same center frequency */
                let center_freq_q15 = silk_limit_int(
                    silk_rshift_round(*nlsf_q15.offset((i_ - 1) as isize) as i32 + *nlsf_q15.offset(i_ as isize) as i32, 1),
                    min_center_q15,
                    max_center_q15,
                ) as i16;
                *nlsf_q15.offset((i_ - 1) as isize) =
                    center_freq_q15 - (*n_delta_min_q15.offset(i_ as isize) as i32 >> 1) as i16;
                *nlsf_q15.offset(i_ as isize) = *nlsf_q15.offset((i_ - 1) as isize) + *n_delta_min_q15.offset(i_ as isize);
            }
            loops += 1;
        }

        /* Safe and simple fall back method, which is less ideal than the above */
        if loops == MAX_LOOPS {
            /* Insertion sort (fast for already almost sorted arrays) */
            silk_insertion_sort_increasing_all_values_int16(nlsf_q15, l);

            /* First NLSF should be no less than NDeltaMin[0] */
            *nlsf_q15.offset(0) = (*nlsf_q15.offset(0) as i32).max(*n_delta_min_q15.offset(0) as i32) as i16;

            /* Keep delta_min distance between the NLSFs */
            let mut i = 1;
            while i < l {
                *nlsf_q15.offset(i as isize) = (*nlsf_q15.offset(i as isize) as i32)
                    .max(*nlsf_q15.offset((i - 1) as isize) as i32 + *n_delta_min_q15.offset(i as isize) as i32)
                    as i16;
                i += 1;
            }

            /* Last NLSF should be no higher than 1 - NDeltaMin[L] */
            *nlsf_q15.offset((l - 1) as isize) =
                (*nlsf_q15.offset((l - 1) as isize) as i32).min((1 << 15) - *n_delta_min_q15.offset(l as isize) as i32) as i16;

            /* Keep NDeltaMin distance between the NLSFs */
            let mut i = l - 2;
            while i >= 0 {
                *nlsf_q15.offset(i as isize) = (*nlsf_q15.offset(i as isize) as i32)
                    .min(*nlsf_q15.offset((i + 1) as isize) as i32 - *n_delta_min_q15.offset((i + 1) as isize) as i32)
                    as i16;
                i -= 1;
            }
        }
    }
}
