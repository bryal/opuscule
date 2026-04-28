//! Translated from `c/silk/sort.c` (RFC 6716).
//!
//! Insertion sort variants used by the SILK decoder.

/// `silk_insertion_sort_increasing` — sort the first `K` smallest elements
/// of `a[0..L]` into `a[0..K]` in increasing order, writing the original
/// indices into `idx[0..K]`.
pub unsafe fn silk_insertion_sort_increasing(a: *mut i32, idx: *mut i32, l: i32, k: i32) {
    unsafe {
        /* Write start indices in index vector */
        let mut i = 0;
        while i < k {
            *idx.offset(i as isize) = i;
            i += 1;
        }

        /* Sort vector elements by value, increasing order */
        i = 1;
        while i < k {
            let value = *a.offset(i as isize);
            let mut j = i - 1;
            while j >= 0 && value < *a.offset(j as isize) {
                *a.offset((j + 1) as isize) = *a.offset(j as isize); /* Shift value */
                *idx.offset((j + 1) as isize) = *idx.offset(j as isize); /* Shift index */
                j -= 1;
            }
            *a.offset((j + 1) as isize) = value; /* Write value */
            *idx.offset((j + 1) as isize) = i; /* Write index */
            i += 1;
        }

        /* If less than L values are asked for, check the remaining values, */
        /* but only spend CPU to ensure that the K first values are correct */
        i = k;
        while i < l {
            let value = *a.offset(i as isize);
            if value < *a.offset((k - 1) as isize) {
                let mut j = k - 2;
                while j >= 0 && value < *a.offset(j as isize) {
                    *a.offset((j + 1) as isize) = *a.offset(j as isize); /* Shift value */
                    *idx.offset((j + 1) as isize) = *idx.offset(j as isize); /* Shift index */
                    j -= 1;
                }
                *a.offset((j + 1) as isize) = value; /* Write value */
                *idx.offset((j + 1) as isize) = i; /* Write index */
            }
            i += 1;
        }
    }
}

/// `silk_insertion_sort_decreasing_int16` — sort the first `K` largest
/// elements of an `i16` vector in decreasing order.
///
/// Only used by the fixed-point build in the C source, but we compile it
/// unconditionally since the linker will discard it if unused.
pub unsafe fn silk_insertion_sort_decreasing_int16(a: *mut i16, idx: *mut i32, l: i32, k: i32) {
    unsafe {
        /* Write start indices in index vector */
        let mut i = 0;
        while i < k {
            *idx.offset(i as isize) = i;
            i += 1;
        }

        /* Sort vector elements by value, decreasing order */
        i = 1;
        while i < k {
            let value = *a.offset(i as isize) as i32;
            let mut j = i - 1;
            while j >= 0 && value > *a.offset(j as isize) as i32 {
                *a.offset((j + 1) as isize) = *a.offset(j as isize); /* Shift value */
                *idx.offset((j + 1) as isize) = *idx.offset(j as isize); /* Shift index */
                j -= 1;
            }
            *a.offset((j + 1) as isize) = value as i16; /* Write value */
            *idx.offset((j + 1) as isize) = i; /* Write index */
            i += 1;
        }

        /* If less than L values are asked for, check the remaining values, */
        /* but only spend CPU to ensure that the K first values are correct */
        i = k;
        while i < l {
            let value = *a.offset(i as isize) as i32;
            if value > *a.offset((k - 1) as isize) as i32 {
                let mut j = k - 2;
                while j >= 0 && value > *a.offset(j as isize) as i32 {
                    *a.offset((j + 1) as isize) = *a.offset(j as isize); /* Shift value */
                    *idx.offset((j + 1) as isize) = *idx.offset(j as isize); /* Shift index */
                    j -= 1;
                }
                *a.offset((j + 1) as isize) = value as i16; /* Write value */
                *idx.offset((j + 1) as isize) = i; /* Write index */
            }
            i += 1;
        }
    }
}

/// `silk_insertion_sort_increasing_all_values_int16` — sort ALL elements
/// of an `i16` vector in increasing order (no index tracking).
pub unsafe fn silk_insertion_sort_increasing_all_values_int16(a: *mut i16, l: i32) {
    unsafe {
        /* Sort vector elements by value, increasing order */
        let mut i = 1;
        while i < l {
            let value = *a.offset(i as isize) as i32;
            let mut j = i - 1;
            while j >= 0 && value < *a.offset(j as isize) as i32 {
                *a.offset((j + 1) as isize) = *a.offset(j as isize); /* Shift value */
                j -= 1;
            }
            *a.offset((j + 1) as isize) = value as i16; /* Write value */
            i += 1;
        }
    }
}
