//! Translated from `c/silk/sort.c` (RFC 6716).
//!
//! Insertion sort. Only the all-values variant is used in the decoder;
//! the K-smallest / K-largest variants from the C source live in the
//! encoder.

/// `silk_insertion_sort_increasing_all_values_int16` — sort ALL elements
/// of an `i16` vector in increasing order (no index tracking).
pub fn silk_insertion_sort_increasing_all_values_int16(a: &mut [i16]) {
    /* Sort vector elements by value, increasing order */
    for i in 1..a.len() {
        let value = a[i];
        let mut j = i;
        while j > 0 && value < a[j - 1] {
            a[j] = a[j - 1]; /* Shift value */
            j -= 1;
        }
        a[j] = value; /* Write value */
    }
}
