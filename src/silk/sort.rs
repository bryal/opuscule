//! Translated from `c/silk/sort.c` (RFC 6716).
//!
//! Only the all-values variant is used in the decoder; the K-smallest /
//! K-largest variants from the C source live in the encoder.

/// `silk_insertion_sort_increasing_all_values_int16` — sort ALL elements
/// of an `i16` vector in increasing order (no index tracking).
///
/// The C used an in-place insertion sort; sorting plain `i16` values is
/// order-independent for equal elements, so the standard unstable sort
/// produces the identical result.
pub fn silk_insertion_sort_increasing_all_values_int16(a: &mut [i16]) {
    a.sort_unstable();
}
