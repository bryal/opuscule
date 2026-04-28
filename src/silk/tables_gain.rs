//! Translated from `c/silk/tables_gain.c` (RFC 6716).
//!
//! Gain quantization iCDF tables.

/// `silk_gain_iCDF` — iCDF for gain quantization (3 stages × 8 levels).
pub static silk_gain_iCDF: [[u8; 8]; 3] =
    [[224, 112, 44, 15, 3, 2, 1, 0], [254, 237, 192, 132, 70, 23, 4, 0], [255, 252, 226, 155, 61, 11, 2, 0]];

/// `silk_delta_gain_iCDF` — iCDF for delta gain quantization (41 entries).
pub static silk_delta_gain_iCDF: [u8; 41] = [
    250, 245, 234, 203, 71, 50, 42, 38, 35, 33, 31, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11,
    10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0,
];
