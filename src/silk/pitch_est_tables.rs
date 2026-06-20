//! Translated from `c/silk/pitch_est_tables.c` (RFC 6716).
//!
//! Pitch estimation codebook lag tables and search ranges.

/// `silk_CB_lags_stage2_10_ms` — stage-2 lag codebook for 10 ms frames (2×3).
pub static SILK_CB_LAGS_STAGE2_10_MS: [[i8; 3]; 2] = [[0, 1, 0], [0, 0, 1]];

/// `silk_CB_lags_stage3_10_ms` — stage-3 lag codebook for 10 ms frames (2×12).
pub static SILK_CB_LAGS_STAGE3_10_MS: [[i8; 12]; 2] =
    [[0, 0, 1, -1, 1, -1, 2, -2, 2, -2, 3, -3], [0, 1, 0, 1, -1, 2, -1, 2, -2, 3, -2, 3]];

/// `silk_CB_lags_stage2` — stage-2 lag codebook for 20 ms frames (4×11).
pub static SILK_CB_LAGS_STAGE2: [[i8; 11]; 4] = [
    [0, 2, -1, -1, -1, 0, 0, 1, 1, 0, 1],
    [0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0],
    [0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0],
    [0, -1, 2, 1, 0, 1, 1, 0, 0, -1, -1],
];

/// `silk_CB_lags_stage3` — stage-3 lag codebook for 20 ms frames (4×34).
pub static SILK_CB_LAGS_STAGE3: [[i8; 34]; 4] = [
    [0, 0, 1, -1, 0, 1, -1, 0, -1, 1, -2, 2, -2, -2, 2, -3, 2, 3, -3, -4, 3, -4, 4, 4, -5, 5, -6, -5, 6, -7, 6, 5, 8, -9],
    [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, -1, 1, 0, 0, 1, -1, 0, 1, -1, -1, 1, -1, 2, 1, -1, 2, -2, -2, 2, -2, 2, 2, 3, -3],
    [0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, -1, 1, 0, 0, 2, 1, -1, 2, -1, -1, 2, -1, 2, 2, -1, 3, -2, -2, -2, 3],
    [0, 1, 0, 0, 1, 0, 1, -1, 2, -1, 2, -1, 2, 3, -2, 3, -2, -2, 4, 4, -3, 5, -3, -4, 6, -4, 6, 5, -5, 8, -6, -5, -7, 9],
];
