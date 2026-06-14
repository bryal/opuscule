//! Translated from `c/silk/sigm_Q15.c` (RFC 6716).
//!
//! Approximate sigmoid function using lookup tables with linear
//! interpolation.

use super::macros::silk_smulbb;

/// Slopes of the piecewise-linear segments (Q10).
/// `round(1024 * ([1./(1+exp(-(1:5))), 1] - 1./(1+exp(-(0:5)))))`
static SIGM_LUT_SLOPE_Q10: [i32; 6] = [237, 153, 73, 30, 12, 7];

/// Positive-side sigmoid values (Q15).
/// `round(32767 * 1./(1+exp(-(0:5))))`
static SIGM_LUT_POS_Q15: [i32; 6] = [16384, 23955, 28861, 31213, 32178, 32548];

/// Negative-side sigmoid values (Q15).
/// `round(32767 * 1./(1+exp((0:5))))`
static SIGM_LUT_NEG_Q15: [i32; 6] = [16384, 8812, 3906, 1554, 589, 219];

/// `silk_sigm_Q15` — approximate sigmoid, Q15 output for Q5 input.
///
/// `ind = in_q5 >> 5` selects the table segment; `ind >= 6` (i.e. the C's
/// `in_q5 >= 6*32` clip case) falls out of the lookup table, so a failed
/// `.get()` IS the clip condition.
pub fn silk_sigm_q15(in_q5: i32) -> i32 {
    if in_q5 < 0 {
        /* Negative input */
        let in_q5 = -in_q5;
        let ind = (in_q5 >> 5) as usize;
        let (Some(&base), Some(&slope)) = (SIGM_LUT_NEG_Q15.get(ind), SIGM_LUT_SLOPE_Q10.get(ind)) else {
            return 0; /* Clip */
        };
        base - silk_smulbb(slope, in_q5 & 0x1F)
    } else {
        /* Positive input */
        let ind = (in_q5 >> 5) as usize;
        let (Some(&base), Some(&slope)) = (SIGM_LUT_POS_Q15.get(ind), SIGM_LUT_SLOPE_Q10.get(ind)) else {
            return 32767; /* Clip */
        };
        base + silk_smulbb(slope, in_q5 & 0x1F)
    }
}
