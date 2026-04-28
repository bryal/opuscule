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
pub fn silk_sigm_Q15(in_q5: i32) -> i32 {
    if in_q5 < 0 {
        /* Negative input */
        let in_q5 = -in_q5;
        if in_q5 >= 6 * 32 {
            return 0; /* Clip */
        } else {
            /* Linear interpolation of look up table */
            let ind = (in_q5 >> 5) as usize;
            return SIGM_LUT_NEG_Q15[ind] - silk_smulbb(SIGM_LUT_SLOPE_Q10[ind], in_q5 & 0x1F);
        }
    } else {
        /* Positive input */
        if in_q5 >= 6 * 32 {
            return 32767; /* clip */
        } else {
            /* Linear interpolation of look up table */
            let ind = (in_q5 >> 5) as usize;
            return SIGM_LUT_POS_Q15[ind] + silk_smulbb(SIGM_LUT_SLOPE_Q10[ind], in_q5 & 0x1F);
        }
    }
}
