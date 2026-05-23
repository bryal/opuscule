//! Translated from `c/silk/stereo_decode_pred.c` (RFC 6716).
//!
//! Decodes mid/side stereo predictors and mid-only flag from the range coder.

use crate::entcode::ec_dec;
use crate::entdec::ec_dec_icdf;

use super::macros::{silk_smlabb, silk_smulwb};
use super::tables_other::{
    SILK_STEREO_ONLY_CODE_MID_ICDF, SILK_STEREO_PRED_JOINT_ICDF, SILK_STEREO_PRED_QUANT_Q13, SILK_UNIFORM3_ICDF,
    SILK_UNIFORM5_ICDF,
};

const STEREO_QUANT_SUB_STEPS: i32 = 5;

/// `SILK_FIX_CONST(0.5 / STEREO_QUANT_SUB_STEPS, 16)` = `SILK_FIX_CONST(0.1, 16)`
const STEP_SCALE_Q16: i32 = (0.5 / STEREO_QUANT_SUB_STEPS as f64 * (1u32 << 16) as f64 + 0.5) as i32;

/// `silk_stereo_decode_pred` — decode mid/side predictors.
pub unsafe fn silk_stereo_decode_pred(ps_range_dec: &mut ec_dec, pred_q13: &mut [i32; 2]) {
    unsafe {
        let mut ix: [[i32; 3]; 2] = [[0; 3]; 2];

        /* Entropy decoding */
        let n = ec_dec_icdf(ps_range_dec, SILK_STEREO_PRED_JOINT_ICDF.as_ptr(), 8);
        ix[0][2] = n / 5;
        ix[1][2] = n - 5 * ix[0][2];
        for n in 0..2 {
            ix[n][0] = ec_dec_icdf(ps_range_dec, SILK_UNIFORM3_ICDF.as_ptr(), 8);
            ix[n][1] = ec_dec_icdf(ps_range_dec, SILK_UNIFORM5_ICDF.as_ptr(), 8);
        }

        /* Dequantize */
        for n in 0..2 {
            ix[n][0] += 3 * ix[n][2];
            let low_q13 = SILK_STEREO_PRED_QUANT_Q13[ix[n][0] as usize] as i32;
            let step_q13 = silk_smulwb(SILK_STEREO_PRED_QUANT_Q13[ix[n][0] as usize + 1] as i32 - low_q13, STEP_SCALE_Q16);
            pred_q13[n] = silk_smlabb(low_q13, step_q13, 2 * ix[n][1] + 1);
        }

        /* Subtract second from first predictor (helps when actually applying these) */
        pred_q13[0] -= pred_q13[1];
    }
}

/// `silk_stereo_decode_mid_only` — decode flag that only mid channel is coded.
pub unsafe fn silk_stereo_decode_mid_only(ps_range_dec: &mut ec_dec, decode_only_mid: &mut i32) {
    unsafe {
        *decode_only_mid = ec_dec_icdf(ps_range_dec, SILK_STEREO_ONLY_CODE_MID_ICDF.as_ptr(), 8);
    }
}
