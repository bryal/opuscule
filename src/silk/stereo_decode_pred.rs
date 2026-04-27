//! Translated from `c/silk/stereo_decode_pred.c` (RFC 6716).
//!
//! Decodes mid/side stereo predictors and mid-only flag from the range coder.

use crate::entcode::ec_dec;
use crate::entdec::ec_dec_icdf;

use super::macros::{silk_smlabb, silk_smulwb};
use super::tables_other::{
    silk_stereo_only_code_mid_iCDF, silk_stereo_pred_joint_iCDF, silk_stereo_pred_quant_Q13, silk_uniform3_iCDF,
    silk_uniform5_iCDF,
};

const STEREO_QUANT_SUB_STEPS: i32 = 5;

/// `SILK_FIX_CONST(0.5 / STEREO_QUANT_SUB_STEPS, 16)` = `SILK_FIX_CONST(0.1, 16)`
const STEP_SCALE_Q16: i32 = (0.5 / STEREO_QUANT_SUB_STEPS as f64 * (1u32 << 16) as f64 + 0.5) as i32;

/// `silk_stereo_decode_pred` — decode mid/side predictors.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_stereo_decode_pred(ps_range_dec: *mut ec_dec, pred_q13: *mut i32) {
    unsafe {
        let mut ix: [[i32; 3]; 2] = [[0; 3]; 2];

        /* Entropy decoding */
        let n = ec_dec_icdf(ps_range_dec, silk_stereo_pred_joint_iCDF.as_ptr(), 8);
        ix[0][2] = n / 5;
        ix[1][2] = n - 5 * ix[0][2];
        let mut n = 0;
        while n < 2 {
            ix[n as usize][0] = ec_dec_icdf(ps_range_dec, silk_uniform3_iCDF.as_ptr(), 8);
            ix[n as usize][1] = ec_dec_icdf(ps_range_dec, silk_uniform5_iCDF.as_ptr(), 8);
            n += 1;
        }

        /* Dequantize */
        let mut n = 0;
        while n < 2 {
            ix[n][0] += 3 * ix[n][2];
            let low_q13 = silk_stereo_pred_quant_Q13[ix[n][0] as usize] as i32;
            let step_q13 = silk_smulwb(silk_stereo_pred_quant_Q13[ix[n][0] as usize + 1] as i32 - low_q13, STEP_SCALE_Q16);
            *pred_q13.offset(n as isize) = silk_smlabb(low_q13, step_q13, 2 * ix[n][1] + 1);
            n += 1;
        }

        /* Subtract second from first predictor (helps when actually applying these) */
        *pred_q13.offset(0) -= *pred_q13.offset(1);
    }
}

/// `silk_stereo_decode_mid_only` — decode flag that only mid channel is coded.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_stereo_decode_mid_only(ps_range_dec: *mut ec_dec, decode_only_mid: *mut i32) {
    unsafe {
        *decode_only_mid = ec_dec_icdf(ps_range_dec, silk_stereo_only_code_mid_iCDF.as_ptr(), 8);
    }
}
