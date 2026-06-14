//! Translated from `c/silk/NLSF_decode.c` (RFC 6716).
//!
//! NLSF vector decoder: reconstructs an NLSF vector (Q15) from a codebook
//! path index plus per-coefficient quantization indices, applies inverse
//! Laroia weighting to mix the residual back in, and finally stabilizes
//! the result so it satisfies the codec's monotonicity / minimum-spacing
//! constraints.

#![allow(clippy::indexing_slicing)] // dense SILK kernels; voice path is deprioritized vs CELT

use super::macros::{silk_limit_int, silk_lshift, silk_smlawb, silk_smulbb};
use super::nlsf_stabilize::silk_nlsf_stabilize;
use super::nlsf_unpack::silk_nlsf_unpack;
use super::nlsf_vq_weights_laroia::silk_nlsf_vq_weights_laroia;
use super::sqrt_approx::silk_sqrt_approx;
use super::structs::SilkNlsfCbStruct;

const MAX_LPC_ORDER: usize = 16;
const NLSF_W_Q: i32 = 2;
/// `SILK_FIX_CONST( NLSF_QUANT_LEVEL_ADJ, 10 )` — 0.1 in Q10.
const NLSF_QUANT_LEVEL_ADJ_Q10: i32 = (0.1 * (1u32 << 10) as f64 + 0.5) as i32;

/// Predictive dequantizer for NLSF residuals.
fn silk_nlsf_residual_dequant(x_q10: &mut [i16], indices: &[i8], pred_coef_q8: &[u8], quant_step_size_q16: i32, order: i16) {
    let mut out_q10: i32 = 0;
    let mut i = order as i32 - 1;
    while i >= 0 {
        let pred_q10 = silk_smulbb(out_q10, pred_coef_q8[i as usize] as i16 as i32) >> 8;
        out_q10 = silk_lshift(indices[i as usize] as i32, 10);
        if out_q10 > 0 {
            out_q10 -= NLSF_QUANT_LEVEL_ADJ_Q10;
        } else if out_q10 < 0 {
            out_q10 += NLSF_QUANT_LEVEL_ADJ_Q10;
        }
        out_q10 = silk_smlawb(pred_q10, out_q10, quant_step_size_q16);
        x_q10[i as usize] = out_q10 as i16;
        i -= 1;
    }
}

/// `silk_NLSF_decode` — NLSF vector decoder.
///
/// `p_nlsf_q15` must be at least `ps_nlsf_cb.order` long; `nlsf_indices`
/// must be at least `ps_nlsf_cb.order + 1` long.
pub fn silk_nlsf_decode(p_nlsf_q15: &mut [i16], nlsf_indices: &[i8], ps_nlsf_cb: &SilkNlsfCbStruct) {
    let mut pred_q8 = [0u8; MAX_LPC_ORDER];
    let mut ec_ix = [0i16; MAX_LPC_ORDER];
    let mut res_q10 = [0i16; MAX_LPC_ORDER];
    let mut w_tmp_qw = [0i16; MAX_LPC_ORDER];

    let order = ps_nlsf_cb.order as i32;

    /* Decode first stage */
    let cb_off = (nlsf_indices[0] as i32 * order) as usize;
    let p_cb_element = &ps_nlsf_cb.cb1_nlsf_q8[cb_off..cb_off + order as usize];
    for i in 0..order as usize {
        p_nlsf_q15[i] = silk_lshift(p_cb_element[i] as i16 as i32, 7) as i16;
    }

    /* Unpack entropy table indices and predictor for current CB1 index */
    silk_nlsf_unpack(&mut ec_ix, &mut pred_q8, ps_nlsf_cb, nlsf_indices[0] as i32);

    /* Predictive residual dequantizer */
    silk_nlsf_residual_dequant(
        &mut res_q10,
        &nlsf_indices[1..1 + order as usize],
        &pred_q8,
        ps_nlsf_cb.quant_step_size_q16 as i32,
        ps_nlsf_cb.order,
    );

    /* Weights from codebook vector */
    silk_nlsf_vq_weights_laroia(&mut w_tmp_qw[..order as usize], &p_nlsf_q15[..order as usize]);

    /* Apply inverse square-rooted weights and add to output */
    for i in 0..order as usize {
        let w_tmp_q9 = silk_sqrt_approx(silk_lshift(w_tmp_qw[i] as i32, 18 - NLSF_W_Q));
        let nlsf_q15_tmp = p_nlsf_q15[i] as i32 + silk_lshift(res_q10[i] as i32, 14) / w_tmp_q9;
        p_nlsf_q15[i] = silk_limit_int(nlsf_q15_tmp, 0, 32767) as i16;
    }

    /* NLSF stabilization */
    silk_nlsf_stabilize(&mut p_nlsf_q15[..order as usize], ps_nlsf_cb.delta_min_q15, order);
}
