//! Translated from `c/silk/NLSF_unpack.c` (RFC 6716).
//!
//! Unpacks predictor values and entropy-coding indices from the NLSF
//! codebook.

use super::macros::silk_smulbb;
use super::structs::SilkNlsfCbStruct;

const NLSF_QUANT_MAX_AMPLITUDE: i32 = 4;

/// `silk_NLSF_unpack` — unpack predictor values and indices for entropy
/// coding tables from the codebook.
pub fn silk_nlsf_unpack(ec_ix: &mut [i16], pred_q8: &mut [u8], ps_nlsf_cb: &SilkNlsfCbStruct, cb1_index: i32) {
    let order = ps_nlsf_cb.order as i32;
    let ec_sel = &ps_nlsf_cb.ec_sel[(cb1_index * order / 2) as usize..];
    let mut i = 0i32;
    while i < order {
        let entry = ec_sel[(i / 2) as usize];
        ec_ix[i as usize] = silk_smulbb((entry as i32 >> 1) & 7, 2 * NLSF_QUANT_MAX_AMPLITUDE + 1) as i16;
        pred_q8[i as usize] = ps_nlsf_cb.pred_q8[(i + (entry as i32 & 1) * (order - 1)) as usize];
        ec_ix[(i + 1) as usize] = silk_smulbb((entry as i32 >> 5) & 7, 2 * NLSF_QUANT_MAX_AMPLITUDE + 1) as i16;
        pred_q8[(i + 1) as usize] = ps_nlsf_cb.pred_q8[(i + ((entry as i32 >> 4) & 1) * (order - 1) + 1) as usize];
        i += 2;
    }
}
