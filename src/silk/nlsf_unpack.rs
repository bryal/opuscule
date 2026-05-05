//! Translated from `c/silk/NLSF_unpack.c` (RFC 6716).
//!
//! Unpacks predictor values and entropy-coding indices from the NLSF
//! codebook.

use super::macros::silk_smulbb;
use super::structs::SilkNlsfCbStruct;

const NLSF_QUANT_MAX_AMPLITUDE: i32 = 4;

/// `silk_NLSF_unpack` — unpack predictor values and indices for entropy
/// coding tables from the codebook.
pub unsafe fn silk_nlsf_unpack(ec_ix: *mut i16, pred_q8: *mut u8, ps_nlsf_cb: *const SilkNlsfCbStruct, cb1_index: i32) {
    unsafe {
        let order = (*ps_nlsf_cb).order as i32;
        let mut ec_sel_ptr = (*ps_nlsf_cb).ec_sel.offset((cb1_index * order / 2) as isize);
        let mut i = 0;
        while i < order {
            let entry = *ec_sel_ptr;
            ec_sel_ptr = ec_sel_ptr.offset(1);
            *ec_ix.offset(i as isize) = silk_smulbb((entry as i32 >> 1) & 7, 2 * NLSF_QUANT_MAX_AMPLITUDE + 1) as i16;
            *pred_q8.offset(i as isize) = *(*ps_nlsf_cb).pred_q8.offset((i + (entry as i32 & 1) * (order - 1)) as isize);
            *ec_ix.offset((i + 1) as isize) = silk_smulbb((entry as i32 >> 5) & 7, 2 * NLSF_QUANT_MAX_AMPLITUDE + 1) as i16;
            *pred_q8.offset((i + 1) as isize) =
                *(*ps_nlsf_cb).pred_q8.offset((i + ((entry as i32 >> 4) & 1) * (order - 1) + 1) as isize);
            i += 2;
        }
    }
}
