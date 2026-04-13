//! SILK decoder structure definitions.
//!
//! Translated from `c/silk/structs.h` (RFC 6716). Types are added on demand
//! as the translation progresses.

/// `silk_NLSF_CB_struct` — NLSF codebook descriptor.
///
/// Contains pointers to the various lookup tables used by the NLSF
/// quantization / dequantization pipeline. Static instances are defined
/// in `tables_NLSF_CB_NB_MB.c` and `tables_NLSF_CB_WB.c`.
#[repr(C)]
pub struct SilkNlsfCbStruct {
    pub n_vectors: i16,
    pub order: i16,
    pub quant_step_size_q16: i16,
    pub inv_quant_step_size_q6: i16,
    pub cb1_nlsf_q8: *const u8,
    pub cb1_icdf: *const u8,
    pub pred_q8: *const u8,
    pub ec_sel: *const u8,
    pub ec_icdf: *const u8,
    pub ec_rates_q5: *const u8,
    pub delta_min_q15: *const i16,
}

// Safety: the struct contains only raw pointers to static data and
// primitive fields. The static instances in the tables are truly immutable
// and live for the entire program, so sharing across threads is safe.
unsafe impl Sync for SilkNlsfCbStruct {}
unsafe impl Send for SilkNlsfCbStruct {}
