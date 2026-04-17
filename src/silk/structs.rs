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

/// `stereo_dec_state` — stereo decoder state.
#[repr(C)]
pub struct StereoDecState {
    pub pred_prev_q13: [i16; 2],
    pub s_mid: [i16; 2],
    pub s_side: [i16; 2],
}

/// `SILK_RESAMPLER_MAX_FIR_ORDER` / `SILK_RESAMPLER_MAX_IIR_ORDER` from
/// `c/silk/resampler_structs.h` — sizes of the FIR and IIR state arrays
/// embedded in [`SilkResamplerStateStruct`].
pub const SILK_RESAMPLER_MAX_FIR_ORDER: usize = 36;
pub const SILK_RESAMPLER_MAX_IIR_ORDER: usize = 6;

/// `silk_resampler_state_struct` — per-channel resampler state.
///
/// The C struct is shared with parts of the decoder that are still in C
/// (the dispatcher in `resampler.c`, `silk/init_decoder.c`, etc.); the
/// `#[repr(C)]` layout and `c_int` choice for `opus_int` fields make the
/// Rust definition ABI-compatible so the two sides can interoperate
/// field-for-field. The comment in the C header that `sIIR` must be the
/// first element is preserved by keeping it at offset 0.
#[repr(C)]
pub struct SilkResamplerStateStruct {
    pub s_iir: [i32; SILK_RESAMPLER_MAX_IIR_ORDER],
    pub s_fir: [i32; SILK_RESAMPLER_MAX_FIR_ORDER],
    pub delay_buf: [i16; 48],
    pub resampler_function: core::ffi::c_int,
    pub batch_size: core::ffi::c_int,
    pub inv_ratio_q16: i32,
    pub fir_order: core::ffi::c_int,
    pub fir_fracs: core::ffi::c_int,
    pub fs_in_khz: core::ffi::c_int,
    pub fs_out_khz: core::ffi::c_int,
    pub input_delay: core::ffi::c_int,
    pub coefs: *const i16,
}
