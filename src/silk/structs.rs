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

/// Constants from `c/silk/define.h` used to size decoder state arrays.
///
/// `MAX_FRAME_LENGTH = MAX_FRAME_LENGTH_MS * MAX_FS_KHZ = (SUB_FRAME_LENGTH_MS * MAX_NB_SUBFR) * MAX_FS_KHZ = 5 * 4 * 16 = 320`.
/// `MAX_SUB_FRAME_LENGTH = SUB_FRAME_LENGTH_MS * MAX_FS_KHZ = 5 * 16 = 80`.
pub const MAX_FRAMES_PER_PACKET: usize = 3;
pub const MAX_NB_SUBFR: usize = 4;
pub const MAX_LPC_ORDER: usize = 16;
pub const MAX_FRAME_LENGTH: usize = 320;
pub const MAX_SUB_FRAME_LENGTH: usize = 80;
pub const LTP_ORDER: usize = 5;

/// `SideInfoIndices` — per-frame side-information indices carried from
/// the range decoder to later decode stages (`c/silk/structs.h:119`).
#[repr(C)]
pub struct SideInfoIndices {
    pub gains_indices: [i8; MAX_NB_SUBFR],
    pub ltp_index: [i8; MAX_NB_SUBFR],
    pub nlsf_indices: [i8; MAX_LPC_ORDER + 1],
    pub lag_index: i16,
    pub contour_index: i8,
    pub signal_type: i8,
    pub quant_offset_type: i8,
    pub nlsf_interp_coef_q2: i8,
    pub per_index: i8,
    pub ltp_scale_index: i8,
    pub seed: i8,
}

/// `silk_CNG_struct` — comfort-noise generator state (`c/silk/structs.h:249`).
#[repr(C)]
pub struct SilkCngStruct {
    pub cng_exc_buf_q14: [i32; MAX_FRAME_LENGTH],
    pub cng_smth_nlsf_q15: [i16; MAX_LPC_ORDER],
    pub cng_synth_state: [i32; MAX_LPC_ORDER],
    pub cng_smth_gain_q16: i32,
    pub rand_seed: i32,
    pub fs_khz: core::ffi::c_int,
}

/// `silk_PLC_struct` — packet-loss-concealment state (`c/silk/structs.h:232`).
#[repr(C)]
pub struct SilkPlcStruct {
    pub pitch_l_q8: i32,
    pub ltp_coef_q14: [i16; LTP_ORDER],
    pub prev_lpc_q12: [i16; MAX_LPC_ORDER],
    pub last_frame_lost: core::ffi::c_int,
    pub rand_seed: i32,
    pub rand_scale_q14: i16,
    pub conc_energy: i32,
    pub conc_energy_shift: core::ffi::c_int,
    pub prev_ltp_scale_q14: i16,
    pub prev_gain_q16: [i32; 2],
    pub fs_khz: core::ffi::c_int,
    pub nb_subfr: core::ffi::c_int,
    pub subfr_length: core::ffi::c_int,
}

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

/// `silk_decoder_state` — per-channel SILK decoder state
/// (`c/silk/structs.h:261`).
///
/// Field-for-field ABI-compatible with the C struct so the still-in-C
/// dec_API / decode_frame / ... code can share it with the translated
/// Rust functions. Field names are snake_case Rust mirrors of the C
/// names.
#[repr(C)]
pub struct SilkDecoderState {
    pub prev_gain_q16: i32,
    pub exc_q14: [i32; MAX_FRAME_LENGTH],
    pub s_lpc_q14_buf: [i32; MAX_LPC_ORDER],
    pub out_buf: [i16; MAX_FRAME_LENGTH + 2 * MAX_SUB_FRAME_LENGTH],
    pub lag_prev: core::ffi::c_int,
    pub last_gain_index: i8,
    pub fs_khz: core::ffi::c_int,
    pub fs_api_hz: i32,
    pub nb_subfr: core::ffi::c_int,
    pub frame_length: core::ffi::c_int,
    pub subfr_length: core::ffi::c_int,
    pub ltp_mem_length: core::ffi::c_int,
    pub lpc_order: core::ffi::c_int,
    pub prev_nlsf_q15: [i16; MAX_LPC_ORDER],
    pub first_frame_after_reset: core::ffi::c_int,
    pub pitch_lag_low_bits_icdf: *const u8,
    pub pitch_contour_icdf: *const u8,

    pub n_frames_decoded: core::ffi::c_int,
    pub n_frames_per_packet: core::ffi::c_int,

    pub ec_prev_signal_type: core::ffi::c_int,
    pub ec_prev_lag_index: i16,

    pub vad_flags: [core::ffi::c_int; MAX_FRAMES_PER_PACKET],
    pub lbrr_flag: core::ffi::c_int,
    pub lbrr_flags: [core::ffi::c_int; MAX_FRAMES_PER_PACKET],

    pub resampler_state: SilkResamplerStateStruct,

    pub ps_nlsf_cb: *const SilkNlsfCbStruct,

    pub indices: SideInfoIndices,

    pub s_cng: SilkCngStruct,

    pub loss_cnt: core::ffi::c_int,
    pub prev_signal_type: core::ffi::c_int,

    pub s_plc: SilkPlcStruct,
}

// Compile-time ABI assertions against the C sizeof/offsetof measurements
// (see `c/silk/structs.h`). Each constant is the value printed by a small
// C probe — if any of these fires, the Rust struct layout has diverged
// from the C header and the FFI boundary with the still-in-C decoder
// code would corrupt memory.
const _: () = {
    assert!(core::mem::size_of::<SideInfoIndices>() == 36);
    assert!(core::mem::size_of::<SilkCngStruct>() == 1388);
    assert!(core::mem::size_of::<SilkPlcStruct>() == 92);
    assert!(core::mem::size_of::<SilkResamplerStateStruct>() == 304);
    assert!(core::mem::size_of::<SilkDecoderState>() == 4288);
    assert!(core::mem::offset_of!(SilkDecoderState, resampler_state) == 2448);
    assert!(core::mem::offset_of!(SilkDecoderState, indices) == 2760);
    assert!(core::mem::offset_of!(SilkDecoderState, s_plc) == 4192);
};
