//! SILK decoder structure definitions.
//!
//! Translated from `c/silk/structs.h` (RFC 6716). Types are added on demand
//! as the translation progresses.

/// `silk_NLSF_CB_struct` — NLSF codebook descriptor.
///
/// Contains references to the various lookup tables used by the NLSF
/// quantization / dequantization pipeline. Static instances are defined
/// in `tables_NLSF_CB_NB_MB.c` and `tables_NLSF_CB_WB.c`.
pub struct SilkNlsfCbStruct {
    pub n_vectors: i16,
    pub order: i16,
    pub quant_step_size_q16: i16,
    pub inv_quant_step_size_q6: i16,
    pub cb1_nlsf_q8: &'static [u8],
    pub cb1_icdf: &'static [u8],
    pub pred_q8: &'static [u8],
    pub ec_sel: &'static [u8],
    pub ec_icdf: &'static [u8],
    pub ec_rates_q5: &'static [u8],
    pub delta_min_q15: &'static [i16],
}

/// `stereo_dec_state` — stereo decoder state.
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
pub struct SilkCngStruct {
    pub cng_exc_buf_q14: [i32; MAX_FRAME_LENGTH],
    pub cng_smth_nlsf_q15: [i16; MAX_LPC_ORDER],
    pub cng_synth_state: [i32; MAX_LPC_ORDER],
    pub cng_smth_gain_q16: i32,
    pub rand_seed: i32,
    pub fs_khz: core::ffi::c_int,
}

/// `silk_PLC_struct` — packet-loss-concealment state (`c/silk/structs.h:232`).
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
    pub coefs: Option<&'static [i16]>,
}

/// `silk_decoder_state` — per-channel SILK decoder state
/// (`c/silk/structs.h:261`). Field names are snake_case Rust mirrors of
/// the C names.
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
    pub pitch_lag_low_bits_icdf: Option<&'static [u8]>,
    pub pitch_contour_icdf: Option<&'static [u8]>,

    pub n_frames_decoded: core::ffi::c_int,
    pub n_frames_per_packet: core::ffi::c_int,

    pub ec_prev_signal_type: core::ffi::c_int,
    pub ec_prev_lag_index: i16,

    pub vad_flags: [core::ffi::c_int; MAX_FRAMES_PER_PACKET],
    pub lbrr_flag: core::ffi::c_int,
    pub lbrr_flags: [core::ffi::c_int; MAX_FRAMES_PER_PACKET],

    pub resampler_state: SilkResamplerStateStruct,

    pub ps_nlsf_cb: Option<&'static SilkNlsfCbStruct>,

    pub indices: SideInfoIndices,

    pub s_cng: SilkCngStruct,

    pub loss_cnt: core::ffi::c_int,
    pub prev_signal_type: core::ffi::c_int,

    pub s_plc: SilkPlcStruct,
}

/// `silk_decoder_control` — per-frame decoder output of
/// `silk_decode_parameters` (`c/silk/structs.h:313`). Holds dequantized
/// gains, LPC prediction coefficients (with first/second-half interp),
/// pitch lags, LTP coefficients, and the LTP scale.
pub struct SilkDecoderControl {
    pub pitch_l: [core::ffi::c_int; MAX_NB_SUBFR],
    pub gains_q16: [i32; MAX_NB_SUBFR],
    /// Holds interpolated and final coefficients; the C side tags this
    /// 4-byte aligned via `silk_DWORD_ALIGN`, but the natural alignment
    /// of `i16` arrays already satisfies that on every supported target.
    pub pred_coef_q12: [[i16; MAX_LPC_ORDER]; 2],
    pub ltp_coef_q14: [i16; LTP_ORDER * MAX_NB_SUBFR],
    pub ltp_scale_q14: core::ffi::c_int,
}
