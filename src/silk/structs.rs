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
    pub cb1_nlsf_q8: &'static [u8],
    pub cb1_icdf: &'static [u8],
    pub pred_q8: &'static [u8],
    pub ec_sel: &'static [u8],
    pub ec_icdf: &'static [u8],
    pub delta_min_q15: &'static [i16],
}

/// `stereo_dec_state` — stereo decoder state.
#[derive(Default)]
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
    pub fs_khz: i32,
}

/// `silk_PLC_struct` — packet-loss-concealment state (`c/silk/structs.h:232`).
pub struct SilkPlcStruct {
    pub pitch_l_q8: i32,
    pub ltp_coef_q14: [i16; LTP_ORDER],
    pub prev_lpc_q12: [i16; MAX_LPC_ORDER],
    pub last_frame_lost: i32,
    pub rand_seed: i32,
    pub rand_scale_q14: i16,
    pub conc_energy: i32,
    pub conc_energy_shift: i32,
    pub prev_ltp_scale_q14: i16,
    pub prev_gain_q16: [i32; 2],
    pub fs_khz: i32,
    pub nb_subfr: i32,
    pub subfr_length: i32,
}

/// `silk_resampler_state_struct` — per-channel resampler state.
/// Clone matches the C's whole-struct memcpy in silk_Decode's
/// stereo-transition path (all fields are plain values).
#[derive(Clone)]
pub struct SilkResamplerStateStruct {
    pub s_iir: [i32; SILK_RESAMPLER_MAX_IIR_ORDER],
    pub s_fir: [i32; SILK_RESAMPLER_MAX_FIR_ORDER],
    pub delay_buf: [i16; 48],
    pub resampler_function: i32,
    pub batch_size: i32,
    pub inv_ratio_q16: i32,
    pub fir_order: i32,
    pub fir_fracs: i32,
    pub fs_in_khz: i32,
    pub fs_out_khz: i32,
    pub input_delay: i32,
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
    pub lag_prev: i32,
    pub last_gain_index: i8,
    pub fs_khz: i32,
    pub fs_api_hz: i32,
    pub nb_subfr: i32,
    pub frame_length: i32,
    pub subfr_length: i32,
    pub ltp_mem_length: i32,
    pub lpc_order: i32,
    pub prev_nlsf_q15: [i16; MAX_LPC_ORDER],
    pub first_frame_after_reset: i32,
    pub pitch_lag_low_bits_icdf: Option<&'static [u8]>,
    pub pitch_contour_icdf: Option<&'static [u8]>,

    pub n_frames_decoded: i32,
    pub n_frames_per_packet: i32,

    pub ec_prev_signal_type: i32,
    pub ec_prev_lag_index: i16,

    pub vad_flags: [i32; MAX_FRAMES_PER_PACKET],
    pub lbrr_flag: i32,
    pub lbrr_flags: [i32; MAX_FRAMES_PER_PACKET],

    pub resampler_state: SilkResamplerStateStruct,

    pub ps_nlsf_cb: Option<&'static SilkNlsfCbStruct>,

    pub indices: SideInfoIndices,

    pub s_cng: SilkCngStruct,

    pub loss_cnt: i32,
    pub prev_signal_type: i32,

    pub s_plc: SilkPlcStruct,
}

/// The freshly-initialised decoder state (the C `silk_init_decoder`
/// reset value). The CNG and PLC sub-states are constructed inline with
/// the same values that `silk_cng_reset` / `silk_plc_reset` would produce
/// when run against a freshly-zeroed state (lpc_order == 0,
/// frame_length == 0).
impl Default for SilkDecoderState {
    fn default() -> Self {
        SilkDecoderState {
            prev_gain_q16: 65536,
            exc_q14: [0; MAX_FRAME_LENGTH],
            s_lpc_q14_buf: [0; MAX_LPC_ORDER],
            out_buf: [0; MAX_FRAME_LENGTH + 2 * MAX_SUB_FRAME_LENGTH],
            lag_prev: 0,
            last_gain_index: 0,
            fs_khz: 0,
            fs_api_hz: 0,
            nb_subfr: 0,
            frame_length: 0,
            subfr_length: 0,
            ltp_mem_length: 0,
            lpc_order: 0,
            prev_nlsf_q15: [0; MAX_LPC_ORDER],
            first_frame_after_reset: 1,
            pitch_lag_low_bits_icdf: None,
            pitch_contour_icdf: None,
            n_frames_decoded: 0,
            n_frames_per_packet: 0,
            ec_prev_signal_type: 0,
            ec_prev_lag_index: 0,
            vad_flags: [0; MAX_FRAMES_PER_PACKET],
            lbrr_flag: 0,
            lbrr_flags: [0; MAX_FRAMES_PER_PACKET],
            resampler_state: SilkResamplerStateStruct {
                s_iir: [0; SILK_RESAMPLER_MAX_IIR_ORDER],
                s_fir: [0; SILK_RESAMPLER_MAX_FIR_ORDER],
                delay_buf: [0; 48],
                resampler_function: 0,
                batch_size: 0,
                inv_ratio_q16: 0,
                fir_order: 0,
                fir_fracs: 0,
                fs_in_khz: 0,
                fs_out_khz: 0,
                input_delay: 0,
                coefs: None,
            },
            ps_nlsf_cb: None,
            indices: SideInfoIndices {
                gains_indices: [0; MAX_NB_SUBFR],
                ltp_index: [0; MAX_NB_SUBFR],
                nlsf_indices: [0; MAX_LPC_ORDER + 1],
                lag_index: 0,
                contour_index: 0,
                signal_type: 0,
                quant_offset_type: 0,
                nlsf_interp_coef_q2: 0,
                per_index: 0,
                ltp_scale_index: 0,
                seed: 0,
            },
            s_cng: SilkCngStruct {
                cng_exc_buf_q14: [0; MAX_FRAME_LENGTH],
                cng_smth_nlsf_q15: [0; MAX_LPC_ORDER],
                cng_synth_state: [0; MAX_LPC_ORDER],
                cng_smth_gain_q16: 0,
                rand_seed: 3176576,
                fs_khz: 0,
            },
            loss_cnt: 0,
            prev_signal_type: 0,
            s_plc: SilkPlcStruct {
                pitch_l_q8: 0,
                ltp_coef_q14: [0; LTP_ORDER],
                prev_lpc_q12: [0; MAX_LPC_ORDER],
                last_frame_lost: 0,
                rand_seed: 0,
                rand_scale_q14: 0,
                conc_energy: 0,
                conc_energy_shift: 0,
                prev_ltp_scale_q14: 0,
                prev_gain_q16: [65536, 65536],
                fs_khz: 0,
                nb_subfr: 2,
                subfr_length: 20,
            },
        }
    }
}

/// `silk_decoder_control` — per-frame decoder output of
/// `silk_decode_parameters` (`c/silk/structs.h:313`). Holds dequantized
/// gains, LPC prediction coefficients (with first/second-half interp),
/// pitch lags, LTP coefficients, and the LTP scale.
pub struct SilkDecoderControl {
    pub pitch_l: [i32; MAX_NB_SUBFR],
    pub gains_q16: [i32; MAX_NB_SUBFR],
    /// Holds interpolated and final coefficients; the C side tags this
    /// 4-byte aligned via `silk_DWORD_ALIGN`, but the natural alignment
    /// of `i16` arrays already satisfies that on every supported target.
    pub pred_coef_q12: [[i16; MAX_LPC_ORDER]; 2],
    pub ltp_coef_q14: [i16; LTP_ORDER * MAX_NB_SUBFR],
    pub ltp_scale_q14: i32,
}
