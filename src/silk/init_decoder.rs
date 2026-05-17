//! Translated from `c/silk/init_decoder.c` (RFC 6716).
//!
//! Initialises a `SilkDecoderState`. Called from `silk_Decoder`'s
//! per-channel setup. The CNG and PLC sub-states are constructed inline
//! with the same values that `silk_cng_reset` / `silk_plc_reset` would
//! produce when run against a freshly-zeroed state (lpc_order == 0,
//! frame_length == 0).

use super::structs::{
    LTP_ORDER, MAX_FRAME_LENGTH, MAX_FRAMES_PER_PACKET, MAX_LPC_ORDER, MAX_NB_SUBFR, MAX_SUB_FRAME_LENGTH,
    SILK_RESAMPLER_MAX_FIR_ORDER, SILK_RESAMPLER_MAX_IIR_ORDER, SideInfoIndices, SilkCngStruct, SilkDecoderState,
    SilkPlcStruct, SilkResamplerStateStruct,
};

/// `silk_init_decoder` — initialise the decoder state struct.
pub unsafe fn silk_init_decoder(ps_dec: *mut SilkDecoderState) -> i32 {
    unsafe {
        *ps_dec = SilkDecoderState {
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
        };

        0
    }
}
