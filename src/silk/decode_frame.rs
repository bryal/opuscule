//! Translated from `c/silk/decode_frame.c` (RFC 6716).
//!
//! Orchestrates decoding of a single SILK frame: side-info indices,
//! pulses, parameters, core NSQ, PLC state update or loss
//! concealment, output-buffer rotation, PLC glue, and CNG.

use crate::entcode::ec_dec;

use super::CNG::silk_cng;
use super::decode_core::silk_decode_core;
use super::decode_indices::silk_decode_indices;
use super::decode_parameters::silk_decode_parameters;
use super::decode_pulses::silk_decode_pulses;
use super::plc::{silk_plc, silk_plc_glue_frames};
use super::structs::{LTP_ORDER, MAX_FRAME_LENGTH, MAX_LPC_ORDER, MAX_NB_SUBFR, SilkDecoderControl, SilkDecoderState};

const FLAG_DECODE_NORMAL: i32 = 0;
const FLAG_DECODE_LBRR: i32 = 2;

/// `silk_decode_frame` — decode one SILK frame.
pub fn silk_decode_frame(
    ps_dec: &mut SilkDecoderState,
    ps_range_dec: &mut ec_dec,
    p_out: &mut [i16],
    p_n: &mut i32,
    lost_flag: i32,
    cond_coding: i32,
) -> i32 {
    /* C: silk_decoder_control sDecCtrl — stack struct, zero-initialised */
    let mut s_dec_ctrl = SilkDecoderControl {
        pitch_l: [0; MAX_NB_SUBFR],
        gains_q16: [0; MAX_NB_SUBFR],
        pred_coef_q12: [[0; MAX_LPC_ORDER]; 2],
        ltp_coef_q14: [0; LTP_ORDER * MAX_NB_SUBFR],
        ltp_scale_q14: 0,
    };
    let ret = 0i32;
    let mut pulses = [0i32; MAX_FRAME_LENGTH];

    let mut l = ps_dec.frame_length;
    s_dec_ctrl.ltp_scale_q14 = 0;

    if lost_flag == FLAG_DECODE_NORMAL
        || (lost_flag == FLAG_DECODE_LBRR && ps_dec.lbrr_flags[ps_dec.n_frames_decoded as usize] == 1)
    {
        /* Decode quantization indices of side info */
        let n_frames_decoded = ps_dec.n_frames_decoded;
        silk_decode_indices(ps_dec, ps_range_dec, n_frames_decoded, lost_flag, cond_coding);

        /* Decode quantization indices of excitation */
        silk_decode_pulses(
            ps_range_dec,
            &mut pulses,
            ps_dec.indices.signal_type as i32,
            ps_dec.indices.quant_offset_type as i32,
            ps_dec.frame_length,
        );

        /* Decode parameters and pulse signal */
        silk_decode_parameters(ps_dec, &mut s_dec_ctrl, cond_coding);

        /* Update length. Sampling frequency may have changed */
        l = ps_dec.frame_length;

        /* Run inverse NSQ */
        silk_decode_core(ps_dec, &mut s_dec_ctrl, p_out, &pulses);

        /* Update PLC state */
        let frame_length = ps_dec.frame_length as usize;
        silk_plc(ps_dec, &mut s_dec_ctrl, &mut p_out[..frame_length], 0);

        ps_dec.loss_cnt = 0;
        ps_dec.prev_signal_type = ps_dec.indices.signal_type as i32;

        /* A frame has been decoded without errors */
        ps_dec.first_frame_after_reset = 0;
    } else {
        /* Handle packet loss by extrapolation */
        let frame_length = ps_dec.frame_length as usize;
        silk_plc(ps_dec, &mut s_dec_ctrl, &mut p_out[..frame_length], 1);
    }

    /* Update output buffer. */
    let mv_len = (ps_dec.ltp_mem_length - ps_dec.frame_length) as usize;
    let frame_length = ps_dec.frame_length as usize;
    // In bounds: src end frame_length + mv_len == ltp_mem_length, which silk_decoder_set_fs
    // keeps within out_buf (MAX_FRAME_LENGTH + 2*MAX_SUB_FRAME_LENGTH).
    ps_dec.out_buf.copy_within(frame_length..frame_length + mv_len, 0);
    ps_dec.out_buf[mv_len..mv_len + frame_length].copy_from_slice(&p_out[..frame_length]);

    /* Ensure smooth connection of extrapolated and good frames */
    silk_plc_glue_frames(ps_dec, &mut p_out[..l as usize]);

    /* Comfort noise generation / estimation */
    silk_cng(ps_dec, &s_dec_ctrl, &mut p_out[..l as usize]);

    /* Update some decoder state variables */
    ps_dec.lag_prev = s_dec_ctrl.pitch_l[(ps_dec.nb_subfr - 1) as usize];

    /* Set output frame length */
    *p_n = l;

    ret
}
