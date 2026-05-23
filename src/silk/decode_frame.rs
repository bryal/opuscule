//! Translated from `c/silk/decode_frame.c` (RFC 6716).
//!
//! Orchestrates decoding of a single SILK frame: side-info indices,
//! pulses, parameters, core NSQ, PLC state update or loss
//! concealment, output-buffer rotation, PLC glue, and CNG.

use crate::entcode::ec_dec;

use super::CNG::silk_cng;
use super::PLC::{silk_plc, silk_plc_glue_frames};
use super::decode_core::silk_decode_core;
use super::decode_indices::silk_decode_indices;
use super::decode_parameters::silk_decode_parameters;
use super::decode_pulses::silk_decode_pulses;
use super::structs::{MAX_FRAME_LENGTH, SilkDecoderControl, SilkDecoderState};

const FLAG_DECODE_NORMAL: i32 = 0;
const FLAG_DECODE_LBRR: i32 = 2;

/// `silk_decode_frame` — decode one SILK frame.
pub unsafe fn silk_decode_frame(
    ps_dec: *mut SilkDecoderState,
    ps_range_dec: *mut ec_dec,
    p_out: *mut i16,
    p_n: *mut i32,
    lost_flag: i32,
    cond_coding: i32,
) -> i32 {
    unsafe {
        let mut s_dec_ctrl: SilkDecoderControl = core::mem::zeroed();
        let ret = 0i32;
        let mut pulses = [0i32; MAX_FRAME_LENGTH];

        let mut l = (*ps_dec).frame_length;
        s_dec_ctrl.ltp_scale_q14 = 0;

        if lost_flag == FLAG_DECODE_NORMAL
            || (lost_flag == FLAG_DECODE_LBRR && (*ps_dec).lbrr_flags[(*ps_dec).n_frames_decoded as usize] == 1)
        {
            /* Decode quantization indices of side info */
            let n_frames_decoded = (*ps_dec).n_frames_decoded;
            silk_decode_indices(&mut *ps_dec, ps_range_dec, n_frames_decoded, lost_flag, cond_coding);

            /* Decode quantization indices of excitation */
            silk_decode_pulses(
                ps_range_dec,
                &mut pulses,
                (*ps_dec).indices.signal_type as i32,
                (*ps_dec).indices.quant_offset_type as i32,
                (*ps_dec).frame_length,
            );

            /* Decode parameters and pulse signal */
            silk_decode_parameters(&mut *ps_dec, &mut s_dec_ctrl, cond_coding);

            /* Update length. Sampling frequency may have changed */
            l = (*ps_dec).frame_length;

            /* Run inverse NSQ */
            silk_decode_core(ps_dec, &mut s_dec_ctrl, p_out, pulses.as_ptr());

            /* Update PLC state */
            silk_plc(&mut *ps_dec, &mut s_dec_ctrl, core::slice::from_raw_parts_mut(p_out, (*ps_dec).frame_length as usize), 0);

            (*ps_dec).loss_cnt = 0;
            (*ps_dec).prev_signal_type = (*ps_dec).indices.signal_type as i32;

            /* A frame has been decoded without errors */
            (*ps_dec).first_frame_after_reset = 0;
        } else {
            /* Handle packet loss by extrapolation */
            silk_plc(&mut *ps_dec, &mut s_dec_ctrl, core::slice::from_raw_parts_mut(p_out, (*ps_dec).frame_length as usize), 1);
        }

        /* Update output buffer. */
        let mv_len = (*ps_dec).ltp_mem_length - (*ps_dec).frame_length;
        core::ptr::copy(
            (*ps_dec).out_buf.as_ptr().offset((*ps_dec).frame_length as isize),
            (*ps_dec).out_buf.as_mut_ptr(),
            mv_len as usize,
        );
        core::ptr::copy_nonoverlapping(
            p_out,
            (*ps_dec).out_buf.as_mut_ptr().offset(mv_len as isize),
            (*ps_dec).frame_length as usize,
        );

        /* Ensure smooth connection of extrapolated and good frames */
        silk_plc_glue_frames(&mut *ps_dec, core::slice::from_raw_parts_mut(p_out, l as usize));

        /* Comfort noise generation / estimation */
        silk_cng(&mut *ps_dec, &s_dec_ctrl, core::slice::from_raw_parts_mut(p_out, l as usize));

        /* Update some decoder state variables */
        (*ps_dec).lag_prev = s_dec_ctrl.pitch_l[((*ps_dec).nb_subfr - 1) as usize];

        /* Set output frame length */
        *p_n = l;

        ret
    }
}
