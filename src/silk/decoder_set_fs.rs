//! Translated from `c/silk/decoder_set_fs.c` (RFC 6716).
//!
//! Updates subframe/frame length, LPC order, NLSF codebook pointer and
//! pitch / range-decoder-low-bits iCDF pointers when the SILK internal
//! sampling rate changes, and (re)initialises the resampler if the
//! API-side sample rate or internal rate moves.

use super::resampler::silk_resampler_init;
use super::structs::{MAX_LPC_ORDER, MAX_NB_SUBFR, SilkDecoderState};
use super::tables_nlsf_cb_nb_mb::SILK_NLSF_CB_NB_MB;
use super::tables_nlsf_cb_wb::SILK_NLSF_CB_WB;
use super::tables_other::{SILK_UNIFORM4_ICDF, SILK_UNIFORM6_ICDF, SILK_UNIFORM8_ICDF};
use super::tables_pitch_lag::{
    SILK_PITCH_CONTOUR_10_MS_ICDF, SILK_PITCH_CONTOUR_10_MS_NB_ICDF, SILK_PITCH_CONTOUR_ICDF, SILK_PITCH_CONTOUR_NB_ICDF,
};

const SUB_FRAME_LENGTH_MS: i32 = 5;
const LTP_MEM_LENGTH_MS: i32 = 20;
const MIN_LPC_ORDER: i32 = 10;
const TYPE_NO_VOICE_ACTIVITY: i32 = 0;

/// `silk_decoder_set_fs` — set SILK internal / API sample rates.
pub unsafe fn silk_decoder_set_fs(ps_dec: *mut SilkDecoderState, fs_khz: i32, fs_api_hz: i32) -> i32 {
    unsafe {
        let mut ret = 0i32;

        /* New (sub)frame length */
        (*ps_dec).subfr_length = SUB_FRAME_LENGTH_MS * fs_khz;
        let frame_length = (*ps_dec).nb_subfr * (*ps_dec).subfr_length;

        /* Initialize resampler when switching internal or external sampling frequency */
        if (*ps_dec).fs_khz != fs_khz || (*ps_dec).fs_api_hz != fs_api_hz {
            ret += silk_resampler_init(&raw mut (*ps_dec).resampler_state, fs_khz * 1000, fs_api_hz, 0);
            (*ps_dec).fs_api_hz = fs_api_hz;
        }

        if (*ps_dec).fs_khz != fs_khz || frame_length != (*ps_dec).frame_length {
            if fs_khz == 8 {
                if (*ps_dec).nb_subfr == MAX_NB_SUBFR as i32 {
                    (*ps_dec).pitch_contour_icdf = Some(&SILK_PITCH_CONTOUR_NB_ICDF);
                } else {
                    (*ps_dec).pitch_contour_icdf = Some(&SILK_PITCH_CONTOUR_10_MS_NB_ICDF);
                }
            } else {
                if (*ps_dec).nb_subfr == MAX_NB_SUBFR as i32 {
                    (*ps_dec).pitch_contour_icdf = Some(&SILK_PITCH_CONTOUR_ICDF);
                } else {
                    (*ps_dec).pitch_contour_icdf = Some(&SILK_PITCH_CONTOUR_10_MS_ICDF);
                }
            }
            if (*ps_dec).fs_khz != fs_khz {
                (*ps_dec).ltp_mem_length = LTP_MEM_LENGTH_MS * fs_khz;
                if fs_khz == 8 || fs_khz == 12 {
                    (*ps_dec).lpc_order = MIN_LPC_ORDER;
                    (*ps_dec).ps_nlsf_cb = Some(&SILK_NLSF_CB_NB_MB);
                } else {
                    (*ps_dec).lpc_order = MAX_LPC_ORDER as i32;
                    (*ps_dec).ps_nlsf_cb = Some(&SILK_NLSF_CB_WB);
                }
                if fs_khz == 16 {
                    (*ps_dec).pitch_lag_low_bits_icdf = Some(&SILK_UNIFORM8_ICDF);
                } else if fs_khz == 12 {
                    (*ps_dec).pitch_lag_low_bits_icdf = Some(&SILK_UNIFORM6_ICDF);
                } else if fs_khz == 8 {
                    (*ps_dec).pitch_lag_low_bits_icdf = Some(&SILK_UNIFORM4_ICDF);
                } else {
                    /* unsupported sampling rate */
                }
                (*ps_dec).first_frame_after_reset = 1;
                (*ps_dec).lag_prev = 100;
                (*ps_dec).last_gain_index = 10;
                (*ps_dec).prev_signal_type = TYPE_NO_VOICE_ACTIVITY;
                core::ptr::write_bytes((*ps_dec).out_buf.as_mut_ptr(), 0, (*ps_dec).out_buf.len());
                core::ptr::write_bytes((*ps_dec).s_lpc_q14_buf.as_mut_ptr(), 0, (*ps_dec).s_lpc_q14_buf.len());
            }

            (*ps_dec).fs_khz = fs_khz;
            (*ps_dec).frame_length = frame_length;
        }

        ret
    }
}
