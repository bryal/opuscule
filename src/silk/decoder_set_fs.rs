//! Translated from `c/silk/decoder_set_fs.c` (RFC 6716).
//!
//! Updates subframe/frame length, LPC order, NLSF codebook pointer and
//! pitch / range-decoder-low-bits iCDF pointers when the SILK internal
//! sampling rate changes, and (re)initialises the resampler if the
//! API-side sample rate or internal rate moves.

use super::structs::{MAX_LPC_ORDER, MAX_NB_SUBFR, SilkDecoderState, SilkResamplerStateStruct};
use super::tables_nlsf_cb_nb_mb::silk_NLSF_CB_NB_MB;
use super::tables_nlsf_cb_wb::silk_NLSF_CB_WB;
use super::tables_other::{silk_uniform4_iCDF, silk_uniform6_iCDF, silk_uniform8_iCDF};
use super::tables_pitch_lag::{
    silk_pitch_contour_10_ms_NB_iCDF, silk_pitch_contour_10_ms_iCDF, silk_pitch_contour_NB_iCDF, silk_pitch_contour_iCDF,
};

const SUB_FRAME_LENGTH_MS: i32 = 5;
const LTP_MEM_LENGTH_MS: i32 = 20;
const MIN_LPC_ORDER: i32 = 10;
const TYPE_NO_VOICE_ACTIVITY: i32 = 0;

unsafe extern "C" {
    fn silk_resampler_init(s: *mut SilkResamplerStateStruct, fs_hz_in: i32, fs_hz_out: i32, forEnc: i32) -> i32;
}

/// `silk_decoder_set_fs` — set SILK internal / API sample rates.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_decoder_set_fs(ps_dec: *mut SilkDecoderState, fs_khz: i32, fs_api_hz: i32) -> i32 {
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
                    (*ps_dec).pitch_contour_icdf = silk_pitch_contour_NB_iCDF.as_ptr();
                } else {
                    (*ps_dec).pitch_contour_icdf = silk_pitch_contour_10_ms_NB_iCDF.as_ptr();
                }
            } else {
                if (*ps_dec).nb_subfr == MAX_NB_SUBFR as i32 {
                    (*ps_dec).pitch_contour_icdf = silk_pitch_contour_iCDF.as_ptr();
                } else {
                    (*ps_dec).pitch_contour_icdf = silk_pitch_contour_10_ms_iCDF.as_ptr();
                }
            }
            if (*ps_dec).fs_khz != fs_khz {
                (*ps_dec).ltp_mem_length = LTP_MEM_LENGTH_MS * fs_khz;
                if fs_khz == 8 || fs_khz == 12 {
                    (*ps_dec).lpc_order = MIN_LPC_ORDER;
                    (*ps_dec).ps_nlsf_cb = &silk_NLSF_CB_NB_MB;
                } else {
                    (*ps_dec).lpc_order = MAX_LPC_ORDER as i32;
                    (*ps_dec).ps_nlsf_cb = &silk_NLSF_CB_WB;
                }
                if fs_khz == 16 {
                    (*ps_dec).pitch_lag_low_bits_icdf = silk_uniform8_iCDF.as_ptr();
                } else if fs_khz == 12 {
                    (*ps_dec).pitch_lag_low_bits_icdf = silk_uniform6_iCDF.as_ptr();
                } else if fs_khz == 8 {
                    (*ps_dec).pitch_lag_low_bits_icdf = silk_uniform4_iCDF.as_ptr();
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
