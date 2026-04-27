//! Translated from `c/silk/decode_indices.c` (RFC 6716).
//!
//! Decodes the per-frame side-information carried in the range-coded
//! payload into `SilkDecoderState::indices`: signal type / quant offset,
//! gain indices, NLSF indices + interpolation factor, pitch lags and
//! contour (voiced frames only), LTP gain indices, LTP scaling, and
//! the unvoiced-excitation seed.

use crate::entcode::ec_dec;
use crate::entdec::ec_dec_icdf;

use super::macros::silk_lshift;
use super::nlsf_unpack::silk_NLSF_unpack;
use super::structs::{MAX_LPC_ORDER, MAX_NB_SUBFR, SilkDecoderState};
use super::tables_gain::{silk_delta_gain_iCDF, silk_gain_iCDF};
use super::tables_ltp::{silk_LTP_gain_iCDF_ptrs, silk_LTP_per_index_iCDF};
use super::tables_other::{
    silk_LTPscale_iCDF, silk_NLSF_EXT_iCDF, silk_NLSF_interpolation_factor_iCDF, silk_type_offset_VAD_iCDF,
    silk_type_offset_no_VAD_iCDF, silk_uniform4_iCDF, silk_uniform8_iCDF,
};
use super::tables_pitch_lag::{silk_pitch_delta_iCDF, silk_pitch_lag_iCDF};

const CODE_INDEPENDENTLY: i32 = 0;
const CODE_CONDITIONALLY: i32 = 2;
const TYPE_VOICED: i32 = 2;
const NLSF_QUANT_MAX_AMPLITUDE: i32 = 4;

/// `silk_decode_indices` — decode side-information parameters from payload.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_decode_indices(
    ps_dec: *mut SilkDecoderState,
    ps_range_dec: *mut ec_dec,
    frame_index: i32,
    decode_lbrr: i32,
    cond_coding: i32,
) {
    unsafe {
        let mut ec_ix = [0i16; MAX_LPC_ORDER];
        let mut pred_q8 = [0u8; MAX_LPC_ORDER];

        /*******************************************/
        /* Decode signal type and quantizer offset */
        /*******************************************/
        let ix = if decode_lbrr != 0 || (*ps_dec).vad_flags[frame_index as usize] != 0 {
            ec_dec_icdf(ps_range_dec, silk_type_offset_VAD_iCDF.as_ptr(), 8) + 2
        } else {
            ec_dec_icdf(ps_range_dec, silk_type_offset_no_VAD_iCDF.as_ptr(), 8)
        };
        (*ps_dec).indices.signal_type = (ix >> 1) as i8;
        (*ps_dec).indices.quant_offset_type = (ix & 1) as i8;

        /****************/
        /* Decode gains */
        /****************/
        /* First subframe */
        if cond_coding == CODE_CONDITIONALLY {
            /* Conditional coding */
            (*ps_dec).indices.gains_indices[0] = ec_dec_icdf(ps_range_dec, silk_delta_gain_iCDF.as_ptr(), 8) as i8;
        } else {
            /* Independent coding, in two stages: MSB bits followed by 3 LSBs */
            (*ps_dec).indices.gains_indices[0] =
                silk_lshift(ec_dec_icdf(ps_range_dec, silk_gain_iCDF[(*ps_dec).indices.signal_type as usize].as_ptr(), 8), 3)
                    as i8;
            (*ps_dec).indices.gains_indices[0] =
                ((*ps_dec).indices.gains_indices[0] as i32 + ec_dec_icdf(ps_range_dec, silk_uniform8_iCDF.as_ptr(), 8)) as i8;
        }

        /* Remaining subframes */
        let mut i = 1;
        while i < (*ps_dec).nb_subfr {
            (*ps_dec).indices.gains_indices[i as usize] = ec_dec_icdf(ps_range_dec, silk_delta_gain_iCDF.as_ptr(), 8) as i8;
            i += 1;
        }

        /**********************/
        /* Decode LSF Indices */
        /**********************/
        let nlsf_cb = (*ps_dec).ps_nlsf_cb;
        (*ps_dec).indices.nlsf_indices[0] = ec_dec_icdf(
            ps_range_dec,
            (*nlsf_cb).cb1_icdf.offset(((((*ps_dec).indices.signal_type as i32) >> 1) * (*nlsf_cb).n_vectors as i32) as isize),
            8,
        ) as i8;
        silk_NLSF_unpack(ec_ix.as_mut_ptr(), pred_q8.as_mut_ptr(), nlsf_cb, (*ps_dec).indices.nlsf_indices[0] as i32);
        /* silk_assert(psDec->psNLSF_CB->order == psDec->LPC_order); */
        let mut i = 0i32;
        while i < (*nlsf_cb).order as i32 {
            let mut ix = ec_dec_icdf(ps_range_dec, (*nlsf_cb).ec_icdf.offset(ec_ix[i as usize] as isize), 8);
            if ix == 0 {
                ix -= ec_dec_icdf(ps_range_dec, silk_NLSF_EXT_iCDF.as_ptr(), 8);
            } else if ix == 2 * NLSF_QUANT_MAX_AMPLITUDE {
                ix += ec_dec_icdf(ps_range_dec, silk_NLSF_EXT_iCDF.as_ptr(), 8);
            }
            (*ps_dec).indices.nlsf_indices[(i + 1) as usize] = (ix - NLSF_QUANT_MAX_AMPLITUDE) as i8;
            i += 1;
        }

        /* Decode LSF interpolation factor */
        if (*ps_dec).nb_subfr == MAX_NB_SUBFR as i32 {
            (*ps_dec).indices.nlsf_interp_coef_q2 =
                ec_dec_icdf(ps_range_dec, silk_NLSF_interpolation_factor_iCDF.as_ptr(), 8) as i8;
        } else {
            (*ps_dec).indices.nlsf_interp_coef_q2 = 4;
        }

        if (*ps_dec).indices.signal_type as i32 == TYPE_VOICED {
            /*********************/
            /* Decode pitch lags */
            /*********************/
            /* Get lag index */
            let mut decode_absolute_lag_index = 1;
            if cond_coding == CODE_CONDITIONALLY && (*ps_dec).ec_prev_signal_type == TYPE_VOICED {
                /* Decode Delta index */
                let mut delta_lag_index = ec_dec_icdf(ps_range_dec, silk_pitch_delta_iCDF.as_ptr(), 8) as i16;
                if delta_lag_index > 0 {
                    delta_lag_index -= 9;
                    (*ps_dec).indices.lag_index = (*ps_dec).ec_prev_lag_index + delta_lag_index;
                    decode_absolute_lag_index = 0;
                }
            }
            if decode_absolute_lag_index != 0 {
                /* Absolute decoding */
                (*ps_dec).indices.lag_index = (ec_dec_icdf(ps_range_dec, silk_pitch_lag_iCDF.as_ptr(), 8) as i16)
                    * ((*ps_dec).fs_khz >> 1) as i16;
                (*ps_dec).indices.lag_index += ec_dec_icdf(ps_range_dec, (*ps_dec).pitch_lag_low_bits_icdf, 8) as i16;
            }
            (*ps_dec).ec_prev_lag_index = (*ps_dec).indices.lag_index;

            /* Get countour index */
            (*ps_dec).indices.contour_index = ec_dec_icdf(ps_range_dec, (*ps_dec).pitch_contour_icdf, 8) as i8;

            /********************/
            /* Decode LTP gains */
            /********************/
            /* Decode PERIndex value */
            (*ps_dec).indices.per_index = ec_dec_icdf(ps_range_dec, silk_LTP_per_index_iCDF.as_ptr(), 8) as i8;

            let mut k = 0;
            while k < (*ps_dec).nb_subfr {
                (*ps_dec).indices.ltp_index[k as usize] =
                    ec_dec_icdf(ps_range_dec, silk_LTP_gain_iCDF_ptrs[(*ps_dec).indices.per_index as usize] as *const u8, 8)
                        as i8;
                k += 1;
            }

            /**********************/
            /* Decode LTP scaling */
            /**********************/
            if cond_coding == CODE_INDEPENDENTLY {
                (*ps_dec).indices.ltp_scale_index = ec_dec_icdf(ps_range_dec, silk_LTPscale_iCDF.as_ptr(), 8) as i8;
            } else {
                (*ps_dec).indices.ltp_scale_index = 0;
            }
        }
        (*ps_dec).ec_prev_signal_type = (*ps_dec).indices.signal_type as i32;

        /***************/
        /* Decode seed */
        /***************/
        (*ps_dec).indices.seed = ec_dec_icdf(ps_range_dec, silk_uniform4_iCDF.as_ptr(), 8) as i8;
    }
}
