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
use super::nlsf_unpack::silk_nlsf_unpack;
use super::structs::{MAX_LPC_ORDER, MAX_NB_SUBFR, SilkDecoderState};
use super::tables_gain::{SILK_DELTA_GAIN_ICDF, SILK_GAIN_ICDF};
use super::tables_ltp::{SILK_LTP_GAIN_ICDF_PTRS, SILK_LTP_PER_INDEX_ICDF};
use super::tables_other::{
    SILK_LTP_SCALE_ICDF, SILK_NLSF_EXT_ICDF, SILK_NLSF_INTERPOLATION_FACTOR_ICDF, SILK_TYPE_OFFSET_NO_VAD_ICDF,
    SILK_TYPE_OFFSET_VAD_ICDF, SILK_UNIFORM4_ICDF, SILK_UNIFORM8_ICDF,
};
use super::tables_pitch_lag::{SILK_PITCH_DELTA_ICDF, SILK_PITCH_LAG_ICDF};

const CODE_INDEPENDENTLY: i32 = 0;
const CODE_CONDITIONALLY: i32 = 2;
const TYPE_VOICED: i32 = 2;
const NLSF_QUANT_MAX_AMPLITUDE: i32 = 4;

/// `silk_decode_indices` — decode side-information parameters from payload.
pub unsafe fn silk_decode_indices(
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
            ec_dec_icdf(ps_range_dec, SILK_TYPE_OFFSET_VAD_ICDF.as_ptr(), 8) + 2
        } else {
            ec_dec_icdf(ps_range_dec, SILK_TYPE_OFFSET_NO_VAD_ICDF.as_ptr(), 8)
        };
        (*ps_dec).indices.signal_type = (ix >> 1) as i8;
        (*ps_dec).indices.quant_offset_type = (ix & 1) as i8;

        /****************/
        /* Decode gains */
        /****************/
        /* First subframe */
        if cond_coding == CODE_CONDITIONALLY {
            /* Conditional coding */
            (*ps_dec).indices.gains_indices[0] = ec_dec_icdf(ps_range_dec, SILK_DELTA_GAIN_ICDF.as_ptr(), 8) as i8;
        } else {
            /* Independent coding, in two stages: MSB bits followed by 3 LSBs */
            (*ps_dec).indices.gains_indices[0] =
                silk_lshift(ec_dec_icdf(ps_range_dec, SILK_GAIN_ICDF[(*ps_dec).indices.signal_type as usize].as_ptr(), 8), 3)
                    as i8;
            (*ps_dec).indices.gains_indices[0] =
                ((*ps_dec).indices.gains_indices[0] as i32 + ec_dec_icdf(ps_range_dec, SILK_UNIFORM8_ICDF.as_ptr(), 8)) as i8;
        }

        /* Remaining subframes */
        let mut i = 1;
        while i < (*ps_dec).nb_subfr {
            (*ps_dec).indices.gains_indices[i as usize] = ec_dec_icdf(ps_range_dec, SILK_DELTA_GAIN_ICDF.as_ptr(), 8) as i8;
            i += 1;
        }

        /**********************/
        /* Decode LSF Indices */
        /**********************/
        let nlsf_cb = (*ps_dec).ps_nlsf_cb;
        (*ps_dec).indices.nlsf_indices[0] = ec_dec_icdf(
            ps_range_dec,
            (*nlsf_cb).cb1_icdf.as_ptr().add(((((*ps_dec).indices.signal_type as i32) >> 1) * (*nlsf_cb).n_vectors as i32) as usize),
            8,
        ) as i8;
        silk_nlsf_unpack(ec_ix.as_mut_ptr(), pred_q8.as_mut_ptr(), nlsf_cb, (*ps_dec).indices.nlsf_indices[0] as i32);
        /* silk_assert(psDec->psNLSF_CB->order == psDec->LPC_order); */
        let mut i = 0i32;
        while i < (*nlsf_cb).order as i32 {
            let mut ix = ec_dec_icdf(ps_range_dec, (*nlsf_cb).ec_icdf.as_ptr().add(ec_ix[i as usize] as usize), 8);
            if ix == 0 {
                ix -= ec_dec_icdf(ps_range_dec, SILK_NLSF_EXT_ICDF.as_ptr(), 8);
            } else if ix == 2 * NLSF_QUANT_MAX_AMPLITUDE {
                ix += ec_dec_icdf(ps_range_dec, SILK_NLSF_EXT_ICDF.as_ptr(), 8);
            }
            (*ps_dec).indices.nlsf_indices[(i + 1) as usize] = (ix - NLSF_QUANT_MAX_AMPLITUDE) as i8;
            i += 1;
        }

        /* Decode LSF interpolation factor */
        if (*ps_dec).nb_subfr == MAX_NB_SUBFR as i32 {
            (*ps_dec).indices.nlsf_interp_coef_q2 =
                ec_dec_icdf(ps_range_dec, SILK_NLSF_INTERPOLATION_FACTOR_ICDF.as_ptr(), 8) as i8;
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
                let mut delta_lag_index = ec_dec_icdf(ps_range_dec, SILK_PITCH_DELTA_ICDF.as_ptr(), 8) as i16;
                if delta_lag_index > 0 {
                    delta_lag_index -= 9;
                    (*ps_dec).indices.lag_index = (*ps_dec).ec_prev_lag_index + delta_lag_index;
                    decode_absolute_lag_index = 0;
                }
            }
            if decode_absolute_lag_index != 0 {
                /* Absolute decoding */
                (*ps_dec).indices.lag_index =
                    (ec_dec_icdf(ps_range_dec, SILK_PITCH_LAG_ICDF.as_ptr(), 8) as i16) * ((*ps_dec).fs_khz >> 1) as i16;
                (*ps_dec).indices.lag_index += ec_dec_icdf(ps_range_dec, (*ps_dec).pitch_lag_low_bits_icdf, 8) as i16;
            }
            (*ps_dec).ec_prev_lag_index = (*ps_dec).indices.lag_index;

            /* Get countour index */
            (*ps_dec).indices.contour_index = ec_dec_icdf(ps_range_dec, (*ps_dec).pitch_contour_icdf, 8) as i8;

            /********************/
            /* Decode LTP gains */
            /********************/
            /* Decode PERIndex value */
            (*ps_dec).indices.per_index = ec_dec_icdf(ps_range_dec, SILK_LTP_PER_INDEX_ICDF.as_ptr(), 8) as i8;

            let mut k = 0;
            while k < (*ps_dec).nb_subfr {
                (*ps_dec).indices.ltp_index[k as usize] =
                    ec_dec_icdf(ps_range_dec, SILK_LTP_GAIN_ICDF_PTRS[(*ps_dec).indices.per_index as usize] as *const u8, 8)
                        as i8;
                k += 1;
            }

            /**********************/
            /* Decode LTP scaling */
            /**********************/
            if cond_coding == CODE_INDEPENDENTLY {
                (*ps_dec).indices.ltp_scale_index = ec_dec_icdf(ps_range_dec, SILK_LTP_SCALE_ICDF.as_ptr(), 8) as i8;
            } else {
                (*ps_dec).indices.ltp_scale_index = 0;
            }
        }
        (*ps_dec).ec_prev_signal_type = (*ps_dec).indices.signal_type as i32;

        /***************/
        /* Decode seed */
        /***************/
        (*ps_dec).indices.seed = ec_dec_icdf(ps_range_dec, SILK_UNIFORM4_ICDF.as_ptr(), 8) as i8;
    }
}
