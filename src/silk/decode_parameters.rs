//! Translated from `c/silk/decode_parameters.c` (RFC 6716).
//!
//! Turns the per-frame side-info indices (produced by
//! `silk_decode_indices`) into the dequantized parameters the later
//! decode stages consume: gains, LPC prediction coefs (with optional
//! interpolated first half), pitch lags, LTP coefs, and LTP scale.

use super::NLSF_decode::silk_nlsf_decode;
use super::NLSF2A::silk_nlsf2a;
use super::bwexpander::silk_bwexpander;
use super::decode_pitch::silk_decode_pitch;
use super::gain_quant::silk_gains_dequant;
use super::macros::silk_lshift;
use super::structs::{LTP_ORDER, MAX_LPC_ORDER, SilkDecoderControl, SilkDecoderState};
use super::tables_ltp::SILK_LTP_VQ_PTRS_Q7;
use super::tables_other::SILK_LTP_SCALES_TABLE_Q14;

const CODE_CONDITIONALLY: i32 = 2;
const TYPE_VOICED: i32 = 2;
const BWE_AFTER_LOSS_Q16: i32 = 63570;

/// `silk_decode_parameters` — decode parameters from payload.
pub unsafe fn silk_decode_parameters(ps_dec: *mut SilkDecoderState, ps_dec_ctrl: *mut SilkDecoderControl, cond_coding: i32) {
    unsafe {
        let mut p_nlsf_q15 = [0i16; MAX_LPC_ORDER];
        let mut p_nlsf0_q15 = [0i16; MAX_LPC_ORDER];

        /* Dequant Gains */
        silk_gains_dequant(
            (*ps_dec_ctrl).gains_q16.as_mut_ptr(),
            (*ps_dec).indices.gains_indices.as_ptr(),
            &mut (*ps_dec).last_gain_index,
            (cond_coding == CODE_CONDITIONALLY) as i32,
            (*ps_dec).nb_subfr,
        );

        /****************/
        /* Decode NLSFs */
        /****************/
        silk_nlsf_decode(p_nlsf_q15.as_mut_ptr(), (*ps_dec).indices.nlsf_indices.as_mut_ptr(), (*ps_dec).ps_nlsf_cb);

        /* Convert NLSF parameters to AR prediction filter coefficients */
        silk_nlsf2a((*ps_dec_ctrl).pred_coef_q12[1].as_mut_ptr(), p_nlsf_q15.as_ptr(), (*ps_dec).lpc_order);

        /* If just reset, e.g., because internal Fs changed, do not allow interpolation */
        /* improves the case of packet loss in the first frame after a switch           */
        if (*ps_dec).first_frame_after_reset == 1 {
            (*ps_dec).indices.nlsf_interp_coef_q2 = 4;
        }

        if ((*ps_dec).indices.nlsf_interp_coef_q2 as i32) < 4 {
            /* Calculation of the interpolated NLSF0 vector from the interpolation factor, */
            /* the previous NLSF1, and the current NLSF1                                   */
            let mut i = 0i32;
            while i < (*ps_dec).lpc_order {
                p_nlsf0_q15[i as usize] = (*ps_dec).prev_nlsf_q15[i as usize]
                    + ((*ps_dec).indices.nlsf_interp_coef_q2 as i32
                        * (p_nlsf_q15[i as usize] - (*ps_dec).prev_nlsf_q15[i as usize]) as i32
                        >> 2) as i16;
                i += 1;
            }

            /* Convert NLSF parameters to AR prediction filter coefficients */
            silk_nlsf2a((*ps_dec_ctrl).pred_coef_q12[0].as_mut_ptr(), p_nlsf0_q15.as_ptr(), (*ps_dec).lpc_order);
        } else {
            /* Copy LPC coefficients for first half from second half */
            core::ptr::copy_nonoverlapping(
                (*ps_dec_ctrl).pred_coef_q12[1].as_ptr(),
                (*ps_dec_ctrl).pred_coef_q12[0].as_mut_ptr(),
                (*ps_dec).lpc_order as usize,
            );
        }

        core::ptr::copy_nonoverlapping(p_nlsf_q15.as_ptr(), (*ps_dec).prev_nlsf_q15.as_mut_ptr(), (*ps_dec).lpc_order as usize);

        /* After a packet loss do BWE of LPC coefs */
        if (*ps_dec).loss_cnt != 0 {
            silk_bwexpander((*ps_dec_ctrl).pred_coef_q12[0].as_mut_ptr(), (*ps_dec).lpc_order, BWE_AFTER_LOSS_Q16);
            silk_bwexpander((*ps_dec_ctrl).pred_coef_q12[1].as_mut_ptr(), (*ps_dec).lpc_order, BWE_AFTER_LOSS_Q16);
        }

        if (*ps_dec).indices.signal_type as i32 == TYPE_VOICED {
            /*********************/
            /* Decode pitch lags */
            /*********************/

            /* Decode pitch values */
            silk_decode_pitch(
                (*ps_dec).indices.lag_index,
                (*ps_dec).indices.contour_index,
                (*ps_dec_ctrl).pitch_l.as_mut_ptr(),
                (*ps_dec).fs_khz,
                (*ps_dec).nb_subfr,
            );

            /* Decode Codebook Index */
            let cbk_ptr_q7 = SILK_LTP_VQ_PTRS_Q7[(*ps_dec).indices.per_index as usize] as *const i8;

            let mut k = 0i32;
            while k < (*ps_dec).nb_subfr {
                let ix = (*ps_dec).indices.ltp_index[k as usize] as i32;
                let mut i = 0i32;
                while i < LTP_ORDER as i32 {
                    (*ps_dec_ctrl).ltp_coef_q14[(k * LTP_ORDER as i32 + i) as usize] =
                        silk_lshift(*cbk_ptr_q7.offset((ix * LTP_ORDER as i32 + i) as isize) as i32, 7) as i16;
                    i += 1;
                }
                k += 1;
            }

            /**********************/
            /* Decode LTP scaling */
            /**********************/
            let ix = (*ps_dec).indices.ltp_scale_index as usize;
            (*ps_dec_ctrl).ltp_scale_q14 = SILK_LTP_SCALES_TABLE_Q14[ix] as i32;
        } else {
            core::ptr::write_bytes((*ps_dec_ctrl).pitch_l.as_mut_ptr(), 0, (*ps_dec).nb_subfr as usize);
            core::ptr::write_bytes(
                (*ps_dec_ctrl).ltp_coef_q14.as_mut_ptr(),
                0,
                (LTP_ORDER as i32 * (*ps_dec).nb_subfr) as usize,
            );
            (*ps_dec).indices.per_index = 0;
            (*ps_dec_ctrl).ltp_scale_q14 = 0;
        }
    }
}
