//! Translated from `c/silk/decode_pulses.c` (RFC 6716).
//!
//! Decodes quantization indices of the excitation signal: rate-level,
//! per-shell-block pulse counts (including LSB-indicator overflow),
//! shell decoding of pulse magnitudes, LSB decoding, and finally signs.

use crate::entcode::ec_dec;
use crate::entdec::ec_dec_icdf;

use super::code_signs::silk_decode_signs;
use super::macros::silk_lshift;
use super::shell_coder::silk_shell_decoder;
use super::tables_other::SILK_LSB_ICDF;
use super::tables_pulses_per_block::{SILK_PULSES_PER_BLOCK_ICDF, SILK_RATE_LEVELS_ICDF};

const SHELL_CODEC_FRAME_LENGTH: i32 = 16;
const LOG2_SHELL_CODEC_FRAME_LENGTH: i32 = 4;
const MAX_FRAME_LENGTH: usize = 5 * 4 * 16;
const MAX_NB_SHELL_BLOCKS: usize = MAX_FRAME_LENGTH / SHELL_CODEC_FRAME_LENGTH as usize;
const MAX_PULSES: i32 = 16;
const N_RATE_LEVELS: usize = 10;

/// `silk_decode_pulses` — decode quantization indices of the excitation signal.
pub unsafe fn silk_decode_pulses(
    ps_range_dec: &mut ec_dec,
    pulses: &mut [i32],
    signal_type: i32,
    quant_offset_type: i32,
    frame_length: i32,
) {
    let mut sum_pulses = [0i32; MAX_NB_SHELL_BLOCKS];
    let mut n_lshifts = [0i32; MAX_NB_SHELL_BLOCKS];

    /*********************/
    /* Decode rate level */
    /*********************/
    let rate_level_index = unsafe { ec_dec_icdf(ps_range_dec, SILK_RATE_LEVELS_ICDF[(signal_type >> 1) as usize].as_ptr(), 8) };

    /* Calculate number of shell blocks */
    /* silk_assert(1 << LOG2_SHELL_CODEC_FRAME_LENGTH == SHELL_CODEC_FRAME_LENGTH); */
    let mut iter = frame_length >> LOG2_SHELL_CODEC_FRAME_LENGTH;
    if iter * SHELL_CODEC_FRAME_LENGTH < frame_length {
        /* silk_assert(frame_length == 12 * 10); Make sure only happens for 10 ms @ 12 kHz */
        iter += 1;
    }

    /***************************************************/
    /* Sum-Weighted-Pulses Decoding                    */
    /***************************************************/
    let cdf_ptr = SILK_PULSES_PER_BLOCK_ICDF[rate_level_index as usize].as_ptr();
    for i in 0..iter as usize {
        n_lshifts[i] = 0;
        sum_pulses[i] = unsafe { ec_dec_icdf(ps_range_dec, cdf_ptr, 8) };

        /* LSB indication */
        while sum_pulses[i] == MAX_PULSES + 1 {
            n_lshifts[i] += 1;
            /* When we've already got 10 LSBs, we shift the table to not allow (MAX_PULSES + 1) */
            sum_pulses[i] = unsafe {
                ec_dec_icdf(
                    ps_range_dec,
                    SILK_PULSES_PER_BLOCK_ICDF[N_RATE_LEVELS - 1].as_ptr().offset((n_lshifts[i] == 10) as isize),
                    8,
                )
            };
        }
    }

    /***************************************************/
    /* Shell decoding                                  */
    /***************************************************/
    let frame_len = SHELL_CODEC_FRAME_LENGTH as usize;
    for i in 0..iter as usize {
        let block = &mut pulses[i * frame_len..(i + 1) * frame_len];
        if sum_pulses[i] > 0 {
            unsafe { silk_shell_decoder(block, ps_range_dec, sum_pulses[i]) };
        } else {
            block.fill(0);
        }
    }

    /***************************************************/
    /* LSB Decoding                                    */
    /***************************************************/
    for i in 0..iter as usize {
        if n_lshifts[i] > 0 {
            let n_ls = n_lshifts[i];
            let block = &mut pulses[i * frame_len..(i + 1) * frame_len];
            for slot in block.iter_mut() {
                for _ in 0..n_ls {
                    *slot = silk_lshift(*slot, 1);
                    *slot += unsafe { ec_dec_icdf(ps_range_dec, SILK_LSB_ICDF.as_ptr(), 8) };
                }
            }
            /* Mark the number of pulses non-zero for sign decoding. */
            sum_pulses[i] |= n_ls << 5;
        }
    }

    /****************************************/
    /* Decode and add signs to pulse signal */
    /****************************************/
    unsafe { silk_decode_signs(ps_range_dec, pulses, frame_length, signal_type, quant_offset_type, &sum_pulses) };
}
