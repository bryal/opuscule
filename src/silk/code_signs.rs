//! Translated from `c/silk/code_signs.c` (RFC 6716).
//!
//! Decodes signs of excitation pulses from the range coder.

use crate::entcode::ec_dec;
use crate::entdec::ec_dec_icdf;

use super::tables_pulses_per_block::SILK_SIGN_ICDF;

const SHELL_CODEC_FRAME_LENGTH: i32 = 16;
const LOG2_SHELL_CODEC_FRAME_LENGTH: i32 = 4;

/// `silk_decode_signs` — decode signs of excitation pulses.
pub unsafe fn silk_decode_signs(
    ps_range_dec: *mut ec_dec,
    pulses: &mut [i32],
    length: i32,
    signal_type: i32,
    quant_offset_type: i32,
    sum_pulses: &[i32],
) {
    let mut icdf: [u8; 2] = [0; 2];
    let icdf_off = (7 * (quant_offset_type + (signal_type << 1))) as usize;
    let nblocks = ((length + SHELL_CODEC_FRAME_LENGTH / 2) >> LOG2_SHELL_CODEC_FRAME_LENGTH) as usize;
    let frame_len = SHELL_CODEC_FRAME_LENGTH as usize;
    for i in 0..nblocks {
        let p = sum_pulses[i];
        if p > 0 {
            icdf[0] = SILK_SIGN_ICDF[icdf_off + core::cmp::min(p & 0x1F, 6) as usize];
            let block = &mut pulses[i * frame_len..(i + 1) * frame_len];
            for slot in block.iter_mut() {
                if *slot > 0 {
                    /* implementation with shift, subtraction, multiplication */
                    let dec_val = unsafe { ec_dec_icdf(ps_range_dec, icdf.as_ptr(), 8) };
                    *slot *= (dec_val << 1) - 1;
                }
            }
        }
    }
}
