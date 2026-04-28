//! Translated from `c/silk/code_signs.c` (RFC 6716).
//!
//! Decodes signs of excitation pulses from the range coder.

use crate::entcode::ec_dec;
use crate::entdec::ec_dec_icdf;

use super::tables_pulses_per_block::silk_sign_iCDF;

const SHELL_CODEC_FRAME_LENGTH: i32 = 16;
const LOG2_SHELL_CODEC_FRAME_LENGTH: i32 = 4;

/// `silk_decode_signs` — decode signs of excitation pulses.
pub unsafe fn silk_decode_signs(
    ps_range_dec: *mut ec_dec,
    pulses: *mut i32,
    length: i32,
    signal_type: i32,
    quant_offset_type: i32,
    sum_pulses: *const i32,
) {
    unsafe {
        let mut icdf: [u8; 2] = [0; 2];
        icdf[1] = 0;
        let mut q_ptr = pulses;
        let i = 7 * (quant_offset_type + (signal_type << 1));
        let icdf_ptr = silk_sign_iCDF.as_ptr().offset(i as isize);
        let length = (length + SHELL_CODEC_FRAME_LENGTH / 2) >> LOG2_SHELL_CODEC_FRAME_LENGTH;
        let mut i = 0;
        while i < length {
            let p = *sum_pulses.offset(i as isize);
            if p > 0 {
                icdf[0] = *icdf_ptr.offset(core::cmp::min(p & 0x1F, 6) as isize);
                let mut j = 0;
                while j < SHELL_CODEC_FRAME_LENGTH {
                    if *q_ptr.offset(j as isize) > 0 {
                        /* implementation with shift, subtraction, multiplication */
                        let dec_val = ec_dec_icdf(ps_range_dec, icdf.as_ptr(), 8);
                        *q_ptr.offset(j as isize) *= (dec_val << 1) - 1;
                    }
                    j += 1;
                }
            }
            q_ptr = q_ptr.offset(SHELL_CODEC_FRAME_LENGTH as isize);
            i += 1;
        }
    }
}
