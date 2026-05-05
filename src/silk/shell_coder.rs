//! Translated from `c/silk/shell_coder.c` (RFC 6716, decoder side only).
//!
//! Shell decoder: reconstructs the 16 pulse magnitudes of one shell-code
//! frame by recursively splitting the parent pulse count into two child
//! pulse counts using cumulative-distribution tables keyed on the parent
//! count. Pulse-subframe length (16) is hard-coded — matching the C —
//! and the binary-tree shape is unrolled in [`silk_shell_decoder`].

use crate::entcode::ec_dec;
use crate::entdec::ec_dec_icdf;

use super::tables_pulses_per_block::{
    SILK_SHELL_CODE_TABLE_OFFSETS, SILK_SHELL_CODE_TABLE0, SILK_SHELL_CODE_TABLE1, SILK_SHELL_CODE_TABLE2,
    SILK_SHELL_CODE_TABLE3,
};

/// `decode_split` — split a parent pulse count `p` into two children
/// using `shell_table` keyed on `silk_shell_code_table_offsets[p]`.
#[inline]
unsafe fn decode_split(p_child1: *mut i32, p_child2: *mut i32, ps_range_dec: *mut ec_dec, p: i32, shell_table: *const u8) {
    unsafe {
        if p > 0 {
            *p_child1 = ec_dec_icdf(ps_range_dec, shell_table.offset(SILK_SHELL_CODE_TABLE_OFFSETS[p as usize] as isize), 8);
            *p_child2 = p - *p_child1;
        } else {
            *p_child1 = 0;
            *p_child2 = 0;
        }
    }
}

/// `silk_shell_decoder` — decode 16 nonnegative pulse amplitudes from a
/// total pulse count `pulses4` (the root of the shell tree).
pub unsafe fn silk_shell_decoder(pulses0: *mut i32, ps_range_dec: *mut ec_dec, pulses4: i32) {
    unsafe {
        let mut pulses3 = [0i32; 2];
        let mut pulses2 = [0i32; 4];
        let mut pulses1 = [0i32; 8];

        /* this function operates on one shell code frame of 16 pulses */
        /* silk_assert(SHELL_CODEC_FRAME_LENGTH == 16); */

        decode_split(
            pulses3.as_mut_ptr().offset(0),
            pulses3.as_mut_ptr().offset(1),
            ps_range_dec,
            pulses4,
            SILK_SHELL_CODE_TABLE3.as_ptr(),
        );

        decode_split(
            pulses2.as_mut_ptr().offset(0),
            pulses2.as_mut_ptr().offset(1),
            ps_range_dec,
            pulses3[0],
            SILK_SHELL_CODE_TABLE2.as_ptr(),
        );

        decode_split(
            pulses1.as_mut_ptr().offset(0),
            pulses1.as_mut_ptr().offset(1),
            ps_range_dec,
            pulses2[0],
            SILK_SHELL_CODE_TABLE1.as_ptr(),
        );
        decode_split(pulses0.offset(0), pulses0.offset(1), ps_range_dec, pulses1[0], SILK_SHELL_CODE_TABLE0.as_ptr());
        decode_split(pulses0.offset(2), pulses0.offset(3), ps_range_dec, pulses1[1], SILK_SHELL_CODE_TABLE0.as_ptr());

        decode_split(
            pulses1.as_mut_ptr().offset(2),
            pulses1.as_mut_ptr().offset(3),
            ps_range_dec,
            pulses2[1],
            SILK_SHELL_CODE_TABLE1.as_ptr(),
        );
        decode_split(pulses0.offset(4), pulses0.offset(5), ps_range_dec, pulses1[2], SILK_SHELL_CODE_TABLE0.as_ptr());
        decode_split(pulses0.offset(6), pulses0.offset(7), ps_range_dec, pulses1[3], SILK_SHELL_CODE_TABLE0.as_ptr());

        decode_split(
            pulses2.as_mut_ptr().offset(2),
            pulses2.as_mut_ptr().offset(3),
            ps_range_dec,
            pulses3[1],
            SILK_SHELL_CODE_TABLE2.as_ptr(),
        );

        decode_split(
            pulses1.as_mut_ptr().offset(4),
            pulses1.as_mut_ptr().offset(5),
            ps_range_dec,
            pulses2[2],
            SILK_SHELL_CODE_TABLE1.as_ptr(),
        );
        decode_split(pulses0.offset(8), pulses0.offset(9), ps_range_dec, pulses1[4], SILK_SHELL_CODE_TABLE0.as_ptr());
        decode_split(pulses0.offset(10), pulses0.offset(11), ps_range_dec, pulses1[5], SILK_SHELL_CODE_TABLE0.as_ptr());

        decode_split(
            pulses1.as_mut_ptr().offset(6),
            pulses1.as_mut_ptr().offset(7),
            ps_range_dec,
            pulses2[3],
            SILK_SHELL_CODE_TABLE1.as_ptr(),
        );
        decode_split(pulses0.offset(12), pulses0.offset(13), ps_range_dec, pulses1[6], SILK_SHELL_CODE_TABLE0.as_ptr());
        decode_split(pulses0.offset(14), pulses0.offset(15), ps_range_dec, pulses1[7], SILK_SHELL_CODE_TABLE0.as_ptr());
    }
}
