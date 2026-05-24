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
/// (written into `children[..2]`) using `shell_table` keyed on
/// `silk_shell_code_table_offsets[p]`.
#[inline]
fn decode_split(children: &mut [i32], ps_range_dec: &mut ec_dec, p: i32, shell_table: &[u8]) {
    if p > 0 {
        let offset = SILK_SHELL_CODE_TABLE_OFFSETS[p as usize] as usize;
        children[0] = ec_dec_icdf(ps_range_dec, &shell_table[offset..], 8);
        children[1] = p - children[0];
    } else {
        children[0] = 0;
        children[1] = 0;
    }
}

/// `silk_shell_decoder` — decode 16 nonnegative pulse amplitudes from a
/// total pulse count `pulses4` (the root of the shell tree).
pub fn silk_shell_decoder(pulses0: &mut [i32], ps_range_dec: &mut ec_dec, pulses4: i32) {
    let mut pulses3 = [0i32; 2];
    let mut pulses2 = [0i32; 4];
    let mut pulses1 = [0i32; 8];

    /* this function operates on one shell code frame of 16 pulses */
    /* silk_assert(SHELL_CODEC_FRAME_LENGTH == 16); */

    decode_split(&mut pulses3, ps_range_dec, pulses4, &SILK_SHELL_CODE_TABLE3);

    decode_split(&mut pulses2[0..2], ps_range_dec, pulses3[0], &SILK_SHELL_CODE_TABLE2);
    decode_split(&mut pulses1[0..2], ps_range_dec, pulses2[0], &SILK_SHELL_CODE_TABLE1);
    decode_split(&mut pulses0[0..2], ps_range_dec, pulses1[0], &SILK_SHELL_CODE_TABLE0);
    decode_split(&mut pulses0[2..4], ps_range_dec, pulses1[1], &SILK_SHELL_CODE_TABLE0);
    decode_split(&mut pulses1[2..4], ps_range_dec, pulses2[1], &SILK_SHELL_CODE_TABLE1);
    decode_split(&mut pulses0[4..6], ps_range_dec, pulses1[2], &SILK_SHELL_CODE_TABLE0);
    decode_split(&mut pulses0[6..8], ps_range_dec, pulses1[3], &SILK_SHELL_CODE_TABLE0);

    decode_split(&mut pulses2[2..4], ps_range_dec, pulses3[1], &SILK_SHELL_CODE_TABLE2);
    decode_split(&mut pulses1[4..6], ps_range_dec, pulses2[2], &SILK_SHELL_CODE_TABLE1);
    decode_split(&mut pulses0[8..10], ps_range_dec, pulses1[4], &SILK_SHELL_CODE_TABLE0);
    decode_split(&mut pulses0[10..12], ps_range_dec, pulses1[5], &SILK_SHELL_CODE_TABLE0);
    decode_split(&mut pulses1[6..8], ps_range_dec, pulses2[3], &SILK_SHELL_CODE_TABLE1);
    decode_split(&mut pulses0[12..14], ps_range_dec, pulses1[6], &SILK_SHELL_CODE_TABLE0);
    decode_split(&mut pulses0[14..16], ps_range_dec, pulses1[7], &SILK_SHELL_CODE_TABLE0);
}
