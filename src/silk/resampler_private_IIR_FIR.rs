//! Translated from `c/silk/resampler_private_IIR_FIR.c` (RFC 6716).
//!
//! Upsampler that combines allpass-based 2× upsampling (via
//! [`silk_resampler_private_up2_HQ`]) with 12-fraction FIR interpolation
//! to reach the target output rate. The FIR state lives in the host
//! resampler struct's `sFIR` field, which is interpreted as a stream of
//! `i16`s for this path (the C uses `opus_int32 sFIR[36]` but copies
//! `RESAMPLER_ORDER_FIR_12 * sizeof(opus_int32) = 32` bytes into an
//! `opus_int16 buf[]`, treating those 32 bytes as 16 `i16`s — preserved
//! here as an explicit low-half/high-half split, matching the C's
//! little-endian byte layout).

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlabb, silk_smulbb, silk_smulwb};
use super::resampler_private_up2_HQ::silk_resampler_private_up2_hq;
use super::resampler_rom::{RESAMPLER_ORDER_FIR_12, SILK_RESAMPLER_FRAC_FIR_12};
use super::structs::SilkResamplerStateStruct;

/// `RESAMPLER_MAX_BATCH_SIZE_MS * RESAMPLER_MAX_FS_KHZ` from
/// `c/silk/resampler_private.h`.
const RESAMPLER_MAX_BATCH_SIZE_IN: usize = 10 * 48;

/// Inner interpolation loop — returns the number of output samples written.
#[inline]
fn silk_resampler_private_iir_fir_interpol(
    out: &mut [i16],
    buf: &[i16],
    max_index_q16: i32,
    index_increment_q16: i32,
) -> usize {
    /* Interpolate upsampled signal and store in output array */
    let mut n = 0usize;
    let mut index_q16: i32 = 0;
    while index_q16 < max_index_q16 {
        let table_index = silk_smulwb(index_q16 & 0xFFFF, 12);
        let b = (index_q16 >> 16) as usize;

        let tbl = SILK_RESAMPLER_FRAC_FIR_12[table_index as usize];
        let tbl_rev = SILK_RESAMPLER_FRAC_FIR_12[(11 - table_index) as usize];
        let mut res_q15 = silk_smulbb(buf[b] as i32, tbl[0] as i32);
        res_q15 = silk_smlabb(res_q15, buf[b + 1] as i32, tbl[1] as i32);
        res_q15 = silk_smlabb(res_q15, buf[b + 2] as i32, tbl[2] as i32);
        res_q15 = silk_smlabb(res_q15, buf[b + 3] as i32, tbl[3] as i32);
        res_q15 = silk_smlabb(res_q15, buf[b + 4] as i32, tbl_rev[3] as i32);
        res_q15 = silk_smlabb(res_q15, buf[b + 5] as i32, tbl_rev[2] as i32);
        res_q15 = silk_smlabb(res_q15, buf[b + 6] as i32, tbl_rev[1] as i32);
        res_q15 = silk_smlabb(res_q15, buf[b + 7] as i32, tbl_rev[0] as i32);
        out[n] = silk_sat16(silk_rshift_round(res_q15, 15)) as i16;
        n += 1;

        index_q16 += index_increment_q16;
    }
    n
}

/// `silk_resampler_private_IIR_FIR` — 2× IIR upsample + fractional FIR.
pub fn silk_resampler_private_iir_fir(s: &mut SilkResamplerStateStruct, out: &mut [i16], in_: &[i16], mut in_len: i32) {
    let mut buf = [0i16; RESAMPLER_MAX_BATCH_SIZE_IN + RESAMPLER_ORDER_FIR_12];

    /* Copy buffered samples to start of buffer */
    /* C: silk_memcpy(buf, S->sFIR, RESAMPLER_ORDER_FIR_12 * sizeof(opus_int32))
     * — copies 32 bytes (RESAMPLER_ORDER_FIR_12=8 × sizeof(int32)=4) into an
     * int16[] buffer, i.e. 16 int16s taken from the first 8 int32 slots of
     * sFIR. Replicated as an explicit little-endian half-word split; the
     * state is only ever written back by this same function (below), so the
     * mapping just has to be self-consistent. */
    for i in 0..RESAMPLER_ORDER_FIR_12 {
        let w = s.s_fir[i];
        buf[2 * i] = w as i16;
        buf[2 * i + 1] = (w >> 16) as i16;
    }

    /* Iterate over blocks of frameSizeIn input samples */
    let index_increment_q16 = s.inv_ratio_q16;
    let mut out_off = 0usize;
    let mut in_off = 0usize;
    let mut n_samples_in;
    loop {
        n_samples_in = in_len.min(s.batch_size);

        /* Upsample 2x */
        silk_resampler_private_up2_hq(&mut s.s_iir, &mut buf[RESAMPLER_ORDER_FIR_12..], &in_[in_off..], n_samples_in);

        let max_index_q16 = silk_lshift(n_samples_in, 16 + 1); /* + 1 because 2x upsampling */
        out_off += silk_resampler_private_iir_fir_interpol(&mut out[out_off..], &buf, max_index_q16, index_increment_q16);
        in_off += n_samples_in as usize;
        in_len -= n_samples_in;

        if in_len > 0 {
            /* More iterations to do; copy last part of filtered signal to beginning of buffer */
            let tail = (n_samples_in << 1) as usize;
            buf.copy_within(tail..tail + 2 * RESAMPLER_ORDER_FIR_12, 0);
        } else {
            break;
        }
    }

    /* Copy last part of filtered signal to the state for the next call */
    let tail = (n_samples_in << 1) as usize;
    for i in 0..RESAMPLER_ORDER_FIR_12 {
        let lo = buf[tail + 2 * i] as u16 as i32;
        let hi = buf[tail + 2 * i + 1] as i32;
        s.s_fir[i] = (hi << 16) | lo;
    }
}
