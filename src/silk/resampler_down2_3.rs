//! Translated from `c/silk/resampler_down2_3.c` (RFC 6716).
//!
//! Low-quality 2/3 fractional downsampler used by some narrow-band paths.

use super::macros::{silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb};
use super::resampler_private_AR2::silk_resampler_private_ar2;
use super::resampler_rom::SILK_RESAMPLER_2_3_COEFS_LQ;

const ORDER_FIR: usize = 4;

/// `RESAMPLER_MAX_BATCH_SIZE_MS * RESAMPLER_MAX_FS_KHZ` from
/// `c/silk/resampler_private.h`.
const RESAMPLER_MAX_BATCH_SIZE_IN: usize = 10 * 48;

/// `silk_resampler_down2_3` — downsample by a factor 2/3, low quality.
///
/// `s` is a 6-element state vector: 4 FIR history slots followed by the
/// 2-element AR2 state.
pub fn silk_resampler_down2_3(s: &mut [i32], out: &mut [i16], in_: &[i16], mut in_len: i32) {
    let mut buf = [0i32; RESAMPLER_MAX_BATCH_SIZE_IN + ORDER_FIR];

    /* Copy buffered samples to start of buffer */
    buf[..ORDER_FIR].copy_from_slice(&s[..ORDER_FIR]);

    let mut in_off = 0usize;
    let mut out_off = 0usize;
    let mut n_samples_in;

    /* Iterate over blocks of frameSizeIn input samples */
    loop {
        n_samples_in = in_len.min(RESAMPLER_MAX_BATCH_SIZE_IN as i32);

        /* Second-order AR filter (output in Q8) */
        silk_resampler_private_ar2(
            &mut s[ORDER_FIR..],
            &mut buf[ORDER_FIR..],
            &in_[in_off..],
            &SILK_RESAMPLER_2_3_COEFS_LQ,
            n_samples_in,
        );

        /* Interpolate filtered signal */
        let mut b = 0usize;
        let mut counter = n_samples_in;
        while counter > 2 {
            /* Inner product */
            let mut res_q6 = silk_smulwb(buf[b], SILK_RESAMPLER_2_3_COEFS_LQ[2] as i32);
            res_q6 = silk_smlawb(res_q6, buf[b + 1], SILK_RESAMPLER_2_3_COEFS_LQ[3] as i32);
            res_q6 = silk_smlawb(res_q6, buf[b + 2], SILK_RESAMPLER_2_3_COEFS_LQ[5] as i32);
            res_q6 = silk_smlawb(res_q6, buf[b + 3], SILK_RESAMPLER_2_3_COEFS_LQ[4] as i32);

            /* Scale down, saturate and store in output array */
            out[out_off] = silk_sat16(silk_rshift_round(res_q6, 6)) as i16;
            out_off += 1;

            let mut res_q6 = silk_smulwb(buf[b + 1], SILK_RESAMPLER_2_3_COEFS_LQ[4] as i32);
            res_q6 = silk_smlawb(res_q6, buf[b + 2], SILK_RESAMPLER_2_3_COEFS_LQ[5] as i32);
            res_q6 = silk_smlawb(res_q6, buf[b + 3], SILK_RESAMPLER_2_3_COEFS_LQ[3] as i32);
            res_q6 = silk_smlawb(res_q6, buf[b + 4], SILK_RESAMPLER_2_3_COEFS_LQ[2] as i32);

            /* Scale down, saturate and store in output array */
            out[out_off] = silk_sat16(silk_rshift_round(res_q6, 6)) as i16;
            out_off += 1;

            b += 3;
            counter -= 3;
        }

        in_off += n_samples_in as usize;
        in_len -= n_samples_in;

        if in_len > 0 {
            /* More iterations to do; copy last part of filtered signal to beginning of buffer */
            buf.copy_within(n_samples_in as usize..n_samples_in as usize + ORDER_FIR, 0);
        } else {
            break;
        }
    }

    /* Copy last part of filtered signal to the state for the next call */
    s[..ORDER_FIR].copy_from_slice(&buf[n_samples_in as usize..n_samples_in as usize + ORDER_FIR]);
}
