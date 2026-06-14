//! Translated from `c/silk/resampler_private_down_FIR.c` (RFC 6716).
//!
//! Resample-by-rational via a 2nd-order AR section followed by an FIR
//! interpolator. Three FIR orders are hard-coded (18, 24, 36) and
//! dispatched on `FIR_Order` to let the compiler unroll the inner
//! product.

#![allow(clippy::indexing_slicing)] // dense SILK kernels; voice path is deprioritized vs CELT

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb};
use super::resampler_private_ar2::silk_resampler_private_ar2;
use super::resampler_rom::{RESAMPLER_DOWN_ORDER_FIR0, RESAMPLER_DOWN_ORDER_FIR1, RESAMPLER_DOWN_ORDER_FIR2};
use super::structs::{SILK_RESAMPLER_MAX_FIR_ORDER, SilkResamplerStateStruct};
use crate::util::OrPanic;

const RESAMPLER_MAX_BATCH_SIZE_IN: usize = 10 * 48;

/// Inner interpolation loop — switches on the FIR order to let the
/// compiler unroll the inner dot product. Returns the number of output
/// samples written.
#[inline]
fn silk_resampler_private_down_fir_interpol(
    out: &mut [i16],
    buf: &[i32],
    fir_coefs: &[i16],
    fir_order: i32,
    fir_fracs: i32,
    max_index_q16: i32,
    index_increment_q16: i32,
) -> usize {
    let mut n = 0usize;
    match fir_order as usize {
        RESAMPLER_DOWN_ORDER_FIR0 => {
            let mut index_q16 = 0i32;
            while index_q16 < max_index_q16 {
                /* Integer part gives pointer to buffered input */
                let b = (index_q16 >> 16) as usize;

                /* Fractional part gives interpolation coefficients */
                let interpol_ind = silk_smulwb(index_q16 & 0xFFFF, fir_fracs);

                /* Inner product */
                let ip = &fir_coefs[(RESAMPLER_DOWN_ORDER_FIR0 as i32 / 2 * interpol_ind) as usize..];
                let mut res_q6 = silk_smulwb(buf[b], ip[0] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 1], ip[1] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 2], ip[2] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 3], ip[3] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 4], ip[4] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 5], ip[5] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 6], ip[6] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 7], ip[7] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 8], ip[8] as i32);
                let ip = &fir_coefs[(RESAMPLER_DOWN_ORDER_FIR0 as i32 / 2 * (fir_fracs - 1 - interpol_ind)) as usize..];
                res_q6 = silk_smlawb(res_q6, buf[b + 17], ip[0] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 16], ip[1] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 15], ip[2] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 14], ip[3] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 13], ip[4] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 12], ip[5] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 11], ip[6] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 10], ip[7] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 9], ip[8] as i32);

                /* Scale down, saturate and store in output array */
                out[n] = silk_sat16(silk_rshift_round(res_q6, 6)) as i16;
                n += 1;

                index_q16 += index_increment_q16;
            }
        }
        RESAMPLER_DOWN_ORDER_FIR1 => {
            let mut index_q16 = 0i32;
            while index_q16 < max_index_q16 {
                let b = (index_q16 >> 16) as usize;

                /* Inner product */
                let mut res_q6 = silk_smulwb(buf[b] + buf[b + 23], fir_coefs[0] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 1] + buf[b + 22], fir_coefs[1] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 2] + buf[b + 21], fir_coefs[2] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 3] + buf[b + 20], fir_coefs[3] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 4] + buf[b + 19], fir_coefs[4] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 5] + buf[b + 18], fir_coefs[5] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 6] + buf[b + 17], fir_coefs[6] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 7] + buf[b + 16], fir_coefs[7] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 8] + buf[b + 15], fir_coefs[8] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 9] + buf[b + 14], fir_coefs[9] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 10] + buf[b + 13], fir_coefs[10] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 11] + buf[b + 12], fir_coefs[11] as i32);

                out[n] = silk_sat16(silk_rshift_round(res_q6, 6)) as i16;
                n += 1;

                index_q16 += index_increment_q16;
            }
        }
        RESAMPLER_DOWN_ORDER_FIR2 => {
            let mut index_q16 = 0i32;
            while index_q16 < max_index_q16 {
                let b = (index_q16 >> 16) as usize;

                let mut res_q6 = silk_smulwb(buf[b] + buf[b + 35], fir_coefs[0] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 1] + buf[b + 34], fir_coefs[1] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 2] + buf[b + 33], fir_coefs[2] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 3] + buf[b + 32], fir_coefs[3] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 4] + buf[b + 31], fir_coefs[4] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 5] + buf[b + 30], fir_coefs[5] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 6] + buf[b + 29], fir_coefs[6] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 7] + buf[b + 28], fir_coefs[7] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 8] + buf[b + 27], fir_coefs[8] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 9] + buf[b + 26], fir_coefs[9] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 10] + buf[b + 25], fir_coefs[10] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 11] + buf[b + 24], fir_coefs[11] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 12] + buf[b + 23], fir_coefs[12] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 13] + buf[b + 22], fir_coefs[13] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 14] + buf[b + 21], fir_coefs[14] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 15] + buf[b + 20], fir_coefs[15] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 16] + buf[b + 19], fir_coefs[16] as i32);
                res_q6 = silk_smlawb(res_q6, buf[b + 17] + buf[b + 18], fir_coefs[17] as i32);

                out[n] = silk_sat16(silk_rshift_round(res_q6, 6)) as i16;
                n += 1;

                index_q16 += index_increment_q16;
            }
        }
        _ => { /* silk_assert(0); */ }
    }
    n
}

/// `silk_resampler_private_down_FIR` — downsampler (rational ratios).
pub fn silk_resampler_private_down_fir(s: &mut SilkResamplerStateStruct, out: &mut [i16], in_: &[i16], mut in_len: i32) {
    let mut buf = [0i32; RESAMPLER_MAX_BATCH_SIZE_IN + SILK_RESAMPLER_MAX_FIR_ORDER];

    /* Copy buffered samples to start of buffer */
    let fir_order = s.fir_order as usize;
    buf[..fir_order].copy_from_slice(&s.s_fir[..fir_order]);

    let coefs = s.coefs.or_panic("down-FIR resampler coefs unset (silk_resampler_init not run?)");
    let fir_coefs = &coefs[2..];

    /* Iterate over blocks of frameSizeIn input samples */
    let index_increment_q16 = s.inv_ratio_q16;
    let mut out_off = 0usize;
    let mut in_off = 0usize;
    let mut n_samples_in;
    loop {
        n_samples_in = in_len.min(s.batch_size);

        /* Second-order AR filter (output in Q8) */
        silk_resampler_private_ar2(&mut s.s_iir, &mut buf[fir_order..], &in_[in_off..], coefs, n_samples_in);

        let max_index_q16 = silk_lshift(n_samples_in, 16);

        /* Interpolate filtered signal */
        out_off += silk_resampler_private_down_fir_interpol(
            &mut out[out_off..],
            &buf,
            fir_coefs,
            s.fir_order,
            s.fir_fracs,
            max_index_q16,
            index_increment_q16,
        );

        in_off += n_samples_in as usize;
        in_len -= n_samples_in;

        if in_len > 1 {
            /* More iterations to do; copy last part of filtered signal to beginning of buffer */
            // In bounds: n_samples_in <= batch_size <= RESAMPLER_MAX_BATCH_SIZE_IN and
            // fir_order <= SILK_RESAMPLER_MAX_FIR_ORDER, so the source end fits buf.
            buf.copy_within(n_samples_in as usize..n_samples_in as usize + fir_order, 0);
        } else {
            break;
        }
    }

    /* Copy last part of filtered signal to the state for the next call */
    s.s_fir[..fir_order].copy_from_slice(&buf[n_samples_in as usize..n_samples_in as usize + fir_order]);
}
