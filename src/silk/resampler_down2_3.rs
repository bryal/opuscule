//! Translated from `c/silk/resampler_down2_3.c` (RFC 6716).
//!
//! Low-quality 2/3 fractional downsampler used by some narrow-band paths.

use super::macros::{silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb};
use super::resampler_private_AR2::silk_resampler_private_AR2;
use super::resampler_rom::silk_Resampler_2_3_COEFS_LQ;

const ORDER_FIR: usize = 4;

/// `RESAMPLER_MAX_BATCH_SIZE_MS * RESAMPLER_MAX_FS_KHZ` from
/// `c/silk/resampler_private.h`.
const RESAMPLER_MAX_BATCH_SIZE_IN: usize = 10 * 48;

/// `silk_resampler_down2_3` — downsample by a factor 2/3, low quality.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_resampler_down2_3(s: *mut i32, out: *mut i16, in_: *const i16, in_len: i32) {
    unsafe {
        let mut buf = [0i32; RESAMPLER_MAX_BATCH_SIZE_IN + ORDER_FIR];

        /* Copy buffered samples to start of buffer */
        core::ptr::copy_nonoverlapping(s, buf.as_mut_ptr(), ORDER_FIR);

        let mut in_ = in_;
        let mut out = out;
        let mut in_len = in_len;
        let mut n_samples_in = 0i32;

        /* Iterate over blocks of frameSizeIn input samples */
        loop {
            n_samples_in = in_len.min(RESAMPLER_MAX_BATCH_SIZE_IN as i32);

            /* Second-order AR filter (output in Q8) */
            silk_resampler_private_AR2(
                s.offset(ORDER_FIR as isize),
                buf.as_mut_ptr().offset(ORDER_FIR as isize),
                in_,
                silk_Resampler_2_3_COEFS_LQ.as_ptr(),
                n_samples_in,
            );

            /* Interpolate filtered signal */
            let mut buf_ptr: *mut i32 = buf.as_mut_ptr();
            let mut counter = n_samples_in;
            while counter > 2 {
                /* Inner product */
                let mut res_q6 = silk_smulwb(*buf_ptr.offset(0), silk_Resampler_2_3_COEFS_LQ[2] as i32);
                res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(1), silk_Resampler_2_3_COEFS_LQ[3] as i32);
                res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(2), silk_Resampler_2_3_COEFS_LQ[5] as i32);
                res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(3), silk_Resampler_2_3_COEFS_LQ[4] as i32);

                /* Scale down, saturate and store in output array */
                *out = silk_sat16(silk_rshift_round(res_q6, 6)) as i16;
                out = out.offset(1);

                let mut res_q6 = silk_smulwb(*buf_ptr.offset(1), silk_Resampler_2_3_COEFS_LQ[4] as i32);
                res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(2), silk_Resampler_2_3_COEFS_LQ[5] as i32);
                res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(3), silk_Resampler_2_3_COEFS_LQ[3] as i32);
                res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(4), silk_Resampler_2_3_COEFS_LQ[2] as i32);

                /* Scale down, saturate and store in output array */
                *out = silk_sat16(silk_rshift_round(res_q6, 6)) as i16;
                out = out.offset(1);

                buf_ptr = buf_ptr.offset(3);
                counter -= 3;
            }

            in_ = in_.offset(n_samples_in as isize);
            in_len -= n_samples_in;

            if in_len > 0 {
                /* More iterations to do; copy last part of filtered signal to beginning of buffer */
                core::ptr::copy_nonoverlapping(buf.as_ptr().offset(n_samples_in as isize), buf.as_mut_ptr(), ORDER_FIR);
            } else {
                break;
            }
        }

        /* Copy last part of filtered signal to the state for the next call */
        core::ptr::copy_nonoverlapping(buf.as_ptr().offset(n_samples_in as isize), s, ORDER_FIR);
    }
}
