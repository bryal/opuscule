//! Translated from `c/silk/resampler_private_IIR_FIR.c` (RFC 6716).
//!
//! Upsampler that combines allpass-based 2× upsampling (via
//! [`silk_resampler_private_up2_HQ`]) with 12-fraction FIR interpolation
//! to reach the target output rate. The FIR state lives in the host
//! resampler struct's `sFIR` field, which is interpreted as a stream of
//! `i16`s for this path (the C uses `opus_int32 sFIR[36]` but copies
//! `RESAMPLER_ORDER_FIR_12 * sizeof(opus_int32) = 32` bytes into an
//! `opus_int16 buf[]`, treating those 32 bytes as 16 `i16`s — preserved
//! literally here).

use core::ffi::c_void;

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlabb, silk_smulbb, silk_smulwb};
use super::resampler_private_up2_HQ::silk_resampler_private_up2_hq;
use super::resampler_rom::{RESAMPLER_ORDER_FIR_12, SILK_RESAMPLER_FRAC_FIR_12};
use super::structs::SilkResamplerStateStruct;

/// `RESAMPLER_MAX_BATCH_SIZE_MS * RESAMPLER_MAX_FS_KHZ` from
/// `c/silk/resampler_private.h`.
const RESAMPLER_MAX_BATCH_SIZE_IN: usize = 10 * 48;

/// Inner interpolation loop — returns the advanced output pointer.
#[inline]
unsafe fn silk_resampler_private_iir_fir_interpol(
    mut out: *mut i16,
    buf: *mut i16,
    max_index_q16: i32,
    index_increment_q16: i32,
) -> *mut i16 {
    unsafe {
        /* Interpolate upsampled signal and store in output array */
        let mut index_q16: i32 = 0;
        while index_q16 < max_index_q16 {
            let table_index = silk_smulwb(index_q16 & 0xFFFF, 12);
            let buf_ptr = buf.offset((index_q16 >> 16) as isize);

            let tbl = SILK_RESAMPLER_FRAC_FIR_12[table_index as usize];
            let tbl_rev = SILK_RESAMPLER_FRAC_FIR_12[(11 - table_index) as usize];
            let mut res_q15 = silk_smulbb(*buf_ptr.offset(0) as i32, tbl[0] as i32);
            res_q15 = silk_smlabb(res_q15, *buf_ptr.offset(1) as i32, tbl[1] as i32);
            res_q15 = silk_smlabb(res_q15, *buf_ptr.offset(2) as i32, tbl[2] as i32);
            res_q15 = silk_smlabb(res_q15, *buf_ptr.offset(3) as i32, tbl[3] as i32);
            res_q15 = silk_smlabb(res_q15, *buf_ptr.offset(4) as i32, tbl_rev[3] as i32);
            res_q15 = silk_smlabb(res_q15, *buf_ptr.offset(5) as i32, tbl_rev[2] as i32);
            res_q15 = silk_smlabb(res_q15, *buf_ptr.offset(6) as i32, tbl_rev[1] as i32);
            res_q15 = silk_smlabb(res_q15, *buf_ptr.offset(7) as i32, tbl_rev[0] as i32);
            *out = silk_sat16(silk_rshift_round(res_q15, 15)) as i16;
            out = out.offset(1);

            index_q16 += index_increment_q16;
        }
        out
    }
}

/// `silk_resampler_private_IIR_FIR` — 2× IIR upsample + fractional FIR.
pub unsafe fn silk_resampler_private_iir_fir(ss: *mut c_void, mut out: *mut i16, mut in_: *const i16, mut in_len: i32) {
    unsafe {
        let s = ss as *mut SilkResamplerStateStruct;
        let mut buf = [0i16; RESAMPLER_MAX_BATCH_SIZE_IN + RESAMPLER_ORDER_FIR_12];

        /* Copy buffered samples to start of buffer */
        /* C: silk_memcpy(buf, S->sFIR, RESAMPLER_ORDER_FIR_12 * sizeof(opus_int32))
         * — copies 32 bytes (RESAMPLER_ORDER_FIR_12=8 × sizeof(int32)=4) into an
         * int16[] buffer. That's 16 int16s worth of bytes, taken from the first
         * 8 int32 slots of sFIR. We replicate the byte-for-byte copy. */
        core::ptr::copy_nonoverlapping(
            (*s).s_fir.as_ptr() as *const u8,
            buf.as_mut_ptr() as *mut u8,
            RESAMPLER_ORDER_FIR_12 * core::mem::size_of::<i32>(),
        );

        /* Iterate over blocks of frameSizeIn input samples */
        let index_increment_q16 = (*s).inv_ratio_q16;
        let mut n_samples_in;
        loop {
            n_samples_in = in_len.min((*s).batch_size);

            /* Upsample 2x */
            silk_resampler_private_up2_hq(
                (*s).s_iir.as_mut_ptr(),
                buf.as_mut_ptr().offset(RESAMPLER_ORDER_FIR_12 as isize),
                in_,
                n_samples_in,
            );

            let max_index_q16 = silk_lshift(n_samples_in, 16 + 1); /* + 1 because 2x upsampling */
            out = silk_resampler_private_iir_fir_interpol(out, buf.as_mut_ptr(), max_index_q16, index_increment_q16);
            in_ = in_.offset(n_samples_in as isize);
            in_len -= n_samples_in;

            if in_len > 0 {
                /* More iterations to do; copy last part of filtered signal to beginning of buffer */
                core::ptr::copy_nonoverlapping(
                    buf.as_ptr().offset((n_samples_in << 1) as isize) as *const u8,
                    buf.as_mut_ptr() as *mut u8,
                    RESAMPLER_ORDER_FIR_12 * core::mem::size_of::<i32>(),
                );
            } else {
                break;
            }
        }

        /* Copy last part of filtered signal to the state for the next call */
        core::ptr::copy_nonoverlapping(
            buf.as_ptr().offset((n_samples_in << 1) as isize) as *const u8,
            (*s).s_fir.as_mut_ptr() as *mut u8,
            RESAMPLER_ORDER_FIR_12 * core::mem::size_of::<i32>(),
        );
    }
}
