//! Translated from `c/silk/resampler_private_down_FIR.c` (RFC 6716).
//!
//! Resample-by-rational via a 2nd-order AR section followed by an FIR
//! interpolator. Three FIR orders are hard-coded (18, 24, 36) and
//! dispatched on `FIR_Order` to let the compiler unroll the inner
//! product.

use core::ffi::c_void;

use super::macros::{silk_lshift, silk_rshift_round, silk_sat16, silk_smlawb, silk_smulwb};
use super::resampler_private_AR2::silk_resampler_private_ar2;
use super::resampler_rom::{RESAMPLER_DOWN_ORDER_FIR0, RESAMPLER_DOWN_ORDER_FIR1, RESAMPLER_DOWN_ORDER_FIR2};
use super::structs::{SILK_RESAMPLER_MAX_FIR_ORDER, SilkResamplerStateStruct};

const RESAMPLER_MAX_BATCH_SIZE_IN: usize = 10 * 48;

/// Inner interpolation loop — switches on the FIR order to let the
/// compiler unroll the inner dot product.
#[inline]
unsafe fn silk_resampler_private_down_fir_interpol(
    mut out: *mut i16,
    buf: *mut i32,
    fir_coefs: *const i16,
    fir_order: i32,
    fir_fracs: i32,
    max_index_q16: i32,
    index_increment_q16: i32,
) -> *mut i16 {
    unsafe {
        match fir_order as usize {
            RESAMPLER_DOWN_ORDER_FIR0 => {
                let mut index_q16 = 0i32;
                while index_q16 < max_index_q16 {
                    /* Integer part gives pointer to buffered input */
                    let buf_ptr = buf.offset((index_q16 >> 16) as isize);

                    /* Fractional part gives interpolation coefficients */
                    let interpol_ind = silk_smulwb(index_q16 & 0xFFFF, fir_fracs);

                    /* Inner product */
                    let mut interpol_ptr = fir_coefs.offset((RESAMPLER_DOWN_ORDER_FIR0 as i32 / 2 * interpol_ind) as isize);
                    let mut res_q6 = silk_smulwb(*buf_ptr.offset(0), *interpol_ptr.offset(0) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(1), *interpol_ptr.offset(1) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(2), *interpol_ptr.offset(2) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(3), *interpol_ptr.offset(3) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(4), *interpol_ptr.offset(4) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(5), *interpol_ptr.offset(5) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(6), *interpol_ptr.offset(6) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(7), *interpol_ptr.offset(7) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(8), *interpol_ptr.offset(8) as i32);
                    interpol_ptr =
                        fir_coefs.offset((RESAMPLER_DOWN_ORDER_FIR0 as i32 / 2 * (fir_fracs - 1 - interpol_ind)) as isize);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(17), *interpol_ptr.offset(0) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(16), *interpol_ptr.offset(1) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(15), *interpol_ptr.offset(2) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(14), *interpol_ptr.offset(3) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(13), *interpol_ptr.offset(4) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(12), *interpol_ptr.offset(5) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(11), *interpol_ptr.offset(6) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(10), *interpol_ptr.offset(7) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(9), *interpol_ptr.offset(8) as i32);

                    /* Scale down, saturate and store in output array */
                    *out = silk_sat16(silk_rshift_round(res_q6, 6)) as i16;
                    out = out.offset(1);

                    index_q16 += index_increment_q16;
                }
            }
            RESAMPLER_DOWN_ORDER_FIR1 => {
                let mut index_q16 = 0i32;
                while index_q16 < max_index_q16 {
                    let buf_ptr = buf.offset((index_q16 >> 16) as isize);

                    /* Inner product */
                    let mut res_q6 = silk_smulwb(*buf_ptr.offset(0) + *buf_ptr.offset(23), *fir_coefs.offset(0) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(1) + *buf_ptr.offset(22), *fir_coefs.offset(1) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(2) + *buf_ptr.offset(21), *fir_coefs.offset(2) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(3) + *buf_ptr.offset(20), *fir_coefs.offset(3) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(4) + *buf_ptr.offset(19), *fir_coefs.offset(4) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(5) + *buf_ptr.offset(18), *fir_coefs.offset(5) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(6) + *buf_ptr.offset(17), *fir_coefs.offset(6) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(7) + *buf_ptr.offset(16), *fir_coefs.offset(7) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(8) + *buf_ptr.offset(15), *fir_coefs.offset(8) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(9) + *buf_ptr.offset(14), *fir_coefs.offset(9) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(10) + *buf_ptr.offset(13), *fir_coefs.offset(10) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(11) + *buf_ptr.offset(12), *fir_coefs.offset(11) as i32);

                    *out = silk_sat16(silk_rshift_round(res_q6, 6)) as i16;
                    out = out.offset(1);

                    index_q16 += index_increment_q16;
                }
            }
            RESAMPLER_DOWN_ORDER_FIR2 => {
                let mut index_q16 = 0i32;
                while index_q16 < max_index_q16 {
                    let buf_ptr = buf.offset((index_q16 >> 16) as isize);

                    let mut res_q6 = silk_smulwb(*buf_ptr.offset(0) + *buf_ptr.offset(35), *fir_coefs.offset(0) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(1) + *buf_ptr.offset(34), *fir_coefs.offset(1) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(2) + *buf_ptr.offset(33), *fir_coefs.offset(2) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(3) + *buf_ptr.offset(32), *fir_coefs.offset(3) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(4) + *buf_ptr.offset(31), *fir_coefs.offset(4) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(5) + *buf_ptr.offset(30), *fir_coefs.offset(5) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(6) + *buf_ptr.offset(29), *fir_coefs.offset(6) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(7) + *buf_ptr.offset(28), *fir_coefs.offset(7) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(8) + *buf_ptr.offset(27), *fir_coefs.offset(8) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(9) + *buf_ptr.offset(26), *fir_coefs.offset(9) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(10) + *buf_ptr.offset(25), *fir_coefs.offset(10) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(11) + *buf_ptr.offset(24), *fir_coefs.offset(11) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(12) + *buf_ptr.offset(23), *fir_coefs.offset(12) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(13) + *buf_ptr.offset(22), *fir_coefs.offset(13) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(14) + *buf_ptr.offset(21), *fir_coefs.offset(14) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(15) + *buf_ptr.offset(20), *fir_coefs.offset(15) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(16) + *buf_ptr.offset(19), *fir_coefs.offset(16) as i32);
                    res_q6 = silk_smlawb(res_q6, *buf_ptr.offset(17) + *buf_ptr.offset(18), *fir_coefs.offset(17) as i32);

                    *out = silk_sat16(silk_rshift_round(res_q6, 6)) as i16;
                    out = out.offset(1);

                    index_q16 += index_increment_q16;
                }
            }
            _ => { /* silk_assert(0); */ }
        }
        out
    }
}

/// `silk_resampler_private_down_FIR` — downsampler (rational ratios).
pub unsafe fn silk_resampler_private_down_fir(ss: *mut c_void, mut out: *mut i16, mut in_: *const i16, mut in_len: i32) {
    unsafe {
        let s = ss as *mut SilkResamplerStateStruct;
        let mut buf = [0i32; RESAMPLER_MAX_BATCH_SIZE_IN + SILK_RESAMPLER_MAX_FIR_ORDER];

        /* Copy buffered samples to start of buffer */
        core::ptr::copy_nonoverlapping((*s).s_fir.as_ptr(), buf.as_mut_ptr(), (*s).fir_order as usize);

        let fir_coefs = (*s).coefs.offset(2);

        /* Iterate over blocks of frameSizeIn input samples */
        let index_increment_q16 = (*s).inv_ratio_q16;
        let mut n_samples_in;
        loop {
            n_samples_in = in_len.min((*s).batch_size);

            /* Second-order AR filter (output in Q8) */
            silk_resampler_private_ar2(
                (*s).s_iir.as_mut_ptr(),
                buf.as_mut_ptr().offset((*s).fir_order as isize),
                in_,
                (*s).coefs,
                n_samples_in,
            );

            let max_index_q16 = silk_lshift(n_samples_in, 16);

            /* Interpolate filtered signal */
            out = silk_resampler_private_down_fir_interpol(
                out,
                buf.as_mut_ptr(),
                fir_coefs,
                (*s).fir_order,
                (*s).fir_fracs,
                max_index_q16,
                index_increment_q16,
            );

            in_ = in_.offset(n_samples_in as isize);
            in_len -= n_samples_in;

            if in_len > 1 {
                /* More iterations to do; copy last part of filtered signal to beginning of buffer */
                core::ptr::copy_nonoverlapping(
                    buf.as_ptr().offset(n_samples_in as isize),
                    buf.as_mut_ptr(),
                    (*s).fir_order as usize,
                );
            } else {
                break;
            }
        }

        /* Copy last part of filtered signal to the state for the next call */
        core::ptr::copy_nonoverlapping(
            buf.as_ptr().offset(n_samples_in as isize),
            (*s).s_fir.as_mut_ptr(),
            (*s).fir_order as usize,
        );
    }
}
