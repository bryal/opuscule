// Translated from c/celt/celt.c (RFC 6716).
//
// CELT decoder core: helper functions and the main decode entry point.
// Functions are translated incrementally, innermost helpers first.

use std::os::raw::c_int;

use crate::arch::{CeltSig, OpusVal16, OpusVal32, Q15ONE, mult16_16_q15, mult16_32_q15, qconst16, shl32};
#[cfg(feature = "fixed-point")]
use crate::arch::{extract16, max32, min32, pshr32};
use crate::entcode::ec_ctx;
use crate::entdec::{ec_dec_bit_logp, ec_tell};
use crate::mdct::{MdctLookup, clt_mdct_backward};
use crate::modes::CELTMode;

// -- Constants --

const SIG_SHIFT: c_int = 12;

#[cfg(not(feature = "fixed-point"))]
const CELT_SIG_SCALE: f32 = 32768.0;

// -- tf_select_table (used by tf_decode) --

static TF_SELECT_TABLE: [[i8; 8]; 4] =
    [[0, -1, 0, -1, 0, -1, 0, -1], [0, -1, 0, -2, 1, 0, 1, -1], [0, -2, 0, -3, 2, 0, 1, -1], [0, -2, 0, -3, 3, 0, 1, -1]];

// -- Small helper functions --

/// Map sample rate to resampling factor.
/// Only the five standard Opus rates are supported.
#[unsafe(no_mangle)]
pub extern "C" fn resampling_factor(rate: i32) -> c_int {
    match rate {
        48000 => 1,
        24000 => 2,
        16000 => 3,
        12000 => 4,
        8000 => 6,
        _ => {
            debug_assert!(false, "unsupported sample rate");
            0
        }
    }
}

/// Convert a celt_sig to a 16-bit output sample.
///
/// Fixed-point: right-shift by SIG_SHIFT, clamp to [-32768, 32767].
/// Float: identity cast (scaling happens in deemphasis via SCALEOUT).
#[unsafe(no_mangle)]
#[cfg(feature = "fixed-point")]
pub extern "C" fn sig2word16(x: crate::arch::CeltSig) -> OpusVal16 {
    let x = pshr32(x, SIG_SHIFT);
    let x = max32(x, -32768);
    let x = min32(x, 32767);
    extract16(x)
}

#[unsafe(no_mangle)]
#[cfg(not(feature = "fixed-point"))]
pub extern "C" fn sig2word16(x: crate::arch::CeltSig) -> OpusVal16 {
    x
}

/// Scale an output sample for final PCM output.
///
/// Fixed-point: identity (already in Q0).
/// Float: divide by CELT_SIG_SCALE (32768).
#[inline(always)]
#[cfg(feature = "fixed-point")]
pub fn scaleout(a: OpusVal16) -> OpusVal16 {
    a
}

#[inline(always)]
#[cfg(not(feature = "fixed-point"))]
pub fn scaleout(a: OpusVal16) -> OpusVal16 {
    a * (1.0 / CELT_SIG_SCALE)
}

// -- tf_decode --

/// Decode the time-frequency resolution flags for each band.
///
/// Reads a sequence of binary flags from the entropy coder indicating
/// whether each band uses a finer time or frequency resolution, then
/// applies a selection table to map these to actual tf_change values.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn tf_decode(
    start: c_int,
    end: c_int,
    is_transient: c_int,
    tf_res: *mut c_int,
    lm: c_int,
    dec: *mut ec_ctx,
) {
    unsafe {
        let budget = (*dec).storage as u32 * 8;
        let mut tell = ec_tell(&*dec) as u32;
        let mut logp: u32 = if is_transient != 0 { 2 } else { 4 };
        let tf_select_rsv = (lm > 0 && tell + logp + 1 <= budget) as c_int;
        let budget = budget - tf_select_rsv as u32;
        let mut tf_changed = 0;
        let mut curr = 0;
        for i in start..end {
            if tell + logp <= budget {
                curr ^= ec_dec_bit_logp(dec, logp);
                tell = ec_tell(&*dec) as u32;
                tf_changed |= curr;
            }
            *tf_res.add(i as usize) = curr;
            logp = if is_transient != 0 { 4 } else { 5 };
        }
        let mut tf_select = 0;
        if tf_select_rsv != 0
            && TF_SELECT_TABLE[lm as usize][(4 * is_transient + 0 + tf_changed) as usize]
                != TF_SELECT_TABLE[lm as usize][(4 * is_transient + 2 + tf_changed) as usize]
        {
            tf_select = ec_dec_bit_logp(dec, 1);
        }
        for i in start..end {
            *tf_res.add(i as usize) =
                TF_SELECT_TABLE[lm as usize][(4 * is_transient + 2 * tf_select + *tf_res.add(i as usize)) as usize] as c_int;
        }
    }
}

// -- init_caps --

/// Initialise the per-band bit allocation caps from the mode's cache.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn init_caps(m: *const CELTMode, cap: *mut c_int, lm: c_int, c: c_int) {
    unsafe {
        let mode = &*m;
        for i in 0..mode.nb_ebands as usize {
            let n = ((*mode.ebands.add(i + 1) - *mode.ebands.add(i)) as c_int) << lm;
            *cap.add(i) =
                (*mode.cache.caps.add(mode.nb_ebands as usize * (2 * lm as usize + c as usize - 1) + i) as c_int + 64) * c * n
                    >> 2;
        }
    }
}

// -- compute_inv_mdcts --

/// Compute inverse MDCTs for all channels, with overlap-add.
///
/// For each channel: runs the IMDCT (possibly multiple short blocks),
/// overlap-adds with the previous frame's tail, and saves the new tail
/// into overlap_mem.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn compute_inv_mdcts(
    mode: *const CELTMode,
    short_blocks: c_int,
    x: *mut CeltSig,
    out_mem: *mut *mut CeltSig,
    overlap_mem: *mut *mut CeltSig,
    c_channels: c_int,
    lm: c_int,
) {
    unsafe {
        let mode = &*mode;
        let n = mode.short_mdct_size << lm;
        let overlap = mode.overlap;
        let mut buf = vec![0 as OpusVal32; (n + overlap) as usize];
        let x_buf = buf.as_mut_ptr();

        let mut c = 0;
        loop {
            let mut n2 = n;
            let mut b_count = 1;

            if short_blocks != 0 {
                n2 = mode.short_mdct_size;
                b_count = short_blocks;
            }
            // Prevents problems from the imdct doing the overlap-add
            for j in 0..overlap as usize {
                *x_buf.add(j) = 0 as OpusVal32;
            }

            for b in 0..b_count {
                clt_mdct_backward(
                    &mode.mdct as *const MdctLookup,
                    x.add((b + c * n2 * b_count) as usize),
                    x_buf.add((n2 * b) as usize),
                    mode.window,
                    overlap,
                    if short_blocks != 0 { mode.max_lm } else { mode.max_lm - lm },
                    b_count,
                );
            }

            let out = *out_mem.add(c as usize);
            let ovlp = *overlap_mem.add(c as usize);
            for j in 0..overlap as usize {
                *out.add(j) = *x_buf.add(j) + *ovlp.add(j);
            }
            for j in overlap as usize..n as usize {
                *out.add(j) = *x_buf.add(j);
            }
            for j in 0..overlap as usize {
                *ovlp.add(j) = *x_buf.add((n as usize) + j);
            }

            c += 1;
            if c >= c_channels {
                break;
            }
        }
    }
}

// -- deemphasis --

/// Apply de-emphasis filter and produce interleaved PCM output.
///
/// The de-emphasis is a first-order IIR filter that undoes the pre-emphasis
/// applied before encoding. Also handles downsampling (e.g. 48→8 kHz)
/// by writing only every `downsample`-th sample.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn deemphasis(
    in_: *mut *mut CeltSig,
    pcm: *mut OpusVal16,
    n: c_int,
    c_channels: c_int,
    downsample: c_int,
    coef: *const OpusVal16,
    mem: *mut CeltSig,
) {
    unsafe {
        let mut count: c_int = 0;
        let mut c = 0;
        loop {
            let x = *in_.add(c as usize);
            let mut y = pcm.add(c as usize);
            let mut m = *mem.add(c as usize);
            for j in 0..n {
                let xj = *x.add(j as usize);
                let tmp = xj + m;
                m = mult16_32_q15(*coef.add(0), tmp) - mult16_32_q15(*coef.add(1), xj);
                let tmp = shl32(mult16_32_q15(*coef.add(3), tmp), 2);
                if count == 0 {
                    *y = scaleout(sig2word16(tmp));
                }
                count += 1;
                if count == downsample {
                    y = y.add(c_channels as usize);
                    count = 0;
                }
            }
            *mem.add(c as usize) = m;

            c += 1;
            if c >= c_channels {
                break;
            }
        }
    }
}

// -- comb_filter --

/// Post-filter comb filter with smooth crossfade between two pitch periods.
///
/// Applies a 3-tap comb filter at pitch lag T1 for the steady-state portion,
/// and crossfades from the old pitch T0 to T1 over the overlap region using
/// a squared-window interpolation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn comb_filter(
    y: *mut OpusVal32,
    x: *mut OpusVal32,
    t0: c_int,
    t1: c_int,
    n: c_int,
    g0: OpusVal16,
    g1: OpusVal16,
    tapset0: c_int,
    tapset1: c_int,
    window: *const OpusVal16,
    overlap: c_int,
) {
    unsafe {
        #[cfg(not(feature = "fixed-point"))]
        let gains: [[OpusVal16; 3]; 3] =
            [[0.3066406250, 0.2170410156, 0.1296386719], [0.4638671875, 0.2680664062, 0.0], [0.7998046875, 0.1000976562, 0.0]];
        #[cfg(feature = "fixed-point")]
        let gains: [[OpusVal16; 3]; 3] = [
            [qconst16(0.3066406250, 15), qconst16(0.2170410156, 15), qconst16(0.1296386719, 15)],
            [qconst16(0.4638671875, 15), qconst16(0.2680664062, 15), qconst16(0.0, 15)],
            [qconst16(0.7998046875, 15), qconst16(0.1000976562, 15), qconst16(0.0, 15)],
        ];

        let g00 = mult16_16_q15(g0, gains[tapset0 as usize][0]) as OpusVal16;
        let g01 = mult16_16_q15(g0, gains[tapset0 as usize][1]) as OpusVal16;
        let g02 = mult16_16_q15(g0, gains[tapset0 as usize][2]) as OpusVal16;
        let g10 = mult16_16_q15(g1, gains[tapset1 as usize][0]) as OpusVal16;
        let g11 = mult16_16_q15(g1, gains[tapset1 as usize][1]) as OpusVal16;
        let g12 = mult16_16_q15(g1, gains[tapset1 as usize][2]) as OpusVal16;

        for i in 0..overlap as isize {
            let f = mult16_16_q15(*window.offset(i), *window.offset(i)) as OpusVal16;
            let one_minus_f = (Q15ONE - f) as OpusVal16;
            *y.offset(i) = *x.offset(i)
                + mult16_32_q15(mult16_16_q15(one_minus_f, g00) as OpusVal16, *x.offset(i - t0 as isize))
                + mult16_32_q15(mult16_16_q15(one_minus_f, g01) as OpusVal16, *x.offset(i - t0 as isize - 1))
                + mult16_32_q15(mult16_16_q15(one_minus_f, g01) as OpusVal16, *x.offset(i - t0 as isize + 1))
                + mult16_32_q15(mult16_16_q15(one_minus_f, g02) as OpusVal16, *x.offset(i - t0 as isize - 2))
                + mult16_32_q15(mult16_16_q15(one_minus_f, g02) as OpusVal16, *x.offset(i - t0 as isize + 2))
                + mult16_32_q15(mult16_16_q15(f, g10) as OpusVal16, *x.offset(i - t1 as isize))
                + mult16_32_q15(mult16_16_q15(f, g11) as OpusVal16, *x.offset(i - t1 as isize - 1))
                + mult16_32_q15(mult16_16_q15(f, g11) as OpusVal16, *x.offset(i - t1 as isize + 1))
                + mult16_32_q15(mult16_16_q15(f, g12) as OpusVal16, *x.offset(i - t1 as isize - 2))
                + mult16_32_q15(mult16_16_q15(f, g12) as OpusVal16, *x.offset(i - t1 as isize + 2));
        }
        for i in overlap as isize..n as isize {
            *y.offset(i) = *x.offset(i)
                + mult16_32_q15(g10, *x.offset(i - t1 as isize))
                + mult16_32_q15(g11, *x.offset(i - t1 as isize - 1))
                + mult16_32_q15(g11, *x.offset(i - t1 as isize + 1))
                + mult16_32_q15(g12, *x.offset(i - t1 as isize - 2))
                + mult16_32_q15(g12, *x.offset(i - t1 as isize + 2));
        }
    }
}
