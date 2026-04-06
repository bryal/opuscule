// Translated from c/celt/celt.c (RFC 6716).
//
// CELT decoder core: helper functions and the main decode entry point.
// Functions are translated incrementally, innermost helpers first.

use std::os::raw::c_int;

use crate::arch::OpusVal16;
#[cfg(feature = "fixed-point")]
use crate::arch::{extract16, max32, min32, pshr32};
use crate::entcode::ec_ctx;
use crate::entdec::{ec_dec_bit_logp, ec_tell};
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
