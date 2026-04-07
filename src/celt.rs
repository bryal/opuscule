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
use crate::modes::{CELTMode, opus_custom_mode_create};

// -- Constants --

const SIG_SHIFT: c_int = 12;

#[cfg(not(feature = "fixed-point"))]
const CELT_SIG_SCALE: f32 = 32768.0;

pub const DECODE_BUFFER_SIZE: c_int = 2048;
const LPC_ORDER: c_int = 24;
const DB_SHIFT: c_int = 10;

// -- Opus error codes (from opus_defines.h) --

const OPUS_OK: c_int = 0;
const OPUS_BAD_ARG: c_int = -1;
const OPUS_ALLOC_FAIL: c_int = -7;
const OPUS_RESET_STATE: c_int = 4028;

// -- OpusCustomDecoder (CELTDecoder) struct --

/// CELT decoder state.
///
/// Matches the C `struct OpusCustomDecoder` (typedef'd as `CELTDecoder`).
/// Uses a flexible array member pattern: `_decode_mem` is declared as
/// `[CeltSig; 1]` but the actual allocation is larger, with trailing
/// space for lpc, oldBandE, oldLogE, oldLogE2, and backgroundLogE.
#[repr(C)]
pub struct OpusCustomDecoder {
    pub mode: *const CELTMode,
    pub overlap: c_int,
    pub channels: c_int,
    pub stream_channels: c_int,

    pub downsample: c_int,
    pub start: c_int,
    pub end: c_int,
    pub signalling: c_int,

    // Everything beyond this point gets cleared on a reset
    // (DECODER_RESET_START = rng)
    pub rng: u32,
    pub error: c_int,
    pub last_pitch_index: c_int,
    pub loss_count: c_int,
    pub postfilter_period: c_int,
    pub postfilter_period_old: c_int,
    pub postfilter_gain: OpusVal16,
    pub postfilter_gain_old: OpusVal16,
    pub postfilter_tapset: c_int,
    pub postfilter_tapset_old: c_int,

    pub preemph_mem_d: [CeltSig; 2],

    pub _decode_mem: [CeltSig; 1],
}

/// Type alias matching `typedef struct OpusCustomDecoder CELTDecoder`.
pub type CELTDecoder = OpusCustomDecoder;

// -- Decoder size and init functions --

/// Return the size in bytes of a CELT decoder for the standard Opus mode.
#[unsafe(no_mangle)]
pub extern "C" fn celt_decoder_get_size(channels: c_int) -> c_int {
    let mode = unsafe { opus_custom_mode_create(48000, 960, std::ptr::null_mut()) };
    opus_custom_decoder_get_size(mode, channels)
}

/// Return the size in bytes of a CELT decoder for a given mode.
#[unsafe(no_mangle)]
pub extern "C" fn opus_custom_decoder_get_size(mode: *const CELTMode, channels: c_int) -> c_int {
    unsafe {
        let size = std::mem::size_of::<OpusCustomDecoder>() as c_int
            + (channels * (DECODE_BUFFER_SIZE + (*mode).overlap) - 1) * std::mem::size_of::<CeltSig>() as c_int
            + channels * LPC_ORDER * std::mem::size_of::<OpusVal16>() as c_int
            + 4 * 2 * (*mode).nb_ebands * std::mem::size_of::<OpusVal16>() as c_int;
        size
    }
}

/// Initialise a CELT decoder for the standard Opus mode at the given sample rate.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn celt_decoder_init(st: *mut CELTDecoder, sampling_rate: i32, channels: c_int) -> c_int {
    unsafe {
        let ret = opus_custom_decoder_init(st, opus_custom_mode_create(48000, 960, std::ptr::null_mut()), channels);
        if ret != OPUS_OK {
            return ret;
        }
        (*st).downsample = resampling_factor(sampling_rate);
        if (*st).downsample == 0 { OPUS_BAD_ARG } else { OPUS_OK }
    }
}

/// Initialise a CELT decoder for a given mode and channel count.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_custom_decoder_init(
    st: *mut CELTDecoder,
    mode: *const CELTMode,
    channels: c_int,
) -> c_int {
    unsafe {
        if channels < 0 || channels > 2 {
            return OPUS_BAD_ARG;
        }
        if st.is_null() {
            return OPUS_ALLOC_FAIL;
        }

        // OPUS_CLEAR: zero the entire allocated block
        let total_size = opus_custom_decoder_get_size(mode, channels) as usize;
        std::ptr::write_bytes(st as *mut u8, 0, total_size);

        (*st).mode = mode;
        (*st).overlap = (*mode).overlap;
        (*st).channels = channels;
        (*st).stream_channels = channels;

        (*st).downsample = 1;
        (*st).start = 0;
        (*st).end = (*mode).eff_ebands;
        (*st).signalling = 1;

        (*st).loss_count = 0;

        // Inline OPUS_RESET_STATE logic: clear from rng onward, init oldLogE/oldLogE2
        celt_decoder_reset(st);

        OPUS_OK
    }
}

/// Reset the decoder state (everything from `rng` onward).
///
/// This is the logic from `opus_custom_decoder_ctl(..., OPUS_RESET_STATE)`.
/// Factored out so both `opus_custom_decoder_init` and the ctl handler can call it.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn celt_decoder_reset(st: *mut CELTDecoder) {
    unsafe {
        let mode = (*st).mode;
        let channels = (*st).channels;
        let nb_ebands = (*mode).nb_ebands;

        // Compute pointer offsets for trailing arrays
        let lpc = ((*st)._decode_mem.as_mut_ptr()).add(((DECODE_BUFFER_SIZE + (*st).overlap) * channels) as usize)
            as *mut OpusVal16;
        let old_band_e = lpc.add((channels * LPC_ORDER) as usize);
        let old_log_e = old_band_e.add((2 * nb_ebands) as usize);
        let old_log_e2 = old_log_e.add((2 * nb_ebands) as usize);

        // Clear from rng to end of allocated block
        let reset_start = &mut (*st).rng as *mut u32 as *mut u8;
        let struct_start = st as *mut u8;
        let total_size = opus_custom_decoder_get_size(mode, channels) as usize;
        let offset = reset_start.offset_from(struct_start) as usize;
        std::ptr::write_bytes(reset_start, 0, total_size - offset);

        // Initialise oldLogE and oldLogE2 to -28 dB
        let init_val = -qconst16(28.0, DB_SHIFT);
        for i in 0..(2 * nb_ebands) as usize {
            *old_log_e.add(i) = init_val;
            *old_log_e2.add(i) = init_val;
        }
    }
}

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

// -- opus_strerror / opus_get_version_string --

static ERROR_STRINGS: [&[u8]; 8] = [
    b"success\0",
    b"invalid argument\0",
    b"buffer too small\0",
    b"internal error\0",
    b"corrupted stream\0",
    b"request not implemented\0",
    b"invalid state\0",
    b"memory allocation failed\0",
];

static UNKNOWN_ERROR: &[u8] = b"unknown error\0";

#[unsafe(no_mangle)]
pub extern "C" fn opus_strerror(error: c_int) -> *const std::os::raw::c_char {
    if error > 0 || error < -7 {
        UNKNOWN_ERROR.as_ptr() as *const std::os::raw::c_char
    } else {
        ERROR_STRINGS[(-error) as usize].as_ptr() as *const std::os::raw::c_char
    }
}

static VERSION_STRING: &[u8] = b"libopus 1.0.0\0";

#[unsafe(no_mangle)]
pub extern "C" fn opus_get_version_string() -> *const std::os::raw::c_char {
    VERSION_STRING.as_ptr() as *const std::os::raw::c_char
}
