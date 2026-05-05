// Translated from c/celt/celt.c (RFC 6716).
//
// CELT decoder core: helper functions and the main decode entry point.
// Functions are translated incrementally, innermost helpers first.

use std::os::raw::c_int;

use crate::arch::*;
use crate::bands::{SPREAD_NORMAL, anti_collapse, celt_lcg_rand, denormalise_bands, quant_all_bands};
use crate::celt_lpc::{_celt_autocorr, _celt_lpc, celt_fir, celt_iir};
use crate::entcode::{BITRES, ec_ctx, ec_tell_frac};
use crate::entdec::{ec_dec_bit_logp, ec_dec_bits, ec_dec_icdf, ec_dec_init, ec_dec_uint, ec_tell};
use crate::mdct::{MdctLookup, clt_mdct_backward};
use crate::modes::{CELTMode, opus_custom_mode_create};
use crate::pitch::{pitch_downsample, pitch_search};
use crate::quant_bands::{log2amp, unquant_coarse_energy, unquant_energy_finalise, unquant_fine_energy};
use crate::rate::compute_allocation;
use crate::vq::renormalise_vector;

// -- Constants --

const SIG_SHIFT: c_int = 12;

#[cfg(not(feature = "fixed-point"))]
const CELT_SIG_SCALE: f32 = 32768.0;

pub const DECODE_BUFFER_SIZE: c_int = 2048;
const MAX_PERIOD: c_int = 1024;
const LPC_ORDER: c_int = 24;
const DB_SHIFT: c_int = 10;

// -- Opus error codes (from opus_defines.h) --

const OPUS_OK: c_int = 0;
const OPUS_BAD_ARG: c_int = -1;
const OPUS_INTERNAL_ERROR: c_int = -3;
const OPUS_ALLOC_FAIL: c_int = -7;
const OPUS_RESET_STATE: c_int = 4028;

const COMBFILTER_MINPERIOD: c_int = 15;

// -- OpusCustomDecoder (CELTDecoder) struct --

// Standard Opus mode constants for fixed-size arrays.
// Without CUSTOM_MODES, overlap=120 and nbEBands=21 are invariant.
const OVERLAP: c_int = 120;
const NB_EBANDS: c_int = 21;
const MAX_CHANNELS: c_int = 2;

// Derived sizes for the trailing arrays (always sized for 2 channels).
const DECODE_MEM_SIZE: usize = (MAX_CHANNELS * (DECODE_BUFFER_SIZE + OVERLAP)) as usize;
const LPC_SIZE: usize = (MAX_CHANNELS * LPC_ORDER) as usize;
const BAND_E_SIZE: usize = (2 * NB_EBANDS) as usize;

/// CELT decoder state.
///
/// The C version uses a flexible array member (`_decode_mem[1]`) with
/// trailing pointer-arithmetic arrays. This Rust version makes all
/// arrays fixed-size, always allocated for 2 channels (the maximum).
/// Mono decoders simply use the first portion of each array.
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

    // Trailing arrays — previously accessed via pointer arithmetic
    // from _decode_mem. Now fixed-size, always sized for 2 channels.
    pub decode_mem: [CeltSig; DECODE_MEM_SIZE],
    pub lpc: [OpusVal16; LPC_SIZE],
    pub old_band_e: [OpusVal16; BAND_E_SIZE],
    pub old_log_e: [OpusVal16; BAND_E_SIZE],
    pub old_log_e2: [OpusVal16; BAND_E_SIZE],
    pub background_log_e: [OpusVal16; BAND_E_SIZE],
}

/// Type alias matching `typedef struct OpusCustomDecoder CELTDecoder`.
pub type CELTDecoder = OpusCustomDecoder;

// -- Decoder size and init functions --

/// Return the size in bytes of a CELT decoder for the standard Opus mode.
pub fn celt_decoder_get_size(_channels: c_int) -> c_int {
    std::mem::size_of::<OpusCustomDecoder>() as c_int
}

/// Return the size in bytes of a CELT decoder for a given mode.
#[unsafe(no_mangle)]
pub extern "C" fn opus_custom_decoder_get_size(_mode: *const CELTMode, _channels: c_int) -> c_int {
    std::mem::size_of::<OpusCustomDecoder>() as c_int
}

/// Initialise a CELT decoder for the standard Opus mode at the given sample rate.
pub unsafe fn celt_decoder_init(st: *mut CELTDecoder, sampling_rate: i32, channels: c_int) -> c_int {
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
pub unsafe extern "C" fn opus_custom_decoder_init(st: *mut CELTDecoder, mode: *const CELTMode, channels: c_int) -> c_int {
    unsafe {
        if channels < 0 || channels > 2 {
            return OPUS_BAD_ARG;
        }
        if st.is_null() {
            return OPUS_ALLOC_FAIL;
        }

        // Zero the entire struct
        std::ptr::write_bytes(st as *mut u8, 0, std::mem::size_of::<OpusCustomDecoder>());

        (*st).mode = mode;
        (*st).overlap = (*mode).overlap;
        (*st).channels = channels;
        (*st).stream_channels = channels;

        (*st).downsample = 1;
        (*st).start = 0;
        (*st).end = (*mode).eff_ebands;
        (*st).signalling = 1;

        (*st).loss_count = 0;

        celt_decoder_reset(st);

        OPUS_OK
    }
}

/// Reset the decoder state (everything from `rng` onward).
///
/// This is the logic from `opus_custom_decoder_ctl(..., OPUS_RESET_STATE)`.
/// Factored out so both `opus_custom_decoder_init` and the ctl handler can call it.
pub unsafe fn celt_decoder_reset(st: *mut CELTDecoder) {
    unsafe {
        // Clear from rng to end of struct
        let reset_start = &mut (*st).rng as *mut u32 as *mut u8;
        let struct_start = st as *mut u8;
        let struct_size = std::mem::size_of::<OpusCustomDecoder>();
        let offset = reset_start.offset_from(struct_start) as usize;
        std::ptr::write_bytes(reset_start, 0, struct_size - offset);

        // Initialise oldLogE and oldLogE2 to -28 dB
        let init_val = -qconst16(28.0, DB_SHIFT);
        for i in 0..BAND_E_SIZE {
            (*st).old_log_e[i] = init_val;
            (*st).old_log_e2[i] = init_val;
        }
    }
}

// -- celt_decode_lost (packet loss concealment) --

/// Packet loss concealment: generate audio when no packet is available.
///
/// Uses noise-based PLC after 5+ consecutive losses (or if start!=0),
/// otherwise pitch-based PLC with LPC synthesis.
pub unsafe fn celt_decode_lost(st: *mut CELTDecoder, pcm: *mut OpusVal16, n: c_int, lm: c_int) {
    unsafe {
        let mut c: c_int;
        let pitch_index: c_int;
        let overlap = (*(*st).mode).overlap;
        let mut fade: OpusVal16 = Q15ONE;
        let cc = (*st).channels;
        let mut offset: c_int;
        let mut out_mem: [*mut CeltSig; 2] = [std::ptr::null_mut(); 2];
        let mut decode_mem: [*mut CeltSig; 2] = [std::ptr::null_mut(); 2];
        let mut overlap_mem: [*mut CeltSig; 2] = [std::ptr::null_mut(); 2];
        let mut out_syn: [*mut OpusVal32; 2] = [std::ptr::null_mut(); 2];

        c = 0;
        loop {
            decode_mem[c as usize] = (*st).decode_mem.as_mut_ptr().add((c * (DECODE_BUFFER_SIZE + (*st).overlap)) as usize);
            out_mem[c as usize] = decode_mem[c as usize].add((DECODE_BUFFER_SIZE - MAX_PERIOD) as usize);
            overlap_mem[c as usize] = decode_mem[c as usize].add(DECODE_BUFFER_SIZE as usize);
            c += 1;
            if c >= cc {
                break;
            }
        }
        let lpc = (*st).lpc.as_mut_ptr();
        let old_band_e = (*st).old_band_e.as_mut_ptr();
        let background_log_e = (*st).background_log_e.as_mut_ptr();

        out_syn[0] = out_mem[0].add((MAX_PERIOD - n) as usize);
        if cc == 2 {
            out_syn[1] = out_mem[1].add((MAX_PERIOD - n) as usize);
        }

        let len = n + (*(*st).mode).overlap;

        if (*st).loss_count >= 5 || (*st).start != 0 {
            // Noise-based PLC/CNG
            let mut seed: u32;

            let mut eff_end = (*st).end;
            if eff_end > (*(*st).mode).eff_ebands {
                eff_end = (*(*st).mode).eff_ebands;
            }

            let mut freq = vec![0 as CeltSig; (cc * n) as usize];
            let mut x = vec![0 as CeltNorm; (cc * n) as usize];
            let mut band_e = vec![0 as CeltEner; ((*(*st).mode).nb_ebands * cc) as usize];

            if (*st).loss_count >= 5 {
                log2amp((*st).mode, (*st).start, (*st).end, band_e.as_mut_ptr(), background_log_e, cc);
            } else {
                // Energy decay
                let decay: OpusVal16 = if (*st).loss_count == 0 { qconst16(1.5, DB_SHIFT) } else { qconst16(0.5, DB_SHIFT) };
                c = 0;
                loop {
                    for i in (*st).start..(*st).end {
                        *old_band_e.add((c * (*(*st).mode).nb_ebands + i) as usize) -= decay;
                    }
                    c += 1;
                    if c >= cc {
                        break;
                    }
                }
                log2amp((*st).mode, (*st).start, (*st).end, band_e.as_mut_ptr(), old_band_e, cc);
            }
            seed = (*st).rng;
            for c in 0..cc {
                for i in 0..(*(*(*st).mode).ebands.add((*st).start as usize) as c_int) << lm {
                    x[(c * n + i) as usize] = 0 as CeltNorm;
                }
                for i in (*st).start..(*(*st).mode).eff_ebands {
                    let boffs = (n * c + ((*(*(*st).mode).ebands.add(i as usize)) << lm) as c_int) as usize;
                    let blen = (((*(*(*st).mode).ebands.add(i as usize + 1)) - (*(*(*st).mode).ebands.add(i as usize))) << lm)
                        as c_int;
                    for j in 0..blen as usize {
                        seed = celt_lcg_rand(seed);
                        x[boffs + j] = (seed as i32 >> 20) as CeltNorm;
                    }
                    renormalise_vector(x.as_mut_ptr().add(boffs), blen, Q15ONE);
                }
                for i in ((*(*(*st).mode).ebands.add((*st).end as usize)) << lm) as c_int..n {
                    x[(c * n + i) as usize] = 0 as CeltNorm;
                }
            }
            (*st).rng = seed;

            denormalise_bands(
                (*st).mode,
                x.as_mut_ptr(),
                freq.as_mut_ptr(),
                band_e.as_mut_ptr(),
                (*(*st).mode).eff_ebands,
                cc,
                1 << lm,
            );

            c = 0;
            loop {
                for i in 0..((*(*(*st).mode).ebands.add((*st).start as usize)) << lm) as c_int {
                    freq[(c * n + i) as usize] = 0 as CeltSig;
                }
                c += 1;
                if c >= cc {
                    break;
                }
            }
            c = 0;
            loop {
                let mut bound = ((*(*(*st).mode).ebands.add(eff_end as usize)) << lm) as c_int;
                if (*st).downsample != 1 {
                    bound = bound.min(n / (*st).downsample);
                }
                for i in bound..n {
                    freq[(c * n + i) as usize] = 0 as CeltSig;
                }
                c += 1;
                if c >= cc {
                    break;
                }
            }
            compute_inv_mdcts((*st).mode, 0, freq.as_mut_ptr(), out_syn.as_mut_ptr(), overlap_mem.as_mut_ptr(), cc, lm);
        } else {
            // Pitch-based PLC
            if (*st).loss_count == 0 {
                let mut pitch_buf = vec![0 as OpusVal16; (DECODE_BUFFER_SIZE >> 1) as usize];
                let poffset: c_int = 720;
                pitch_downsample(decode_mem.as_ptr() as *const *const CeltSig, pitch_buf.as_mut_ptr(), DECODE_BUFFER_SIZE, cc);
                let mut pi: c_int = 0;
                pitch_search(
                    pitch_buf.as_mut_ptr().add((poffset >> 1) as usize),
                    pitch_buf.as_mut_ptr(),
                    DECODE_BUFFER_SIZE - poffset,
                    poffset - 100,
                    &mut pi,
                );
                pitch_index = poffset - pi;
                (*st).last_pitch_index = pitch_index;
            } else {
                pitch_index = (*st).last_pitch_index;
                fade = qconst16(0.8, 15);
            }

            c = 0;
            loop {
                let mut exc = [0 as OpusVal16; MAX_PERIOD as usize];
                let mut ac = [0 as OpusVal32; (LPC_ORDER + 1) as usize];
                let mut decay: OpusVal16;
                let mut s1: OpusVal32 = 0 as OpusVal32;
                let mut mem = [0 as OpusVal16; LPC_ORDER as usize];

                let mut e = vec![0 as OpusVal32; (MAX_PERIOD + 2 * (*(*st).mode).overlap) as usize];

                offset = MAX_PERIOD - pitch_index;
                for i in 0..MAX_PERIOD as usize {
                    exc[i] = round16(*out_mem[c as usize].add(i), SIG_SHIFT);
                }

                if (*st).loss_count == 0 {
                    _celt_autocorr(
                        exc.as_ptr(),
                        ac.as_mut_ptr(),
                        (*(*st).mode).window,
                        (*(*st).mode).overlap,
                        LPC_ORDER,
                        MAX_PERIOD,
                    );

                    // Noise floor -40 dB
                    #[cfg(feature = "fixed-point")]
                    {
                        ac[0] += shr32(ac[0], 13);
                    }
                    #[cfg(not(feature = "fixed-point"))]
                    {
                        ac[0] *= 1.0001;
                    }
                    // Lag windowing
                    for i in 1..=LPC_ORDER {
                        #[cfg(feature = "fixed-point")]
                        {
                            ac[i as usize] -= mult16_32_q15((2 * i * i) as OpusVal16, ac[i as usize]);
                        }
                        #[cfg(not(feature = "fixed-point"))]
                        {
                            ac[i as usize] -= ac[i as usize] * (0.008 * i as f32) * (0.008 * i as f32);
                        }
                    }

                    _celt_lpc(lpc.add((c * LPC_ORDER) as usize), ac.as_ptr(), LPC_ORDER);
                }
                for i in 0..LPC_ORDER as usize {
                    mem[i] = round16(*out_mem[c as usize].add(MAX_PERIOD as usize - 1 - i), SIG_SHIFT);
                }
                celt_fir(
                    exc.as_mut_ptr() as *mut OpusVal16,
                    lpc.add((c * LPC_ORDER) as usize),
                    exc.as_mut_ptr() as *mut OpusVal16,
                    MAX_PERIOD,
                    LPC_ORDER,
                    mem.as_mut_ptr(),
                );
                // Check if the waveform is decaying (and if so how fast)
                {
                    let mut e1: OpusVal32 = 1 as OpusVal32;
                    let mut e2: OpusVal32 = 1 as OpusVal32;
                    let period: c_int;
                    if pitch_index <= MAX_PERIOD / 2 {
                        period = pitch_index;
                    } else {
                        period = MAX_PERIOD / 2;
                    }
                    for i in 0..period {
                        e1 += shr32(
                            mult16_16(exc[(MAX_PERIOD - period + i) as usize], exc[(MAX_PERIOD - period + i) as usize]),
                            8,
                        );
                        e2 += shr32(
                            mult16_16(exc[(MAX_PERIOD - 2 * period + i) as usize], exc[(MAX_PERIOD - 2 * period + i) as usize]),
                            8,
                        );
                    }
                    if e1 > e2 {
                        e1 = e2;
                    }
                    decay = celt_sqrt(frac_div32(shr32(e1, 1), e2)) as OpusVal16;
                }

                // Copy excitation, taking decay into account
                for i in 0..len + (*(*st).mode).overlap {
                    if offset + i >= MAX_PERIOD {
                        offset -= pitch_index;
                        decay = mult16_16_q15(decay, decay) as OpusVal16;
                    }
                    e[i as usize] = shl32(extend32(mult16_16_q15(decay, exc[(offset + i) as usize]) as OpusVal16), SIG_SHIFT);
                    let tmp: OpusVal16 = round16(*out_mem[c as usize].add((offset + i) as usize), SIG_SHIFT);
                    s1 += shr32(mult16_16(tmp, tmp), 8);
                }
                for i in 0..LPC_ORDER as usize {
                    mem[i] = round16(*out_mem[c as usize].add(MAX_PERIOD as usize - 1 - i), SIG_SHIFT);
                }
                for i in 0..(len + (*(*st).mode).overlap) as usize {
                    e[i] = mult16_32_q15(fade, e[i]);
                }
                celt_iir(
                    e.as_mut_ptr(),
                    lpc.add((c * LPC_ORDER) as usize),
                    e.as_mut_ptr(),
                    len + (*(*st).mode).overlap,
                    LPC_ORDER,
                    mem.as_mut_ptr(),
                );

                {
                    let mut s2: OpusVal32 = 0 as OpusVal32;
                    for i in 0..(len + overlap) as usize {
                        let tmp: OpusVal16 = round16(e[i], SIG_SHIFT);
                        s2 += shr32(mult16_16(tmp, tmp), 8);
                    }
                    // This checks for an "explosion" in the synthesis
                    #[cfg(feature = "fixed-point")]
                    let explosion = !(s1 > shr32(s2, 2));
                    #[cfg(not(feature = "fixed-point"))]
                    let explosion = !(s1 > 0.2 * s2);

                    if explosion {
                        for i in 0..(len + overlap) as usize {
                            e[i] = 0 as OpusVal32;
                        }
                    } else if s1 < s2 {
                        let ratio: OpusVal16 =
                            celt_sqrt(frac_div32(shr32(s1, 1) + 1 as OpusVal32, s2 + 1 as OpusVal32)) as OpusVal16;
                        for i in 0..(len + overlap) as usize {
                            e[i] = mult16_32_q15(ratio, e[i]);
                        }
                    }
                }

                // Apply post-filter to the MDCT overlap of the previous frame
                comb_filter(
                    out_mem[c as usize].add(MAX_PERIOD as usize),
                    out_mem[c as usize].add(MAX_PERIOD as usize),
                    (*st).postfilter_period,
                    (*st).postfilter_period,
                    (*st).overlap,
                    (*st).postfilter_gain,
                    (*st).postfilter_gain,
                    (*st).postfilter_tapset,
                    (*st).postfilter_tapset,
                    std::ptr::null(),
                    0,
                );

                for i in 0..(MAX_PERIOD + (*(*st).mode).overlap - n) as usize {
                    *out_mem[c as usize].add(i) = *out_mem[c as usize].add(n as usize + i);
                }

                // Apply TDAC to the concealed audio so that it blends with the
                // previous and next frames
                for i in 0..overlap / 2 {
                    let tmp: OpusVal32 =
                        mult16_32_q15(*(*(*st).mode).window.add(i as usize), e[(n + overlap - 1 - i) as usize])
                            + mult16_32_q15(*(*(*st).mode).window.add((overlap - i - 1) as usize), e[(n + i) as usize]);
                    *out_mem[c as usize].add((MAX_PERIOD + i) as usize) =
                        mult16_32_q15(*(*(*st).mode).window.add((overlap - i - 1) as usize), tmp);
                    *out_mem[c as usize].add((MAX_PERIOD + overlap - i - 1) as usize) =
                        mult16_32_q15(*(*(*st).mode).window.add(i as usize), tmp);
                }
                for i in 0..n {
                    *out_mem[c as usize].add((MAX_PERIOD - n + i) as usize) = e[i as usize];
                }

                // Apply pre-filter to the MDCT overlap for the next frame
                comb_filter(
                    e.as_mut_ptr(),
                    out_mem[c as usize].add(MAX_PERIOD as usize),
                    (*st).postfilter_period,
                    (*st).postfilter_period,
                    (*st).overlap,
                    -(*st).postfilter_gain,
                    -(*st).postfilter_gain,
                    (*st).postfilter_tapset,
                    (*st).postfilter_tapset,
                    std::ptr::null(),
                    0,
                );
                for i in 0..overlap as usize {
                    *out_mem[c as usize].add(MAX_PERIOD as usize + i) = e[i];
                }

                c += 1;
                if c >= cc {
                    break;
                }
            }
        }

        deemphasis(
            out_syn.as_mut_ptr(),
            pcm,
            n,
            cc,
            (*st).downsample,
            (*(*st).mode).preemph.as_ptr(),
            (*st).preemph_mem_d.as_mut_ptr(),
        );

        (*st).loss_count += 1;
    }
}

// -- ICDFs (used by celt_decode_with_ec) --

static TRIM_ICDF: [u8; 11] = [126, 124, 119, 109, 87, 41, 19, 9, 4, 2, 0];
static SPREAD_ICDF: [u8; 4] = [25, 23, 2, 0];
static TAPSET_ICDF: [u8; 3] = [2, 1, 0];

// -- celt_decode_with_ec (main decoder entry) --

/// Main CELT decode function.
///
/// Decodes a CELT frame from the bitstream into PCM samples.
/// If data is NULL or len <= 1, runs packet loss concealment instead.
pub unsafe fn celt_decode_with_ec(
    st: *mut CELTDecoder,
    data: *const u8,
    len: c_int,
    pcm: *mut OpusVal16,
    frame_size: c_int,
    dec: *mut ec_ctx,
) -> c_int {
    unsafe {
        let mut c: c_int;
        let cc = (*st).channels;
        let mut out_mem: [*mut CeltSig; 2] = [std::ptr::null_mut(); 2];
        let mut decode_mem: [*mut CeltSig; 2] = [std::ptr::null_mut(); 2];
        let mut overlap_mem: [*mut CeltSig; 2] = [std::ptr::null_mut(); 2];
        let mut out_syn: [*mut CeltSig; 2] = [std::ptr::null_mut(); 2];

        let mut frame_size = frame_size;
        frame_size *= (*st).downsample;

        c = 0;
        loop {
            decode_mem[c as usize] = (*st).decode_mem.as_mut_ptr().add((c * (DECODE_BUFFER_SIZE + (*st).overlap)) as usize);
            out_mem[c as usize] = decode_mem[c as usize].add((DECODE_BUFFER_SIZE - MAX_PERIOD) as usize);
            overlap_mem[c as usize] = decode_mem[c as usize].add(DECODE_BUFFER_SIZE as usize);
            c += 1;
            if c >= cc {
                break;
            }
        }
        let old_band_e = (*st).old_band_e.as_mut_ptr();
        let old_log_e = (*st).old_log_e.as_mut_ptr();
        let old_log_e2 = (*st).old_log_e2.as_mut_ptr();
        let background_log_e = (*st).background_log_e.as_mut_ptr();

        // Without CUSTOM_MODES, find LM from frame_size
        let mut lm: c_int = 0;
        while lm <= (*(*st).mode).max_lm {
            if (*(*st).mode).short_mdct_size << lm == frame_size {
                break;
            }
            lm += 1;
        }
        if lm > (*(*st).mode).max_lm {
            return OPUS_BAD_ARG;
        }
        let m: c_int = 1 << lm;

        let len = len;
        if len < 0 || len > 1275 || pcm.is_null() {
            return OPUS_BAD_ARG;
        }

        let n: c_int = m * (*(*st).mode).short_mdct_size;

        let mut eff_end = (*st).end;
        if eff_end > (*(*st).mode).eff_ebands {
            eff_end = (*(*st).mode).eff_ebands;
        }

        let c_channels = (*st).stream_channels;
        let mut freq = vec![0 as CeltSig; (cc.max(c_channels) * n) as usize];
        let mut x = vec![0 as CeltNorm; (c_channels * n) as usize];
        let mut band_e = vec![0 as CeltEner; ((*(*st).mode).nb_ebands * c_channels) as usize];

        c = 0;
        loop {
            for ii in 0..m * *(*(*st).mode).ebands.add((*st).start as usize) as c_int {
                x[(c * n + ii) as usize] = 0 as CeltNorm;
            }
            c += 1;
            if c >= c_channels {
                break;
            }
        }
        c = 0;
        loop {
            for ii in m * *(*(*st).mode).ebands.add(eff_end as usize) as c_int..n {
                x[(c * n + ii) as usize] = 0 as CeltNorm;
            }
            c += 1;
            if c >= c_channels {
                break;
            }
        }

        if data.is_null() || len <= 1 {
            celt_decode_lost(st, pcm, n, lm);
            return frame_size / (*st).downsample;
        }

        let mut _dec: ec_ctx = std::mem::zeroed();
        let dec = if dec.is_null() {
            ec_dec_init(&mut _dec, data as *mut u8, len as u32);
            &mut _dec as *mut ec_ctx
        } else {
            dec
        };

        if c_channels == 1 {
            for ii in 0..(*(*st).mode).nb_ebands {
                *old_band_e.add(ii as usize) =
                    max16(*old_band_e.add(ii as usize), *old_band_e.add(((*(*st).mode).nb_ebands + ii) as usize));
            }
        }

        let mut total_bits: i32 = len * 8;
        let mut tell: i32 = ec_tell(&*dec) as i32;

        let silence: c_int;
        if tell >= total_bits {
            silence = 1;
        } else if tell == 1 {
            silence = ec_dec_bit_logp(dec, 15);
        } else {
            silence = 0;
        }
        if silence != 0 {
            tell = len * 8;
            (*dec).nbits_total += tell - ec_tell(&*dec) as i32;
        }

        let mut postfilter_gain: OpusVal16 = 0 as OpusVal16;
        let mut postfilter_pitch: c_int = 0;
        let mut postfilter_tapset: c_int = 0;
        if (*st).start == 0 && tell + 16 <= total_bits {
            if ec_dec_bit_logp(dec, 1) != 0 {
                let octave = ec_dec_uint(dec, 6) as c_int;
                postfilter_pitch = (16 << octave) + ec_dec_bits(dec, (4 + octave) as u32) as c_int - 1;
                let qg = ec_dec_bits(dec, 3) as c_int;
                if ec_tell(&*dec) as i32 + 2 <= total_bits {
                    postfilter_tapset = ec_dec_icdf(dec, TAPSET_ICDF.as_ptr(), 2);
                }
                postfilter_gain = qconst16(0.09375, 15) * (qg + 1) as OpusVal16;
            }
            tell = ec_tell(&*dec) as i32;
        }

        let is_transient: c_int;
        if lm > 0 && tell + 3 <= total_bits {
            is_transient = ec_dec_bit_logp(dec, 3);
            tell = ec_tell(&*dec) as i32;
        } else {
            is_transient = 0;
        }

        let short_blocks: c_int = if is_transient != 0 { m } else { 0 };

        // Decode the global flags (first symbols in the stream)
        let intra_ener: c_int = if tell + 3 <= total_bits { ec_dec_bit_logp(dec, 3) } else { 0 };
        // Get band energies
        unquant_coarse_energy((*st).mode, (*st).start, (*st).end, old_band_e, intra_ener, dec, c_channels, lm);

        let mut tf_res = vec![0i32; (*(*st).mode).nb_ebands as usize];
        tf_decode((*st).start, (*st).end, is_transient, tf_res.as_mut_ptr(), lm, dec);

        tell = ec_tell(&*dec) as i32;
        let mut spread_decision: c_int = SPREAD_NORMAL;
        if tell + 4 <= total_bits {
            spread_decision = ec_dec_icdf(dec, SPREAD_ICDF.as_ptr(), 5);
        }

        let mut pulses = vec![0i32; (*(*st).mode).nb_ebands as usize];
        let mut cap = vec![0i32; (*(*st).mode).nb_ebands as usize];
        let mut offsets = vec![0i32; (*(*st).mode).nb_ebands as usize];
        let mut fine_priority = vec![0i32; (*(*st).mode).nb_ebands as usize];

        init_caps((*st).mode, cap.as_mut_ptr(), lm, c_channels);

        let mut dynalloc_logp: c_int = 6;
        total_bits <<= BITRES;
        tell = ec_tell_frac(dec) as i32;
        for ii in (*st).start..(*st).end {
            let width = (c_channels
                * (*(*(*st).mode).ebands.add(ii as usize + 1) - *(*(*st).mode).ebands.add(ii as usize)) as c_int)
                << lm;
            let quanta = (width << BITRES).min((6i32 << BITRES).max(width));
            let mut dynalloc_loop_logp = dynalloc_logp;
            let mut boost: c_int = 0;
            while tell + (dynalloc_loop_logp << BITRES) < total_bits && boost < cap[ii as usize] {
                let flag = ec_dec_bit_logp(dec, dynalloc_loop_logp as u32);
                tell = ec_tell_frac(dec) as i32;
                if flag == 0 {
                    break;
                }
                boost += quanta;
                total_bits -= quanta;
                dynalloc_loop_logp = 1;
            }
            offsets[ii as usize] = boost;
            if boost > 0 {
                dynalloc_logp = 2i32.max(dynalloc_logp - 1);
            }
        }

        let mut fine_quant = vec![0i32; (*(*st).mode).nb_ebands as usize];
        let alloc_trim: c_int = if tell + (6 << BITRES) <= total_bits { ec_dec_icdf(dec, TRIM_ICDF.as_ptr(), 7) } else { 5 };

        let mut bits: i32 = ((len as i32 * 8) << BITRES) - ec_tell_frac(dec) as i32 - 1;
        let anti_collapse_rsv: c_int =
            if is_transient != 0 && lm >= 2 && bits >= ((lm + 2) << BITRES) { 1 << BITRES } else { 0 };
        bits -= anti_collapse_rsv;
        let mut intensity: c_int = 0;
        let mut dual_stereo: c_int = 0;
        let mut balance: i32 = 0;
        let coded_bands = compute_allocation(
            (*st).mode,
            (*st).start,
            (*st).end,
            offsets.as_mut_ptr(),
            cap.as_mut_ptr(),
            alloc_trim,
            &mut intensity,
            &mut dual_stereo,
            bits,
            &mut balance,
            pulses.as_mut_ptr(),
            fine_quant.as_mut_ptr(),
            fine_priority.as_mut_ptr(),
            c_channels,
            lm,
            dec,
            0,
            0,
        );

        unquant_fine_energy((*st).mode, (*st).start, (*st).end, old_band_e, fine_quant.as_mut_ptr(), dec, c_channels);

        // Decode fixed codebook
        let mut collapse_masks = vec![0u8; (c_channels * (*(*st).mode).nb_ebands) as usize];
        quant_all_bands(
            0,
            (*st).mode,
            (*st).start,
            (*st).end,
            x.as_mut_ptr(),
            if c_channels == 2 { x.as_mut_ptr().add(n as usize) } else { std::ptr::null_mut() },
            collapse_masks.as_mut_ptr(),
            pulses.as_mut_ptr(),
            short_blocks,
            spread_decision,
            dual_stereo,
            intensity,
            tf_res.as_mut_ptr(),
            len * (8 << BITRES) - anti_collapse_rsv,
            balance,
            dec,
            lm,
            coded_bands,
            &mut (*st).rng,
        );

        let mut anti_collapse_on: c_int = 0;
        if anti_collapse_rsv > 0 {
            anti_collapse_on = ec_dec_bits(dec, 1) as c_int;
        }

        unquant_energy_finalise(
            (*st).mode,
            (*st).start,
            (*st).end,
            old_band_e,
            fine_quant.as_mut_ptr(),
            fine_priority.as_mut_ptr(),
            len * 8 - ec_tell(&*dec) as c_int,
            dec,
            c_channels,
        );

        if anti_collapse_on != 0 {
            anti_collapse(
                (*st).mode,
                x.as_mut_ptr(),
                collapse_masks.as_mut_ptr(),
                lm,
                c_channels,
                n,
                (*st).start,
                (*st).end,
                old_band_e,
                old_log_e,
                old_log_e2,
                pulses.as_mut_ptr(),
                (*st).rng,
            );
        }

        log2amp((*st).mode, (*st).start, (*st).end, band_e.as_mut_ptr(), old_band_e, c_channels);

        if silence != 0 {
            for ii in 0..(c_channels * (*(*st).mode).nb_ebands) as usize {
                band_e[ii] = 0 as CeltEner;
                *old_band_e.add(ii) = -qconst16(28.0, DB_SHIFT);
            }
        }
        // Synthesis
        denormalise_bands((*st).mode, x.as_mut_ptr(), freq.as_mut_ptr(), band_e.as_mut_ptr(), eff_end, c_channels, m);

        // OPUS_MOVE: memmove decode_mem forward by N
        std::ptr::copy(decode_mem[0].add(n as usize), decode_mem[0], (DECODE_BUFFER_SIZE - n) as usize);
        if cc == 2 {
            std::ptr::copy(decode_mem[1].add(n as usize), decode_mem[1], (DECODE_BUFFER_SIZE - n) as usize);
        }

        c = 0;
        loop {
            for ii in 0..m * *(*(*st).mode).ebands.add((*st).start as usize) as c_int {
                freq[(c * n + ii) as usize] = 0 as CeltSig;
            }
            c += 1;
            if c >= c_channels {
                break;
            }
        }
        c = 0;
        loop {
            let mut bound = m * *(*(*st).mode).ebands.add(eff_end as usize) as c_int;
            if (*st).downsample != 1 {
                bound = bound.min(n / (*st).downsample);
            }
            for ii in bound..n {
                freq[(c * n + ii) as usize] = 0 as CeltSig;
            }
            c += 1;
            if c >= c_channels {
                break;
            }
        }

        out_syn[0] = out_mem[0].add((MAX_PERIOD - n) as usize);
        if cc == 2 {
            out_syn[1] = out_mem[1].add((MAX_PERIOD - n) as usize);
        }

        if cc == 2 && c_channels == 1 {
            for ii in 0..n as usize {
                freq[n as usize + ii] = freq[ii];
            }
        }
        if cc == 1 && c_channels == 2 {
            for ii in 0..n as usize {
                freq[ii] = half32(add32(freq[ii], freq[n as usize + ii]));
            }
        }

        // Compute inverse MDCTs
        compute_inv_mdcts((*st).mode, short_blocks, freq.as_mut_ptr(), out_syn.as_mut_ptr(), overlap_mem.as_mut_ptr(), cc, lm);

        c = 0;
        loop {
            (*st).postfilter_period = (*st).postfilter_period.max(COMBFILTER_MINPERIOD);
            (*st).postfilter_period_old = (*st).postfilter_period_old.max(COMBFILTER_MINPERIOD);
            comb_filter(
                out_syn[c as usize],
                out_syn[c as usize],
                (*st).postfilter_period_old,
                (*st).postfilter_period,
                (*(*st).mode).short_mdct_size,
                (*st).postfilter_gain_old,
                (*st).postfilter_gain,
                (*st).postfilter_tapset_old,
                (*st).postfilter_tapset,
                (*(*st).mode).window,
                (*st).overlap,
            );
            if lm != 0 {
                comb_filter(
                    out_syn[c as usize].add((*(*st).mode).short_mdct_size as usize),
                    out_syn[c as usize].add((*(*st).mode).short_mdct_size as usize),
                    (*st).postfilter_period,
                    postfilter_pitch,
                    n - (*(*st).mode).short_mdct_size,
                    (*st).postfilter_gain,
                    postfilter_gain,
                    (*st).postfilter_tapset,
                    postfilter_tapset,
                    (*(*st).mode).window,
                    (*(*st).mode).overlap,
                );
            }
            c += 1;
            if c >= cc {
                break;
            }
        }
        (*st).postfilter_period_old = (*st).postfilter_period;
        (*st).postfilter_gain_old = (*st).postfilter_gain;
        (*st).postfilter_tapset_old = (*st).postfilter_tapset;
        (*st).postfilter_period = postfilter_pitch;
        (*st).postfilter_gain = postfilter_gain;
        (*st).postfilter_tapset = postfilter_tapset;
        if lm != 0 {
            (*st).postfilter_period_old = (*st).postfilter_period;
            (*st).postfilter_gain_old = (*st).postfilter_gain;
            (*st).postfilter_tapset_old = (*st).postfilter_tapset;
        }

        if c_channels == 1 {
            for ii in 0..(*(*st).mode).nb_ebands {
                *old_band_e.add(((*(*st).mode).nb_ebands + ii) as usize) = *old_band_e.add(ii as usize);
            }
        }

        // In case start or end were to change
        if is_transient == 0 {
            for ii in 0..2 * (*(*st).mode).nb_ebands {
                *old_log_e2.add(ii as usize) = *old_log_e.add(ii as usize);
            }
            for ii in 0..2 * (*(*st).mode).nb_ebands {
                *old_log_e.add(ii as usize) = *old_band_e.add(ii as usize);
            }
            for ii in 0..2 * (*(*st).mode).nb_ebands {
                *background_log_e.add(ii as usize) = min16(
                    *background_log_e.add(ii as usize) + m as OpusVal16 * qconst16(0.001, DB_SHIFT),
                    *old_band_e.add(ii as usize),
                );
            }
        } else {
            for ii in 0..2 * (*(*st).mode).nb_ebands {
                *old_log_e.add(ii as usize) = min16(*old_log_e.add(ii as usize), *old_band_e.add(ii as usize));
            }
        }
        c = 0;
        loop {
            for ii in 0..(*st).start {
                *old_band_e.add((c * (*(*st).mode).nb_ebands + ii) as usize) = 0 as OpusVal16;
                *old_log_e.add((c * (*(*st).mode).nb_ebands + ii) as usize) = -qconst16(28.0, DB_SHIFT);
                *old_log_e2.add((c * (*(*st).mode).nb_ebands + ii) as usize) = -qconst16(28.0, DB_SHIFT);
            }
            for ii in (*st).end..(*(*st).mode).nb_ebands {
                *old_band_e.add((c * (*(*st).mode).nb_ebands + ii) as usize) = 0 as OpusVal16;
                *old_log_e.add((c * (*(*st).mode).nb_ebands + ii) as usize) = -qconst16(28.0, DB_SHIFT);
                *old_log_e2.add((c * (*(*st).mode).nb_ebands + ii) as usize) = -qconst16(28.0, DB_SHIFT);
            }
            c += 1;
            if c >= 2 {
                break;
            }
        }
        (*st).rng = (*dec).rng;

        deemphasis(
            out_syn.as_mut_ptr(),
            pcm,
            n,
            cc,
            (*st).downsample,
            (*(*st).mode).preemph.as_ptr(),
            (*st).preemph_mem_d.as_mut_ptr(),
        );
        (*st).loss_count = 0;
        if ec_tell(&*dec) as c_int > 8 * len {
            return OPUS_INTERNAL_ERROR;
        }
        if (*dec).error != 0 {
            (*st).error = 1;
        }
        frame_size / (*st).downsample
    }
}

// -- tf_select_table (used by tf_decode) --

static TF_SELECT_TABLE: [[i8; 8]; 4] =
    [[0, -1, 0, -1, 0, -1, 0, -1], [0, -1, 0, -2, 1, 0, 1, -1], [0, -2, 0, -3, 2, 0, 1, -1], [0, -2, 0, -3, 3, 0, 1, -1]];

// -- Small helper functions --

/// Map sample rate to resampling factor.
/// Only the five standard Opus rates are supported.
pub fn resampling_factor(rate: i32) -> c_int {
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
#[cfg(feature = "fixed-point")]
pub fn sig2word16(x: crate::arch::CeltSig) -> OpusVal16 {
    let x = pshr32(x, SIG_SHIFT);
    let x = max32(x, -32768);
    let x = min32(x, 32767);
    extract16(x)
}

#[cfg(not(feature = "fixed-point"))]
pub fn sig2word16(x: crate::arch::CeltSig) -> OpusVal16 {
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
pub unsafe fn tf_decode(start: c_int, end: c_int, is_transient: c_int, tf_res: *mut c_int, lm: c_int, dec: *mut ec_ctx) {
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
pub unsafe fn init_caps(m: *const CELTMode, cap: *mut c_int, lm: c_int, c: c_int) {
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
pub unsafe fn compute_inv_mdcts(
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
pub unsafe fn deemphasis(
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
pub unsafe fn comb_filter(
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

// -- opus_custom_decoder_ctl --

// Request codes (from opus_defines.h and celt/celt.h)
const OPUS_GET_LOOKAHEAD_REQUEST: c_int = 4027;
const OPUS_GET_FINAL_RANGE_REQUEST: c_int = 4031;
const OPUS_GET_PITCH_REQUEST: c_int = 4033;
const CELT_SET_START_BAND_REQUEST: c_int = 10010;
const CELT_SET_END_BAND_REQUEST: c_int = 10012;
const CELT_GET_AND_CLEAR_ERROR_REQUEST: c_int = 10007;
const CELT_SET_CHANNELS_REQUEST: c_int = 10008;
const CELT_GET_MODE_REQUEST: c_int = 10015;
const CELT_SET_SIGNALLING_REQUEST: c_int = 10016;

/// FFI-safe tagged enum for CELT decoder CTL requests.
///
/// Replaces the C varargs interface. Each variant corresponds to one
/// request code. The discriminant values match the C `*_REQUEST` constants.
///
/// Layout: `#[repr(C, i32)]` gives a C-compatible struct { i32 tag; union payload; }.
#[repr(C, i32)]
pub enum CeltDecCtl {
    SetStartBand(c_int) = CELT_SET_START_BAND_REQUEST,
    SetEndBand(c_int) = CELT_SET_END_BAND_REQUEST,
    SetChannels(c_int) = CELT_SET_CHANNELS_REQUEST,
    SetSignalling(c_int) = CELT_SET_SIGNALLING_REQUEST,
    GetAndClearError(*mut c_int) = CELT_GET_AND_CLEAR_ERROR_REQUEST,
    GetLookahead(*mut c_int) = OPUS_GET_LOOKAHEAD_REQUEST,
    GetPitch(*mut c_int) = OPUS_GET_PITCH_REQUEST,
    GetFinalRange(*mut u32) = OPUS_GET_FINAL_RANGE_REQUEST,
    GetMode(*mut *const CELTMode) = CELT_GET_MODE_REQUEST,
    ResetState = OPUS_RESET_STATE,
}

/// CELT decoder control — enum-based replacement for the C varargs interface.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_custom_decoder_ctl(st: *mut CELTDecoder, request: CeltDecCtl) -> c_int {
    unsafe {
        match request {
            CeltDecCtl::SetStartBand(value) => {
                if value < 0 || value >= (*(*st).mode).nb_ebands {
                    return OPUS_BAD_ARG;
                }
                (*st).start = value;
            }
            CeltDecCtl::SetEndBand(value) => {
                if value < 1 || value > (*(*st).mode).nb_ebands {
                    return OPUS_BAD_ARG;
                }
                (*st).end = value;
            }
            CeltDecCtl::SetChannels(value) => {
                if value < 1 || value > 2 {
                    return OPUS_BAD_ARG;
                }
                (*st).stream_channels = value;
            }
            CeltDecCtl::SetSignalling(value) => {
                (*st).signalling = value;
            }
            CeltDecCtl::GetAndClearError(ptr) => {
                if ptr.is_null() {
                    return OPUS_BAD_ARG;
                }
                *ptr = (*st).error;
                (*st).error = 0;
            }
            CeltDecCtl::GetLookahead(ptr) => {
                if ptr.is_null() {
                    return OPUS_BAD_ARG;
                }
                *ptr = (*st).overlap / (*st).downsample;
            }
            CeltDecCtl::GetPitch(ptr) => {
                if ptr.is_null() {
                    return OPUS_BAD_ARG;
                }
                *ptr = (*st).postfilter_period;
            }
            CeltDecCtl::GetFinalRange(ptr) => {
                if ptr.is_null() {
                    return OPUS_BAD_ARG;
                }
                *ptr = (*st).rng;
            }
            CeltDecCtl::GetMode(ptr) => {
                if ptr.is_null() {
                    return OPUS_BAD_ARG;
                }
                *ptr = (*st).mode;
            }
            CeltDecCtl::ResetState => {
                celt_decoder_reset(st);
            }
        }
        OPUS_OK
    }
}

/// Convenience wrapper matching the old `celt_decoder_ctl` name.
pub unsafe fn celt_decoder_ctl(st: *mut CELTDecoder, request: CeltDecCtl) -> c_int {
    unsafe { opus_custom_decoder_ctl(st, request) }
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
