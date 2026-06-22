// Translated from c/celt/celt.c (RFC 6716).
//
// CELT decoder core: helper functions and the main decode entry point.
// Functions are translated incrementally, innermost helpers first.

use crate::arch::*;
use crate::bands::{SPREAD_NORMAL, anti_collapse, celt_lcg_rand, denormalise_bands, quant_all_bands};
use crate::celt_lpc::{_celt_autocorr, _celt_lpc, celt_fir, celt_iir};
use crate::entcode::{BITRES, EcCtx, ec_tell_frac};
use crate::entdec::{ec_dec_bit_logp, ec_dec_bits, ec_dec_icdf, ec_dec_init, ec_dec_uint, ec_tell};
use crate::error::{OPUS_BAD_ARG, OPUS_INTERNAL_ERROR, OPUS_OK};
use crate::mdct::clt_mdct_backward;
use crate::modes::{CELTMode, celt_mode};
use crate::pitch::{pitch_downsample, pitch_search};
use crate::quant_bands::{log2amp, unquant_coarse_energy, unquant_energy_finalise, unquant_fine_energy};
use crate::rate::compute_allocation;
use crate::util::{OrPanic, zip, zip3, zip4};
use crate::vq::renormalise_vector;

// -- Constants --

const SIG_SHIFT: i32 = 12;

#[cfg(not(feature = "fixed-point"))]
const CELT_SIG_SCALE: f32 = 32768.0;

pub const DECODE_BUFFER_SIZE: i32 = 2048;
const MAX_PERIOD: i32 = 1024;
const LPC_ORDER: i32 = 24;
const DB_SHIFT: i32 = 10;

const COMBFILTER_MINPERIOD: i32 = 15;

// -- OpusCustomDecoder (CELTDecoder) struct --

// Standard Opus mode constants for fixed-size arrays.
// Without CUSTOM_MODES, overlap=120 and nbEBands=21 are invariant.
const OVERLAP: i32 = 120;
const NB_EBANDS: i32 = 21;
const MAX_CHANNELS: i32 = 2;
/// Largest CELT frame: nb_short_mdcts (8) * short_mdct_size (120) at LM=3.
const MAX_FRAME_SIZE: usize = 960;

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
pub struct OpusCustomDecoder {
    pub mode: &'static CELTMode,
    pub overlap: i32,
    pub channels: i32,
    pub stream_channels: i32,

    pub downsample: i32,
    pub start: i32,
    pub end: i32,
    pub signalling: i32,

    // Everything beyond this point gets cleared on a reset
    // (DECODER_RESET_START = rng)
    pub rng: u32,
    pub error: i32,
    pub last_pitch_index: i32,
    pub loss_count: i32,
    pub postfilter_period: i32,
    pub postfilter_period_old: i32,
    pub postfilter_gain: Val,
    pub postfilter_gain_old: Val,
    pub postfilter_tapset: i32,
    pub postfilter_tapset_old: i32,

    pub preemph_mem_d: [CeltSig; 2],

    // Trailing arrays — previously accessed via pointer arithmetic
    // from _decode_mem. Now fixed-size, always sized for 2 channels.
    pub decode_mem: [CeltSig; DECODE_MEM_SIZE],
    pub lpc: [Val; LPC_SIZE],
    pub old_band_e: [Val; BAND_E_SIZE],
    pub old_log_e: [Val; BAND_E_SIZE],
    pub old_log_e2: [Val; BAND_E_SIZE],
    pub background_log_e: [Val; BAND_E_SIZE],
}

/// Type alias matching `typedef struct OpusCustomDecoder CELTDecoder`.
pub type CELTDecoder = OpusCustomDecoder;

// -- Decoder size and init functions --

/// Initialise a CELT decoder for the standard Opus mode at the given sample rate.
pub fn celt_decoder_init(st: &mut CELTDecoder, sampling_rate: i32, channels: i32) -> i32 {
    if !(0..=2).contains(&channels) {
        return OPUS_BAD_ARG;
    }
    *st = OpusCustomDecoder::new(celt_mode(), channels);
    st.downsample = resampling_factor(sampling_rate);
    if st.downsample == 0 { OPUS_BAD_ARG } else { OPUS_OK }
}

impl OpusCustomDecoder {
    /// Build a fresh CELT decoder for `mode` and channel count (no heap).
    pub fn new(mode: &'static CELTMode, channels: i32) -> Self {
        let init_log_e = -qconst16(28.0, DB_SHIFT);
        OpusCustomDecoder {
            mode,
            overlap: mode.overlap,
            channels,
            stream_channels: channels,
            downsample: 1,
            start: 0,
            end: mode.eff_ebands,
            signalling: 1,
            rng: 0,
            error: 0,
            last_pitch_index: 0,
            loss_count: 0,
            postfilter_period: 0,
            postfilter_period_old: 0,
            postfilter_gain: 0 as Val,
            postfilter_gain_old: 0 as Val,
            postfilter_tapset: 0,
            postfilter_tapset_old: 0,
            preemph_mem_d: [0 as CeltSig; 2],
            decode_mem: [0 as CeltSig; DECODE_MEM_SIZE],
            lpc: [0 as Val; LPC_SIZE],
            old_band_e: [0 as Val; BAND_E_SIZE],
            old_log_e: [init_log_e; BAND_E_SIZE],
            old_log_e2: [init_log_e; BAND_E_SIZE],
            background_log_e: [0 as Val; BAND_E_SIZE],
        }
    }

    /// Final range-coder state of the last decoded frame.
    pub fn final_range(&self) -> u32 {
        self.rng
    }
}

/// Reset the decoder state (everything from `rng` onward).
///
/// This is the logic from `opus_custom_decoder_ctl(..., OPUS_RESET_STATE)`.
/// Factored out so both `opus_custom_decoder_init` and the ctl handler can call it.
pub fn celt_decoder_reset(st: &mut CELTDecoder) {
    // Fields up through `signalling` are configuration set at init time
    // (mode, channel layout, band range); everything from `rng` onward is
    // runtime state and gets cleared.
    let OpusCustomDecoder {
        mode: _,
        overlap: _,
        channels: _,
        stream_channels: _,
        downsample: _,
        start: _,
        end: _,
        signalling: _,
        rng,
        error,
        last_pitch_index,
        loss_count,
        postfilter_period,
        postfilter_period_old,
        postfilter_gain,
        postfilter_gain_old,
        postfilter_tapset,
        postfilter_tapset_old,
        preemph_mem_d,
        decode_mem,
        lpc,
        old_band_e,
        old_log_e,
        old_log_e2,
        background_log_e,
    } = st;

    *rng = 0;
    *error = 0;
    *last_pitch_index = 0;
    *loss_count = 0;
    *postfilter_period = 0;
    *postfilter_period_old = 0;
    *postfilter_gain = 0 as Val;
    *postfilter_gain_old = 0 as Val;
    *postfilter_tapset = 0;
    *postfilter_tapset_old = 0;
    preemph_mem_d.fill(0 as CeltSig);
    decode_mem.fill(0 as CeltSig);
    lpc.fill(0 as Val);
    old_band_e.fill(0 as Val);
    background_log_e.fill(0 as Val);

    // Initialise oldLogE and oldLogE2 to -28 dB
    let init_val = -qconst16(28.0, DB_SHIFT);
    old_log_e.fill(init_val);
    old_log_e2.fill(init_val);
}

// -- celt_decode_lost (packet loss concealment) --

/// Packet loss concealment: generate audio when no packet is available.
///
/// Uses noise-based PLC after 5+ consecutive losses (or if start!=0),
/// otherwise pitch-based PLC with LPC synthesis.
// Packet-loss concealment: a dense DSP kernel — seed-driven band-noise
// synthesis (renormalise per band), LPC analysis/synthesis (autocorr, FIR,
// IIR), excitation copy with a decaying pitch offset, symmetric TDAC
// windowing, and comb pre/post-filtering, all over channel/band-strided
// buffers (exc/mem/e and chans[cu][OM + ..], freq[c*n + i]). Indices are
// bounded by MAX_PERIOD / n / overlap and the band structure; kept indexed.
#[allow(clippy::indexing_slicing)]
pub fn celt_decode_lost(st: &mut CELTDecoder, pcm: &mut [Val], n: i32, lm: i32) {
    let mode = st.mode;
    let overlap = mode.overlap;
    let mut fade: Val = Q15ONE;
    let cc = st.channels;
    let mut offset: i32;
    let pitch_index: i32;

    // Per-channel views of decode_mem; out_mem[c] starts at OM and
    // out_syn[c] at DECODE_BUFFER_SIZE - n within each channel.
    let ch_size = (DECODE_BUFFER_SIZE + st.overlap) as usize;
    const OM: usize = (DECODE_BUFFER_SIZE - MAX_PERIOD) as usize;
    // decode_mem holds exactly MAX_CHANNELS regions of ch_size; split into a
    // fixed array of channel slices (mono leaves the second region unused).
    let (ch0, ch1) = st.decode_mem.split_at_mut(ch_size);
    let mut chans: [&mut [CeltSig]; MAX_CHANNELS as usize] = [ch0, ch1];
    let os = (DECODE_BUFFER_SIZE - n) as usize; // out_syn offset

    let len = n + mode.overlap;

    if st.loss_count >= 5 || st.start != 0 {
        // Noise-based PLC/CNG
        let mut seed: u32;

        let mut eff_end = st.end;
        if eff_end > mode.eff_ebands {
            eff_end = mode.eff_ebands;
        }

        let mut freq = [0 as CeltSig; MAX_CHANNELS as usize * MAX_FRAME_SIZE];
        let freq = &mut freq[..(cc * n) as usize];
        let mut x = [0 as CeltNorm; MAX_CHANNELS as usize * MAX_FRAME_SIZE];
        let x = &mut x[..(cc * n) as usize];
        let mut band_e = [0 as CeltEner; (NB_EBANDS * MAX_CHANNELS) as usize];
        let band_e = &mut band_e[..(mode.nb_ebands * cc) as usize];

        if st.loss_count >= 5 {
            log2amp(mode, st.start, st.end, band_e, &st.background_log_e, cc);
        } else {
            // Energy decay
            let decay: Val = if st.loss_count == 0 { qconst16(1.5, DB_SHIFT) } else { qconst16(0.5, DB_SHIFT) };
            let mut c = 0;
            loop {
                for i in st.start..st.end {
                    st.old_band_e[(c * mode.nb_ebands + i) as usize] -= decay;
                }
                c += 1;
                if c >= cc {
                    break;
                }
            }
            log2amp(mode, st.start, st.end, band_e, &st.old_band_e, cc);
        }
        seed = st.rng;
        for c in 0..cc {
            for i in 0..(mode.ebands[st.start as usize] as i32) << lm {
                x[(c * n + i) as usize] = 0 as CeltNorm;
            }
            for i in st.start..mode.eff_ebands {
                let boffs = (n * c + ((mode.ebands[i as usize] as i32) << lm)) as usize;
                let blen = ((mode.ebands[i as usize + 1] - mode.ebands[i as usize]) as i32) << lm;
                for j in 0..blen as usize {
                    seed = celt_lcg_rand(seed);
                    x[boffs + j] = (seed as i32 >> 20) as CeltNorm;
                }
                renormalise_vector(&mut x[boffs..boffs + blen as usize], Q15ONE);
            }
            for i in ((mode.ebands[st.end as usize] as i32) << lm)..n {
                x[(c * n + i) as usize] = 0 as CeltNorm;
            }
        }
        st.rng = seed;

        denormalise_bands(mode, x, freq, band_e, mode.eff_ebands, cc, 1 << lm);

        let mut c = 0;
        loop {
            for i in 0..((mode.ebands[st.start as usize] as i32) << lm) {
                freq[(c * n + i) as usize] = 0 as CeltSig;
            }
            c += 1;
            if c >= cc {
                break;
            }
        }
        c = 0;
        loop {
            let mut bound = (mode.ebands[eff_end as usize] as i32) << lm;
            if st.downsample != 1 {
                bound = bound.min(n / st.downsample);
            }
            for i in bound..n {
                freq[(c * n + i) as usize] = 0 as CeltSig;
            }
            c += 1;
            if c >= cc {
                break;
            }
        }
        {
            let ch_len = (n + st.overlap) as usize;
            if cc == 2 {
                let (c0, c1) = chans.split_at_mut_checked(1).or_panic("chans has fewer than 2 channel slices");
                compute_inv_mdcts(mode, 0, freq, &mut [&mut c0[0][os..os + ch_len], &mut c1[0][os..os + ch_len]], cc, lm);
            } else {
                compute_inv_mdcts(mode, 0, freq, &mut [&mut chans[0][os..os + ch_len]], cc, lm);
            }
        }
    } else {
        // Pitch-based PLC
        if st.loss_count == 0 {
            let mut pitch_buf = [0 as Val; (DECODE_BUFFER_SIZE >> 1) as usize];
            let poffset: i32 = 720;
            {
                let ch0 = &chans[0][..DECODE_BUFFER_SIZE as usize];
                if cc == 2 {
                    let ch1 = &chans[1][..DECODE_BUFFER_SIZE as usize];
                    pitch_downsample(&[ch0, ch1], &mut pitch_buf, DECODE_BUFFER_SIZE, cc);
                } else {
                    pitch_downsample(&[ch0], &mut pitch_buf, DECODE_BUFFER_SIZE, cc);
                }
            }
            let mut pi: i32 = 0;
            pitch_search(
                &pitch_buf[(poffset >> 1) as usize..],
                &pitch_buf,
                DECODE_BUFFER_SIZE - poffset,
                poffset - 100,
                &mut pi,
            );
            pitch_index = poffset - pi;
            st.last_pitch_index = pitch_index;
        } else {
            pitch_index = st.last_pitch_index;
            fade = qconst16(0.8, 15);
        }

        let mut c = 0;
        loop {
            let cu = c as usize;
            let mut exc = [0 as Val; MAX_PERIOD as usize];
            let mut ac = [0 as Wal; (LPC_ORDER + 1) as usize];
            let mut decay: Val;
            let mut s1: Wal = 0 as Wal;
            let mut mem = [0 as Val; LPC_ORDER as usize];

            let mut e = [0 as Wal; (MAX_PERIOD + 2 * OVERLAP) as usize];

            offset = MAX_PERIOD - pitch_index;
            for i in 0..MAX_PERIOD as usize {
                exc[i] = round16(chans[cu][OM + i], SIG_SHIFT);
            }

            if st.loss_count == 0 {
                _celt_autocorr(&exc, &mut ac, mode.window, mode.overlap, LPC_ORDER);

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
                        ac[i as usize] -= mult16_32_q15((2 * i * i) as Val, ac[i as usize]);
                    }
                    #[cfg(not(feature = "fixed-point"))]
                    {
                        ac[i as usize] -= ac[i as usize] * (0.008 * i as f32) * (0.008 * i as f32);
                    }
                }

                let lpc_off = (c * LPC_ORDER) as usize;
                _celt_lpc(&mut st.lpc[lpc_off..lpc_off + LPC_ORDER as usize], &ac, LPC_ORDER);
            }
            for i in 0..LPC_ORDER as usize {
                mem[i] = round16(chans[cu][OM + MAX_PERIOD as usize - 1 - i], SIG_SHIFT);
            }
            {
                let lpc_off = (c * LPC_ORDER) as usize;
                celt_fir(&mut exc, &st.lpc[lpc_off..lpc_off + LPC_ORDER as usize], LPC_ORDER, &mut mem);
            }
            // Check if the waveform is decaying (and if so how fast)
            {
                let mut e1: Wal = 1 as Wal;
                let mut e2: Wal = 1 as Wal;
                let period = if pitch_index <= MAX_PERIOD / 2 { pitch_index } else { MAX_PERIOD / 2 };
                for i in 0..period {
                    e1 += shr32(mult16_16(exc[(MAX_PERIOD - period + i) as usize], exc[(MAX_PERIOD - period + i) as usize]), 8);
                    e2 += shr32(
                        mult16_16(exc[(MAX_PERIOD - 2 * period + i) as usize], exc[(MAX_PERIOD - 2 * period + i) as usize]),
                        8,
                    );
                }
                if e1 > e2 {
                    e1 = e2;
                }
                decay = celt_sqrt(frac_div32(shr32(e1, 1), e2)) as Val;
            }

            // Copy excitation, taking decay into account
            for i in 0..len + mode.overlap {
                if offset + i >= MAX_PERIOD {
                    offset -= pitch_index;
                    decay = mult16_16_q15(decay, decay) as Val;
                }
                e[i as usize] = shl32(extend32(mult16_16_q15(decay, exc[(offset + i) as usize]) as Val), SIG_SHIFT);
                let tmp: Val = round16(chans[cu][OM + (offset + i) as usize], SIG_SHIFT);
                s1 += shr32(mult16_16(tmp, tmp), 8);
            }
            for i in 0..LPC_ORDER as usize {
                mem[i] = round16(chans[cu][OM + MAX_PERIOD as usize - 1 - i], SIG_SHIFT);
            }
            let e_len = (len + mode.overlap) as usize;
            for x in &mut e[..e_len] {
                *x = mult16_32_q15(fade, *x);
            }
            {
                let lpc_off = (c * LPC_ORDER) as usize;
                celt_iir(&mut e[..e_len], &st.lpc[lpc_off..lpc_off + LPC_ORDER as usize], LPC_ORDER, &mut mem);
            }

            {
                let mut s2: Wal = 0 as Wal;
                for &x in &e[..e_len] {
                    let tmp: Val = round16(x, SIG_SHIFT);
                    s2 += shr32(mult16_16(tmp, tmp), 8);
                }
                // This checks for an "explosion" in the synthesis
                #[cfg(feature = "fixed-point")]
                let explosion = s1 <= shr32(s2, 2);
                // Negated `>` (not `<=`) is deliberate, mirroring the C: an
                // unordered/NaN comparison must count as an explosion.
                #[cfg(not(feature = "fixed-point"))]
                #[allow(clippy::neg_cmp_op_on_partial_ord)]
                let explosion = !(s1 > 0.2 * s2);

                if explosion {
                    e[..e_len].fill(0 as Wal);
                } else if s1 < s2 {
                    let ratio: Val = celt_sqrt(frac_div32(shr32(s1, 1) + 1 as Wal, s2 + 1 as Wal)) as Val;
                    for x in &mut e[..e_len] {
                        *x = mult16_32_q15(ratio, *x);
                    }
                }
            }

            // Apply post-filter to the MDCT overlap of the previous frame
            comb_filter(
                None,
                &mut chans[cu][..],
                DECODE_BUFFER_SIZE as usize,
                st.postfilter_period,
                st.postfilter_period,
                st.overlap,
                st.postfilter_gain,
                st.postfilter_gain,
                st.postfilter_tapset,
                st.postfilter_tapset,
                &[],
                0,
            );

            // Move the decoded audio back by N samples
            let mv_len = (MAX_PERIOD + mode.overlap - n) as usize;
            // In bounds: src end OM + n + mv_len == OM + MAX_PERIOD + overlap == ch_size
            // (the channel length), so the copy stays within chans[cu].
            chans[cu].copy_within(OM + n as usize..OM + n as usize + mv_len, OM);

            // Apply TDAC to the concealed audio so that it blends with the
            // previous and next frames
            for i in 0..overlap / 2 {
                let tmp: Wal = mult16_32_q15(mode.window[i as usize], e[(n + overlap - 1 - i) as usize])
                    + mult16_32_q15(mode.window[(overlap - i - 1) as usize], e[(n + i) as usize]);
                chans[cu][OM + (MAX_PERIOD + i) as usize] = mult16_32_q15(mode.window[(overlap - i - 1) as usize], tmp);
                chans[cu][OM + (MAX_PERIOD + overlap - i - 1) as usize] = mult16_32_q15(mode.window[i as usize], tmp);
            }
            for i in 0..n {
                chans[cu][OM + (MAX_PERIOD - n + i) as usize] = e[i as usize];
            }

            // Apply pre-filter to the MDCT overlap for the next frame
            comb_filter(
                Some(&mut e),
                &mut chans[cu][..],
                DECODE_BUFFER_SIZE as usize,
                st.postfilter_period,
                st.postfilter_period,
                st.overlap,
                -st.postfilter_gain,
                -st.postfilter_gain,
                st.postfilter_tapset,
                st.postfilter_tapset,
                &[],
                0,
            );
            for i in 0..overlap as usize {
                chans[cu][OM + MAX_PERIOD as usize + i] = e[i];
            }

            c += 1;
            if c >= cc {
                break;
            }
        }
    }

    {
        let ch0 = &chans[0][os..os + n as usize];
        if cc == 2 {
            let ch1 = &chans[1][os..os + n as usize];
            deemphasis(&[ch0, ch1], pcm, n, cc, st.downsample, &mode.preemph, &mut st.preemph_mem_d);
        } else {
            deemphasis(&[ch0], pcm, n, cc, st.downsample, &mode.preemph, &mut st.preemph_mem_d);
        }
    }

    st.loss_count += 1;
}

// -- ICDFs (used by celt_decode_with_ec) --

static TRIM_ICDF: [u8; 11] = [126, 124, 119, 109, 87, 41, 19, 9, 4, 2, 0];
static SPREAD_ICDF: [u8; 4] = [25, 23, 2, 0];
static TAPSET_ICDF: [u8; 3] = [2, 1, 0];

// -- celt_decode_with_ec (main decoder entry) --

/// Main CELT decode function.
///
/// Decodes a CELT frame from the bitstream into PCM samples.
/// If data is None or len <= 1, runs packet loss concealment instead.
pub fn celt_decode_with_ec<'a>(
    st: &mut CELTDecoder,
    data: Option<&'a [u8]>,
    len: i32,
    pcm: &mut [Val],
    frame_size: i32,
    dec: Option<&mut EcCtx<'a>>,
) -> i32 {
    let mode = st.mode;
    let cc = st.channels;
    let ch_size = (DECODE_BUFFER_SIZE + st.overlap) as usize;

    let mut frame_size = frame_size;
    frame_size *= st.downsample;

    // Without CUSTOM_MODES, find LM from frame_size
    let mut lm: i32 = 0;
    while lm <= mode.max_lm {
        if mode.short_mdct_size << lm == frame_size {
            break;
        }
        lm += 1;
    }
    if lm > mode.max_lm {
        return OPUS_BAD_ARG;
    }
    let m: i32 = 1 << lm;

    if !(0..=1275).contains(&len) {
        return OPUS_BAD_ARG;
    }

    let n: i32 = m * mode.short_mdct_size;

    let mut eff_end = st.end;
    if eff_end > mode.eff_ebands {
        eff_end = mode.eff_ebands;
    }

    let c_channels = st.stream_channels;
    let mut freq = [0 as CeltSig; MAX_CHANNELS as usize * MAX_FRAME_SIZE];
    let freq_len = (cc.max(c_channels) * n) as usize;
    let freq = freq.get_mut(..freq_len).or_panic(freq_len);
    let mut x = [0 as CeltNorm; MAX_CHANNELS as usize * MAX_FRAME_SIZE];
    let x_len = (c_channels * n) as usize;
    let x = x.get_mut(..x_len).or_panic(x_len);
    let mut band_e = [0 as CeltEner; (NB_EBANDS * MAX_CHANNELS) as usize];
    let band_e_len = (mode.nb_ebands * c_channels) as usize;
    let band_e = band_e.get_mut(..band_e_len).or_panic(band_e_len);

    // First/last active bin (in MDCT samples) for the start and eff_end bands.
    let band_start = (m * i32::from(*mode.ebands.get(st.start as usize).or_panic(st.start))) as usize;
    let band_eff_end = (m * i32::from(*mode.ebands.get(eff_end as usize).or_panic(eff_end))) as usize;

    // Per-channel clear of the inactive low/high band bins.
    for ch in x.chunks_mut(n as usize).take(c_channels as usize) {
        ch.get_mut(..band_start).or_panic(band_start).fill(0 as CeltNorm);
        ch.get_mut(band_eff_end..).or_panic(band_eff_end).fill(0 as CeltNorm);
    }

    let Some(data) = data.filter(|_| len > 1) else {
        let pcm_len = (n / st.downsample * cc) as usize;
        celt_decode_lost(st, pcm.get_mut(..pcm_len).or_panic(pcm_len), n, lm);
        return frame_size / st.downsample;
    };

    let mut _dec = EcCtx::empty();
    let dec: &mut EcCtx = match dec {
        Some(d) => d,
        None => {
            ec_dec_init(&mut _dec, data.get(..len as usize).or_panic(len), len as u32);
            &mut _dec
        }
    };

    if c_channels == 1 {
        let nb = mode.nb_ebands as usize;
        let (lo, hi) = st.old_band_e.split_at_mut_checked(nb).or_panic("old_band_e shorter than nb");
        for (a, &b) in zip(lo, &*hi) {
            *a = max16(*a, b);
        }
    }

    let mut total_bits: i32 = len * 8;
    let mut tell: i32 = ec_tell(dec);

    let silence: i32;
    if tell >= total_bits {
        silence = 1;
    } else if tell == 1 {
        silence = ec_dec_bit_logp(dec, 15);
    } else {
        silence = 0;
    }
    if silence != 0 {
        tell = len * 8;
        dec.nbits_total += tell - ec_tell(dec);
    }

    let mut postfilter_gain: Val = 0 as Val;
    let mut postfilter_pitch: i32 = 0;
    let mut postfilter_tapset: i32 = 0;
    if st.start == 0 && tell + 16 <= total_bits {
        if ec_dec_bit_logp(dec, 1) != 0 {
            let octave = ec_dec_uint(dec, 6) as i32;
            postfilter_pitch = (16 << octave) + ec_dec_bits(dec, (4 + octave) as u32) as i32 - 1;
            let qg = ec_dec_bits(dec, 3) as i32;
            if ec_tell(dec) + 2 <= total_bits {
                postfilter_tapset = ec_dec_icdf(dec, &TAPSET_ICDF, 2);
            }
            postfilter_gain = qconst16(0.09375, 15) * (qg + 1) as Val;
        }
        tell = ec_tell(dec);
    }

    let is_transient: i32;
    if lm > 0 && tell + 3 <= total_bits {
        is_transient = ec_dec_bit_logp(dec, 3);
        tell = ec_tell(dec);
    } else {
        is_transient = 0;
    }

    let short_blocks: i32 = if is_transient != 0 { m } else { 0 };

    // Decode the global flags (first symbols in the stream)
    let intra_ener: i32 = if tell + 3 <= total_bits { ec_dec_bit_logp(dec, 3) } else { 0 };
    // Get band energies
    unquant_coarse_energy(mode, st.start, st.end, &mut st.old_band_e, intra_ener, dec, c_channels, lm);

    let mut tf_res = [0i32; NB_EBANDS as usize];
    tf_decode(st.start, st.end, is_transient, &mut tf_res, lm, dec);

    tell = ec_tell(dec);
    let mut spread_decision: i32 = SPREAD_NORMAL;
    if tell + 4 <= total_bits {
        spread_decision = ec_dec_icdf(dec, &SPREAD_ICDF, 5);
    }

    let mut pulses = [0i32; NB_EBANDS as usize];
    let mut cap = [0i32; NB_EBANDS as usize];
    let mut offsets = [0i32; NB_EBANDS as usize];
    let mut fine_priority = [0i32; NB_EBANDS as usize];

    init_caps(mode, &mut cap, lm, c_channels);

    let mut dynalloc_logp: i32 = 6;
    total_bits <<= BITRES;
    tell = ec_tell_frac(dec) as i32;
    // Decoder-interleaved per-band dynalloc: ec reads gated by cap[ii] with
    // mode.ebands[..] widths, writing offsets[ii]. Kept indexed.
    #[allow(clippy::indexing_slicing)]
    for ii in st.start..st.end {
        let width = (c_channels * (mode.ebands[ii as usize + 1] - mode.ebands[ii as usize]) as i32) << lm;
        let quanta = (width << BITRES).min((6i32 << BITRES).max(width));
        let mut dynalloc_loop_logp = dynalloc_logp;
        let mut boost: i32 = 0;
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

    let mut fine_quant = [0i32; NB_EBANDS as usize];
    let alloc_trim: i32 = if tell + (6 << BITRES) <= total_bits { ec_dec_icdf(dec, &TRIM_ICDF, 7) } else { 5 };

    let mut bits: i32 = ((len * 8) << BITRES) - ec_tell_frac(dec) as i32 - 1;
    let anti_collapse_rsv: i32 = if is_transient != 0 && lm >= 2 && bits >= ((lm + 2) << BITRES) { 1 << BITRES } else { 0 };
    bits -= anti_collapse_rsv;
    let mut intensity: i32 = 0;
    let mut dual_stereo: i32 = 0;
    let mut balance: i32 = 0;
    let coded_bands = compute_allocation(
        mode,
        st.start,
        st.end,
        &offsets,
        &cap,
        alloc_trim,
        &mut intensity,
        &mut dual_stereo,
        bits,
        &mut balance,
        &mut pulses,
        &mut fine_quant,
        &mut fine_priority,
        c_channels,
        lm,
        dec,
        0,
        0,
    );

    unquant_fine_energy(mode, st.start, st.end, &mut st.old_band_e, &fine_quant, dec, c_channels);

    // Decode fixed codebook
    let mut collapse_masks = [0u8; (MAX_CHANNELS * NB_EBANDS) as usize];
    let cm_len = (c_channels * mode.nb_ebands) as usize;
    let collapse_masks = collapse_masks.get_mut(..cm_len).or_panic(cm_len);
    {
        let (x_ch, y_ch) = x.split_at_mut_checked(n as usize).or_panic("x shorter than n");
        quant_all_bands(
            0,
            mode,
            st.start,
            st.end,
            x_ch,
            if c_channels == 2 { Some(y_ch) } else { None },
            collapse_masks,
            &pulses,
            short_blocks,
            spread_decision,
            dual_stereo,
            intensity,
            &tf_res,
            len * (8 << BITRES) - anti_collapse_rsv,
            balance,
            dec,
            lm,
            coded_bands,
            &mut st.rng,
        );
    }

    let mut anti_collapse_on: i32 = 0;
    if anti_collapse_rsv > 0 {
        anti_collapse_on = ec_dec_bits(dec, 1) as i32;
    }

    unquant_energy_finalise(
        mode,
        st.start,
        st.end,
        &mut st.old_band_e,
        &fine_quant,
        &fine_priority,
        len * 8 - ec_tell(dec),
        dec,
        c_channels,
    );

    if anti_collapse_on != 0 {
        anti_collapse(
            mode, x, collapse_masks, lm, c_channels, n, st.start, st.end, &st.old_band_e, &st.old_log_e, &st.old_log_e2,
            &pulses, st.rng,
        );
    }

    log2amp(mode, st.start, st.end, band_e, &st.old_band_e, c_channels);

    if silence != 0 {
        let n_bands = (c_channels * mode.nb_ebands) as usize;
        band_e.fill(0 as CeltEner);
        st.old_band_e.get_mut(..n_bands).or_panic(n_bands).fill(-qconst16(28.0, DB_SHIFT));
    }
    // Synthesis
    denormalise_bands(mode, x, freq, band_e, eff_end, c_channels, m);

    // OPUS_MOVE: memmove decode_mem forward by N
    for ci in 0..cc as usize {
        let base = ci * ch_size;
        // In bounds: base + DECODE_BUFFER_SIZE < base + ch_size <= cc*ch_size <= decode_mem.len().
        st.decode_mem.copy_within(base + n as usize..base + DECODE_BUFFER_SIZE as usize, base);
    }

    // Per-channel clear of the inactive freq bins; the high bound is also
    // capped by the downsample ratio.
    let freq_bound = if st.downsample != 1 { band_eff_end.min((n / st.downsample) as usize) } else { band_eff_end };
    for ch in freq.chunks_mut(n as usize).take(c_channels as usize) {
        ch.get_mut(..band_start).or_panic(band_start).fill(0 as CeltSig);
        ch.get_mut(freq_bound..).or_panic(freq_bound).fill(0 as CeltSig);
    }

    // out_syn[c] starts at DECODE_BUFFER_SIZE - n within each channel
    let os = (DECODE_BUFFER_SIZE - n) as usize;

    if cc == 2 && c_channels == 1 {
        // Duplicate the single decoded channel into the second region.
        freq.copy_within(0..n as usize, n as usize);
    }
    if cc == 1 && c_channels == 2 {
        // Downmix the two decoded channels into the first region.
        let (lo, hi) = freq.split_at_mut(n as usize);
        for (x, &y) in zip(lo, &*hi) {
            *x = half32(add32(*x, y));
        }
    }

    // Compute inverse MDCTs, one channel region per channel.
    {
        let ch_len = (n + st.overlap) as usize;
        // decode_mem is MAX_CHANNELS regions of ch_size (DECODE_BUFFER_SIZE +
        // overlap) each, so the split at ch_size always lands on the channel-1
        // boundary; the checked split documents that and reports if it ever
        // doesn't, rather than panicking bare like split_at_mut would.
        let (c0, c1) = st.decode_mem.split_at_mut_checked(ch_size).or_panic("decode_mem shorter than ch_size");
        if cc == 2 {
            compute_inv_mdcts(
                mode,
                short_blocks,
                freq,
                &mut [
                    c0.get_mut(os..os + ch_len).or_panic_dbg((os, ch_len)),
                    c1.get_mut(os..os + ch_len).or_panic_dbg((os, ch_len)),
                ],
                cc,
                lm,
            );
        } else {
            compute_inv_mdcts(mode, short_blocks, freq, &mut [c0.get_mut(os..os + ch_len).or_panic_dbg((os, ch_len))], cc, lm);
        }
    }

    st.postfilter_period = st.postfilter_period.max(COMBFILTER_MINPERIOD);
    st.postfilter_period_old = st.postfilter_period_old.max(COMBFILTER_MINPERIOD);
    for ch in st.decode_mem.chunks_mut(ch_size).take(cc as usize) {
        comb_filter(
            None,
            ch,
            (DECODE_BUFFER_SIZE - n) as usize,
            st.postfilter_period_old,
            st.postfilter_period,
            mode.short_mdct_size,
            st.postfilter_gain_old,
            st.postfilter_gain,
            st.postfilter_tapset_old,
            st.postfilter_tapset,
            mode.window,
            st.overlap,
        );
        if lm != 0 {
            comb_filter(
                None,
                ch,
                (DECODE_BUFFER_SIZE - n + mode.short_mdct_size) as usize,
                st.postfilter_period,
                postfilter_pitch,
                n - mode.short_mdct_size,
                st.postfilter_gain,
                postfilter_gain,
                st.postfilter_tapset,
                postfilter_tapset,
                mode.window,
                mode.overlap,
            );
        }
    }
    st.postfilter_period_old = st.postfilter_period;
    st.postfilter_gain_old = st.postfilter_gain;
    st.postfilter_tapset_old = st.postfilter_tapset;
    st.postfilter_period = postfilter_pitch;
    st.postfilter_gain = postfilter_gain;
    st.postfilter_tapset = postfilter_tapset;
    if lm != 0 {
        st.postfilter_period_old = st.postfilter_period;
        st.postfilter_gain_old = st.postfilter_gain;
        st.postfilter_tapset_old = st.postfilter_tapset;
    }

    if c_channels == 1 {
        let nb = mode.nb_ebands as usize;
        st.old_band_e.copy_within(0..nb, nb);
    }

    // In case start or end were to change
    let two_nb = (2 * mode.nb_ebands) as usize;
    if is_transient == 0 {
        let (le2, le) = (st.old_log_e2.get_mut(..two_nb).or_panic(two_nb), st.old_log_e.get(..two_nb).or_panic(two_nb));
        for (d, &s) in zip(le2, le) {
            *d = s;
        }
        let (le, be) = (st.old_log_e.get_mut(..two_nb).or_panic(two_nb), st.old_band_e.get(..two_nb).or_panic(two_nb));
        for (d, &s) in zip(le, be) {
            *d = s;
        }
        let (bg, be) = (st.background_log_e.get_mut(..two_nb).or_panic(two_nb), st.old_band_e.get(..two_nb).or_panic(two_nb));
        for (bg, &oe) in zip(bg, be) {
            *bg = min16(*bg + m as Val * qconst16(0.001, DB_SHIFT), oe);
        }
    } else {
        let (le, be) = (st.old_log_e.get_mut(..two_nb).or_panic(two_nb), st.old_band_e.get(..two_nb).or_panic(two_nb));
        for (le, &oe) in zip(le, be) {
            *le = min16(*le, oe);
        }
    }
    // Reset the inactive bands ([0,start) and [end,nb)) across all three
    // energy-history arrays, for both channels.
    let nb = mode.nb_ebands as usize;
    let (start, end) = (st.start as usize, st.end as usize);
    for (be, le, le2) in zip3(st.old_band_e.chunks_mut(nb), st.old_log_e.chunks_mut(nb), st.old_log_e2.chunks_mut(nb)).take(2) {
        for (i, (b, l, l2)) in zip3(be, le, le2).enumerate() {
            if i < start || i >= end {
                *b = 0 as Val;
                *l = -qconst16(28.0, DB_SHIFT);
                *l2 = -qconst16(28.0, DB_SHIFT);
            }
        }
    }
    st.rng = dec.rng;

    {
        let pcm_len = (n / st.downsample * cc) as usize;
        let ch0 = st.decode_mem.get(os..os + n as usize).or_panic_dbg((os, n));
        if cc == 2 {
            let ch1 = st.decode_mem.get(ch_size + os..ch_size + os + n as usize).or_panic_dbg((ch_size + os, n));
            deemphasis(
                &[ch0, ch1],
                pcm.get_mut(..pcm_len).or_panic(pcm_len),
                n,
                cc,
                st.downsample,
                &mode.preemph,
                &mut st.preemph_mem_d,
            );
        } else {
            deemphasis(
                &[ch0],
                pcm.get_mut(..pcm_len).or_panic(pcm_len),
                n,
                cc,
                st.downsample,
                &mode.preemph,
                &mut st.preemph_mem_d,
            );
        }
    }
    st.loss_count = 0;
    if ec_tell(dec) > 8 * len {
        return OPUS_INTERNAL_ERROR;
    }
    if dec.error != 0 {
        st.error = 1;
    }
    frame_size / st.downsample
}

// -- tf_select_table (used by tf_decode) --

static TF_SELECT_TABLE: [[i8; 8]; 4] =
    [[0, -1, 0, -1, 0, -1, 0, -1], [0, -1, 0, -2, 1, 0, 1, -1], [0, -2, 0, -3, 2, 0, 1, -1], [0, -2, 0, -3, 3, 0, 1, -1]];

// -- Small helper functions --

/// Map sample rate to resampling factor.
/// Only the five standard Opus rates are supported.
pub fn resampling_factor(rate: i32) -> i32 {
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
pub fn sig2word16(x: crate::arch::CeltSig) -> Val {
    let x = pshr32(x, SIG_SHIFT);
    let x = max32(x, -32768);
    let x = min32(x, 32767);
    extract16(x)
}

#[cfg(not(feature = "fixed-point"))]
pub fn sig2word16(x: crate::arch::CeltSig) -> Val {
    x
}

/// Scale an output sample for final PCM output.
///
/// Fixed-point: identity (already in Q0).
/// Float: divide by CELT_SIG_SCALE (32768).
#[inline(always)]
#[cfg(feature = "fixed-point")]
pub fn scaleout(a: Val) -> Val {
    a
}

#[inline(always)]
#[cfg(not(feature = "fixed-point"))]
pub fn scaleout(a: Val) -> Val {
    a * (1.0 / CELT_SIG_SCALE)
}

// -- tf_decode --

/// Decode the time-frequency resolution flags for each band.
///
/// Reads a sequence of binary flags from the entropy coder indicating
/// whether each band uses a finer time or frequency resolution, then
/// applies a selection table to map these to actual tf_change values.
pub fn tf_decode(start: i32, end: i32, is_transient: i32, tf_res: &mut [i32], lm: i32, dec: &mut EcCtx) {
    let budget = dec.storage * 8;
    let mut tell = ec_tell(dec) as u32;
    let mut logp: u32 = if is_transient != 0 { 2 } else { 4 };
    let tf_select_rsv = (lm > 0 && tell + logp < budget) as i32;
    let budget = budget - tf_select_rsv as u32;
    let mut tf_changed = 0;
    let mut curr = 0;
    for r in tf_res.get_mut(start as usize..end as usize).or_panic_dbg((start, end)) {
        if tell + logp <= budget {
            curr ^= ec_dec_bit_logp(dec, logp);
            tell = ec_tell(dec) as u32;
            tf_changed |= curr;
        }
        *r = curr;
        logp = if is_transient != 0 { 4 } else { 5 };
    }
    let tf_row = TF_SELECT_TABLE.get(lm as usize).or_panic(lm);
    let tf_at = |k: i32| *tf_row.get(k as usize).or_panic(k);
    let mut tf_select = 0;
    if tf_select_rsv != 0 && tf_at(4 * is_transient + tf_changed) != tf_at(4 * is_transient + 2 + tf_changed) {
        tf_select = ec_dec_bit_logp(dec, 1);
    }
    for r in tf_res.get_mut(start as usize..end as usize).or_panic_dbg((start, end)) {
        *r = i32::from(tf_at(4 * is_transient + 2 * tf_select + *r));
    }
}

// -- init_caps --

/// Initialise the per-band bit allocation caps from the mode's cache.
pub fn init_caps(m: &CELTMode, cap: &mut [i32], lm: i32, c: i32) {
    // Walk cap alongside adjacent eBands pairs (band width) and the matching
    // row of the caps cache.
    let row = m.nb_ebands as usize * (2 * lm as usize + c as usize - 1);
    for (capi, &lo, &hi, &capval) in zip4(cap, m.ebands, m.ebands.iter().skip(1), m.cache.caps.iter().skip(row)) {
        let n = i32::from(hi - lo) << lm;
        *capi = ((i32::from(capval) + 64) * c * n) >> 2;
    }
}

// -- compute_inv_mdcts --

/// Compute inverse MDCTs for all channels, with overlap-add.
///
/// For each channel: runs the IMDCT (possibly multiple short blocks),
/// overlap-adds with the previous frame's tail, and saves the new tail
/// into overlap_mem.
/// `out_syn` holds one slice per channel covering `n + overlap` samples:
/// the C's separate `out_mem[c]` (n samples) and `overlap_mem[c]`
/// (overlap samples) pointers are contiguous in decode_mem, so each
/// channel passes the combined region and the overlap memory lives at
/// `[n..n + overlap]`.
// Per-channel IMDCT + overlap-add: strided channel slices of `x`, the
// `clt_mdct_backward` butterfly, and an overlap-add that aliases `chan` at
// `[j]` and `[n+j]`. Kept indexed (bounds: n + overlap per channel region).
#[allow(clippy::indexing_slicing)]
pub fn compute_inv_mdcts(
    mode: &CELTMode,
    short_blocks: i32,
    x: &[CeltSig],
    out_syn: &mut [&mut [CeltSig]],
    c_channels: i32,
    lm: i32,
) {
    let n = mode.short_mdct_size << lm;
    let overlap = mode.overlap;
    let nu = n as usize;
    let ov = overlap as usize;
    let mut buf = [0 as Wal; MAX_FRAME_SIZE + OVERLAP as usize];
    let buf = &mut buf[..(n + overlap) as usize];

    let (n2, b_count) = if short_blocks != 0 { (mode.short_mdct_size, short_blocks) } else { (n, 1) };

    for (c, chan) in out_syn.iter_mut().enumerate().take(c_channels as usize) {
        // Prevents problems from the imdct doing the overlap-add
        buf.iter_mut().take(ov).for_each(|v| *v = 0 as Wal);

        for b in 0..b_count {
            let x_ch = x.get(c * nu + b as usize..(c + 1) * nu).or_panic_dbg((c * nu + b as usize, (c + 1) * nu));
            clt_mdct_backward(
                &mode.mdct,
                x_ch,
                buf,
                (n2 * b) as usize,
                mode.window,
                overlap,
                if short_blocks != 0 { mode.max_lm } else { mode.max_lm - lm },
                b_count,
            );
        }

        // Overlap-add: body[..ov] mixes the previous tail (chan[n..]) with the
        // fresh imdct output; body[ov..] copies it; then the new tail is saved.
        let (body, tail) = chan.split_at_mut(nu);
        for (b, &bf, &t) in zip3(body.iter_mut(), buf.iter(), tail.iter()) {
            *b = bf + t;
        }
        for (b, &bf) in zip(body.iter_mut(), buf.iter()).skip(ov) {
            *b = bf;
        }
        for (t, &bf) in zip(tail, buf.iter().skip(nu)) {
            *t = bf;
        }
    }
}

// -- deemphasis --

/// Apply de-emphasis filter and produce interleaved PCM output.
///
/// The de-emphasis is a first-order IIR filter that undoes the pre-emphasis
/// applied before encoding. Also handles downsampling (e.g. 48→8 kHz)
/// by writing only every `downsample`-th sample.
pub fn deemphasis(
    in_: &[&[CeltSig]],
    pcm: &mut [Val],
    n: i32,
    c_channels: i32,
    downsample: i32,
    coef: &[Val],
    mem: &mut [CeltSig],
) {
    let c0 = *coef.first().or_panic("deemphasis coef[0] out of range");
    let c1 = *coef.get(1).or_panic("deemphasis coef[1] out of range");
    let c3 = *coef.get(3).or_panic("deemphasis coef[3] out of range");
    // `count` deliberately carries across channels, matching the C.
    let mut count: i32 = 0;
    for (c, (&x, m_slot)) in zip(in_, mem.iter_mut()).enumerate().take(c_channels as usize) {
        // Channel c's interleaved output positions: c, c+channels, c+2*channels, ...
        let mut out = pcm.iter_mut().skip(c).step_by(c_channels as usize);
        let mut m = *m_slot;
        for &xj in x.get(..n as usize).or_panic(n) {
            let tmp = xj + m;
            m = mult16_32_q15(c0, tmp) - mult16_32_q15(c1, xj);
            let tmp = shl32(mult16_32_q15(c3, tmp), 2);
            if count == 0
                && let Some(o) = out.next()
            {
                *o = scaleout(sig2word16(tmp));
            }
            count += 1;
            if count == downsample {
                count = 0;
            }
        }
        *m_slot = m;
    }
}

// -- comb_filter --

/// Post-filter comb filter with smooth crossfade between two pitch periods.
///
/// Applies a 3-tap comb filter at pitch lag T1 for the steady-state portion,
/// and crossfades from the old pitch T0 to T1 over the overlap region using
/// a squared-window interpolation.
///
/// The C takes x/y pointers where x is read at negative offsets (pitch
/// history before the start) and y usually aliases x (in-place, with the
/// filter feeding back through its own output once i > T). The Rust
/// version takes the whole backing buffer `x` plus `x_off` (the position
/// the C pointer would have had), and `y: None` for the in-place case or
/// `y: Some(out)` for a separate output (used by the PLC pre-filter).
// 3-tap comb filter reading pitch history at negative offsets
// (x[xi - t0 +- k], x[xi - t1 +- k]) with a crossfade window and a 2D
// `gains[tapset][k]` table. Kept indexed (a DSP kernel).
#[allow(clippy::too_many_arguments, clippy::indexing_slicing)]
pub fn comb_filter(
    y: Option<&mut [Wal]>,
    x: &mut [Wal],
    x_off: usize,
    t0: i32,
    t1: i32,
    n: i32,
    g0: Val,
    g1: Val,
    tapset0: i32,
    tapset1: i32,
    window: &[Val],
    overlap: i32,
) {
    #[cfg(not(feature = "fixed-point"))]
    #[allow(clippy::excessive_precision)] // canonical tapset gains, kept verbatim
    let gains: [[Val; 3]; 3] =
        [[0.3066406250, 0.2170410156, 0.1296386719], [0.4638671875, 0.2680664062, 0.0], [0.7998046875, 0.1000976562, 0.0]];
    #[cfg(feature = "fixed-point")]
    let gains: [[Val; 3]; 3] = [
        [qconst16(0.306_640_63, 15), qconst16(0.217_041_02, 15), qconst16(0.129_638_67, 15)],
        [qconst16(0.463_867_2, 15), qconst16(0.268_066_4, 15), qconst16(0.0, 15)],
        [qconst16(0.799_804_7, 15), qconst16(0.100_097_656, 15), qconst16(0.0, 15)],
    ];

    let g00 = mult16_16_q15(g0, gains[tapset0 as usize][0]) as Val;
    let g01 = mult16_16_q15(g0, gains[tapset0 as usize][1]) as Val;
    let g02 = mult16_16_q15(g0, gains[tapset0 as usize][2]) as Val;
    let g10 = mult16_16_q15(g1, gains[tapset1 as usize][0]) as Val;
    let g11 = mult16_16_q15(g1, gains[tapset1 as usize][1]) as Val;
    let g12 = mult16_16_q15(g1, gains[tapset1 as usize][2]) as Val;

    let t0 = t0 as usize;
    let t1 = t1 as usize;
    let mut y = y;

    for i in 0..overlap as usize {
        let f = mult16_16_q15(window[i], window[i]) as Val;
        let one_minus_f = (Q15ONE - f) as Val;
        let xi = x_off + i;
        let val = x[xi]
            + mult16_32_q15(mult16_16_q15(one_minus_f, g00) as Val, x[xi - t0])
            + mult16_32_q15(mult16_16_q15(one_minus_f, g01) as Val, x[xi - t0 - 1])
            + mult16_32_q15(mult16_16_q15(one_minus_f, g01) as Val, x[xi - t0 + 1])
            + mult16_32_q15(mult16_16_q15(one_minus_f, g02) as Val, x[xi - t0 - 2])
            + mult16_32_q15(mult16_16_q15(one_minus_f, g02) as Val, x[xi - t0 + 2])
            + mult16_32_q15(mult16_16_q15(f, g10) as Val, x[xi - t1])
            + mult16_32_q15(mult16_16_q15(f, g11) as Val, x[xi - t1 - 1])
            + mult16_32_q15(mult16_16_q15(f, g11) as Val, x[xi - t1 + 1])
            + mult16_32_q15(mult16_16_q15(f, g12) as Val, x[xi - t1 - 2])
            + mult16_32_q15(mult16_16_q15(f, g12) as Val, x[xi - t1 + 2]);
        match y {
            Some(ref mut out) => out[i] = val,
            None => x[xi] = val,
        }
    }
    for i in overlap as usize..n as usize {
        let xi = x_off + i;
        let val = x[xi]
            + mult16_32_q15(g10, x[xi - t1])
            + mult16_32_q15(g11, x[xi - t1 - 1])
            + mult16_32_q15(g11, x[xi - t1 + 1])
            + mult16_32_q15(g12, x[xi - t1 - 2])
            + mult16_32_q15(g12, x[xi - t1 + 2]);
        match y {
            Some(ref mut out) => out[i] = val,
            None => x[xi] = val,
        }
    }
}

// -- celt_decoder_ctl --

/// CELT decoder control requests (the subset the Opus decoder actually uses).
pub enum CeltDecCtl {
    SetStartBand(i32),
    SetEndBand(i32),
    SetChannels(i32),
    SetSignalling(i32),
    ResetState,
}

/// Apply a CELT decoder control request. Returns `OPUS_OK` or an error code.
pub fn celt_decoder_ctl(st: &mut CELTDecoder, request: CeltDecCtl) -> i32 {
    match request {
        CeltDecCtl::SetStartBand(value) => {
            if value < 0 || value >= st.mode.nb_ebands {
                return OPUS_BAD_ARG;
            }
            st.start = value;
        }
        CeltDecCtl::SetEndBand(value) => {
            if value < 1 || value > st.mode.nb_ebands {
                return OPUS_BAD_ARG;
            }
            st.end = value;
        }
        CeltDecCtl::SetChannels(value) => {
            if !(1..=2).contains(&value) {
                return OPUS_BAD_ARG;
            }
            st.stream_channels = value;
        }
        CeltDecCtl::SetSignalling(value) => {
            st.signalling = value;
        }
        CeltDecCtl::ResetState => {
            celt_decoder_reset(st);
        }
    }
    OPUS_OK
}
