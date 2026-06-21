// Translated from c/src/opus_decoder.c (RFC 6716).
//
// Opus decoder: top-level wrapper that dispatches between SILK (narrowband/
// mediumband/wideband voice) and CELT (fullband audio), handling hybrid mode,
// redundancy frames, mode transitions, and packet loss concealment.
//
// SILK and CELT are both fully in Rust.

use crate::arch::*;
use crate::celt::{CELTDecoder, CeltDecCtl, celt_decode_with_ec, celt_decoder_ctl, celt_decoder_init};
use crate::entcode::EcCtx;
use crate::entdec::{ec_dec_bit_logp, ec_dec_init, ec_dec_uint, ec_tell};
use crate::error::Error;
use crate::packet::{
    opus_packet_parse_impl, packet_get_bandwidth, packet_get_mode, packet_get_nb_channels, packet_get_samples_per_frame,
};
use crate::silk::dec_api::{SilkDecControlStruct, SilkDecoder, silk_decode, silk_init_decoder};
use crate::util::{OrPanic, zip};

// -- Constants --

const OPUS_OK: i32 = 0;
const OPUS_BAD_ARG: i32 = -1;
const OPUS_BUFFER_TOO_SMALL: i32 = -2;
const OPUS_INTERNAL_ERROR: i32 = -3;
const OPUS_INVALID_PACKET: i32 = -4;

const MODE_SILK_ONLY: i32 = 1000;
const MODE_HYBRID: i32 = 1001;
const MODE_CELT_ONLY: i32 = 1002;

const OPUS_BANDWIDTH_NARROWBAND: i32 = 1101;
const OPUS_BANDWIDTH_MEDIUMBAND: i32 = 1102;
const OPUS_BANDWIDTH_WIDEBAND: i32 = 1103;
const OPUS_BANDWIDTH_SUPERWIDEBAND: i32 = 1104;
const OPUS_BANDWIDTH_FULLBAND: i32 = 1105;

// Stack-scratch bounds for the no-alloc decode-frame buffers (largest values
// occur at 48 kHz, 2 channels).
const MAX_CHANNELS: usize = 2;
const MAX_F5: usize = 240; // 5 ms frame (fs/200)
const MAX_FRAME_DEC: usize = 2880; // largest single Opus frame: 60 ms (fs*60/1000)

#[cfg(not(feature = "fixed-point"))]
const CELT_SIG_SCALE: f32 = 32768.0;

// -- Public configuration types --

/// Output sample rate. Opus decodes to one of five rates; the enum makes any
/// other value unrepresentable, so decoder construction cannot fail on it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum SampleRate {
    Hz8000,
    Hz12000,
    Hz16000,
    Hz24000,
    Hz48000,
}

impl SampleRate {
    /// The rate in Hz.
    pub fn hz(self) -> i32 {
        match self {
            SampleRate::Hz8000 => 8000,
            SampleRate::Hz12000 => 12000,
            SampleRate::Hz16000 => 16000,
            SampleRate::Hz24000 => 24000,
            SampleRate::Hz48000 => 48000,
        }
    }

    /// Convert a raw Hz value into a `SampleRate`, or `None` if unsupported.
    pub fn from_hz(hz: i32) -> Option<SampleRate> {
        Some(match hz {
            8000 => SampleRate::Hz8000,
            12000 => SampleRate::Hz12000,
            16000 => SampleRate::Hz16000,
            24000 => SampleRate::Hz24000,
            48000 => SampleRate::Hz48000,
            _ => return None,
        })
    }
}

/// Channel layout: mono or stereo.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Channels {
    Mono,
    Stereo,
}

impl Channels {
    /// Number of channels (1 for mono, 2 for stereo).
    pub fn count(self) -> usize {
        match self {
            Channels::Mono => 1,
            Channels::Stereo => 2,
        }
    }

    /// Convert a raw channel count (1 or 2) into a `Channels`, or `None`.
    pub fn from_count(count: usize) -> Option<Channels> {
        match count {
            1 => Some(Channels::Mono),
            2 => Some(Channels::Stereo),
            _ => None,
        }
    }
}

// -- Decoder struct --

/// Top-level Opus decoder state.
///
/// The C version embeds the SILK and CELT decoder sub-states at computed byte
/// offsets within a single allocation (`silk_dec_offset` / `celt_dec_offset`).
/// Here they are plain struct fields and the whole decoder is constructed by
/// value via [`Decoder::new`] — no allocation, no offset arithmetic.
pub struct Decoder {
    channels: i32,
    fs: i32,
    dec_control: SilkDecControlStruct,

    stream_channels: i32,
    bandwidth: i32,
    mode: i32,
    prev_mode: i32,
    frame_size: i32,
    prev_redundancy: i32,

    range_final: u32,

    /// Embedded SILK decoder state (C: blob region at silk_dec_offset).
    silk_dec: SilkDecoder,
    /// Embedded CELT decoder state (C: blob region at celt_dec_offset).
    celt_dec: CELTDecoder,
}

// -- Public API --

impl Decoder {
    /// Create a decoder for the given output sample rate and channel layout.
    /// Constructed by value (no heap allocation) and infallible: the typed
    /// arguments make every configuration valid.
    pub fn new(sample_rate: SampleRate, channels: Channels) -> Decoder {
        let mut dec = Decoder {
            channels: 0,
            fs: 0,
            stream_channels: 0,
            bandwidth: 0,
            mode: 0,
            prev_mode: 0,
            frame_size: 0,
            prev_redundancy: 0,
            range_final: 0,
            dec_control: SilkDecControlStruct {
                n_channels_api: 0,
                n_channels_internal: 0,
                api_sample_rate: 0,
                internal_sample_rate: 0,
                payload_size_ms: 0,
                prev_pitch_lag: 0,
            },
            // Placeholder reset state; `init` fully overwrites it (the
            // per-channel states via `silk_init_decoder`, the super-header
            // fields explicitly) before any decode.
            silk_dec: SilkDecoder::default(),
            celt_dec: CELTDecoder::new(crate::modes::celt_mode(), 1),
        };
        // The typed arguments are always valid, so init cannot fail.
        assert_eq!(
            dec.init(sample_rate.hz(), channels.count() as i32),
            OPUS_OK,
            "decoder init failed for a valid configuration"
        );
        dec
    }

    /// (Re)initialise the decoder in place (logic of the C `opus_decoder_init`).
    fn init(&mut self, fs: i32, channels: i32) -> i32 {
        if (fs != 48000 && fs != 24000 && fs != 16000 && fs != 12000 && fs != 8000) || (channels != 1 && channels != 2) {
            return OPUS_BAD_ARG;
        }

        self.channels = channels;
        self.fs = fs;
        self.dec_control = SilkDecControlStruct {
            n_channels_api: channels,
            n_channels_internal: 0,
            api_sample_rate: fs,
            internal_sample_rate: 0,
            payload_size_ms: 0,
            prev_pitch_lag: 0,
        };
        self.stream_channels = channels;
        self.bandwidth = 0;
        self.mode = 0;
        self.prev_mode = 0;
        self.frame_size = fs / 400;
        self.prev_redundancy = 0;
        self.range_final = 0;

        // SilkDecoder super-header: the C relied on the caller's zeroed
        // allocation for these (silk_init_decoder only resets the per-channel
        // states, and the runtime CELT→SILK re-init path must not touch them).
        self.silk_dec.s_stereo.pred_prev_q13 = [0; 2];
        self.silk_dec.s_stereo.s_mid = [0; 2];
        self.silk_dec.s_stereo.s_side = [0; 2];
        self.silk_dec.n_channels_api = 0;
        self.silk_dec.n_channels_internal = 0;
        self.silk_dec.prev_decode_only_middle = 0;

        if silk_init_decoder(&mut self.silk_dec) != 0 {
            return OPUS_INTERNAL_ERROR;
        }
        if celt_decoder_init(&mut self.celt_dec, fs, channels) != OPUS_OK {
            return OPUS_INTERNAL_ERROR;
        }
        celt_decoder_ctl(&mut self.celt_dec, CeltDecCtl::SetSignalling(0));
        OPUS_OK
    }

    /// Decode one Opus packet into `pcm` (native sample type, interleaved by
    /// channel). `packet` is `None` for packet loss concealment. The output
    /// capacity in samples-per-channel is taken from `pcm.len()`. Returns the
    /// number of samples decoded per channel.
    pub fn decode(&mut self, packet: Option<&[u8]>, pcm: &mut [Val], fec: bool) -> Result<usize, Error> {
        let frame_size = (pcm.len() / self.channels as usize) as i32;
        let len = packet.map_or(0, |p| p.len()) as i32;
        let ret = opus_decode_native(self, packet, len, pcm, frame_size, fec as i32, 0, None);
        if ret < 0 { Err(Error::from_code(ret)) } else { Ok(ret as usize) }
    }

    /// Final range-coder state of the last decoded frame (for the
    /// encoder/decoder consistency check).
    pub fn final_range(&self) -> u32 {
        self.range_final
    }
}

/// Convert one decoded native sample to clamped 16-bit PCM. In the float
/// build this applies the standard scale + round; in the fixed-point build the
/// native sample is already `i16`.
#[cfg(not(feature = "fixed-point"))]
pub fn sample_to_i16(s: Val) -> i16 {
    float2int16(s)
}
#[cfg(feature = "fixed-point")]
pub fn sample_to_i16(s: Val) -> i16 {
    s
}

// -- smooth_fade --

/// Cross-fade between two signals over the overlap region using a squared
/// window. `in1`/`in2` of `None` means that input aliases `out` — the C
/// calls this in place (out == in1 or out == in2). Each sample reads both
/// inputs at `idx` before writing `out[idx]`, so reading the aliased value
/// from `out` is identical to reading it through the original pointer.
fn smooth_fade(
    in1: Option<&[Val]>,
    in2: Option<&[Val]>,
    out: &mut [Val],
    overlap: i32,
    channels: i32,
    window: &[Val],
    fs: i32,
) {
    let inc = 48000 / fs;
    for c in 0..channels {
        for i in 0..overlap {
            let idx = (i * channels + c) as usize;
            let wv = *window.get((i * inc) as usize).or_panic(i * inc);
            let w: Val = mult16_16_q15(wv, wv) as Val;
            let cur = *out.get(idx).or_panic(idx);
            let v1 = in1.map_or(cur, |s| *s.get(idx).or_panic(idx));
            let v2 = in2.map_or(cur, |s| *s.get(idx).or_panic(idx));
            *out.get_mut(idx).or_panic(idx) = shr32(mac16_16(mult16_16(w, v2), Q15ONE - w, v1), 15) as Val;
        }
    }
}

// -- FLOAT2INT16 (float mode only) --

#[cfg(all(not(feature = "std"), not(feature = "fixed-point")))]
use crate::arch::FloatMath;

#[cfg(not(feature = "fixed-point"))]
pub(crate) fn float2int16(x: f32) -> i16 {
    let x = x * CELT_SIG_SCALE;
    let x = if x > 32767.0 { 32767.0 } else { x };
    let x = if x < -32768.0 { -32768.0 } else { x };
    (x + 0.5).floor() as i16
}

// -- SAT16 (fixed-point mode only) --

#[cfg(feature = "fixed-point")]
fn sat16(x: i32) -> i16 {
    if x > 32767 {
        32767
    } else if x < -32768 {
        -32768
    } else {
        x as i16
    }
}

// -- opus_decode_frame --

fn opus_decode_frame(
    st: &mut Decoder,
    data: Option<&[u8]>,
    len: i32,
    pcm: &mut [Val],
    frame_size: i32,
    decode_fec: i32,
) -> i32 {
    let channels = st.channels;
    let mut silk_ret: i32;
    let mut celt_ret: i32 = 0;
    let mut dec = EcCtx::empty();
    let mut silk_frame_size: i32 = 0;

    let audiosize: i32;
    let mode: i32;
    let mut transition: i32 = 0;
    let mut redundancy: i32 = 0;
    let mut redundancy_bytes: i32 = 0;
    let mut celt_to_silk: i32 = 0;

    let f20 = st.fs / 50;
    let f10 = f20 >> 1;
    let f5 = f10 >> 1;
    let f2_5 = f5 >> 1;

    let mut data = data;
    let mut len = len;
    let mut frame_size = frame_size;

    if frame_size < f2_5 {
        return OPUS_BUFFER_TOO_SMALL;
    }

    // Payloads of 1 (2 including ToC) or 0 trigger the PLC/DTX
    if len <= 1 {
        data = None;
        // In that case, don't conceal more than what the ToC says
        frame_size = frame_size.min(st.frame_size);
    }

    if let Some(d) = data {
        audiosize = st.frame_size;
        mode = st.mode;
        ec_dec_init(&mut dec, d.get(..len as usize).or_panic(len), len as u32);
    } else {
        audiosize = frame_size;
        if st.prev_mode == 0 {
            // If we haven't got any packet yet, all we can do is return zeros
            for v in pcm.get_mut(..(audiosize * channels) as usize).or_panic(audiosize * channels) {
                *v = 0 as Val;
            }
            return audiosize;
        } else {
            mode = st.prev_mode;
        }
    }

    // For CELT/hybrid PLC of more than 20 ms, do multiple calls
    if data.is_none() && frame_size > f20 && mode != MODE_SILK_ONLY {
        let mut nb_samples: i32 = 0;
        loop {
            let ret = opus_decode_frame(
                st,
                None,
                0,
                pcm.get_mut((nb_samples * channels) as usize..).or_panic(nb_samples * channels),
                f20,
                0,
            );
            if ret != f20 {
                return OPUS_INTERNAL_ERROR;
            }
            nb_samples += f20;
            if nb_samples >= frame_size {
                break;
            }
        }
        return frame_size;
    }

    let mut pcm_transition_buf = [0 as Val; MAX_F5 * MAX_CHANNELS];
    let nt = (f5 * channels) as usize;
    let mut pcm_transition_buf = pcm_transition_buf.get_mut(..nt).or_panic(nt);

    if data.is_some()
        && st.prev_mode > 0
        && ((mode == MODE_CELT_ONLY && st.prev_mode != MODE_CELT_ONLY && st.prev_redundancy == 0)
            || (mode != MODE_CELT_ONLY && st.prev_mode == MODE_CELT_ONLY))
    {
        transition = 1;
        if mode == MODE_CELT_ONLY {
            opus_decode_frame(st, None, 0, &mut pcm_transition_buf, f5.min(audiosize), 0);
        }
    }

    if audiosize > frame_size {
        return OPUS_BAD_ARG;
    } else {
        frame_size = audiosize;
    }

    let mut pcm_silk_buf = [0i16; MAX_FRAME_DEC * MAX_CHANNELS];
    let ns = (f10.max(frame_size) * channels) as usize;
    let pcm_silk_buf = pcm_silk_buf.get_mut(..ns).or_panic(ns);
    let mut redundant_audio_buf = [0 as Val; MAX_F5 * MAX_CHANNELS];
    let nr = (f5 * channels) as usize;
    let mut redundant_audio_buf = redundant_audio_buf.get_mut(..nr).or_panic(nr);

    let mut redundant_rng: u32 = 0;

    // SILK processing
    if mode != MODE_CELT_ONLY {
        let lost_flag: i32;
        let mut decoded_samples: i32;

        if st.prev_mode == MODE_CELT_ONLY {
            silk_init_decoder(&mut st.silk_dec);
        }

        // The SILK PLC cannot produce frames of less than 10 ms
        st.dec_control.payload_size_ms = (1000 * audiosize / st.fs).max(10);

        if data.is_some() {
            st.dec_control.n_channels_internal = st.stream_channels;
            if mode == MODE_SILK_ONLY {
                if st.bandwidth == OPUS_BANDWIDTH_NARROWBAND {
                    st.dec_control.internal_sample_rate = 8000;
                } else if st.bandwidth == OPUS_BANDWIDTH_MEDIUMBAND {
                    st.dec_control.internal_sample_rate = 12000;
                } else if st.bandwidth == OPUS_BANDWIDTH_WIDEBAND {
                    st.dec_control.internal_sample_rate = 16000;
                } else {
                    st.dec_control.internal_sample_rate = 16000;
                }
            } else {
                // Hybrid mode
                st.dec_control.internal_sample_rate = 16000;
            }
        }

        lost_flag = if data.is_none() { 1 } else { 2 * decode_fec };
        decoded_samples = 0;
        loop {
            // Call SILK decoder
            let first_frame = if decoded_samples == 0 { 1 } else { 0 };
            let silk_off = (decoded_samples * channels) as usize;
            silk_ret = silk_decode(
                &mut st.silk_dec,
                &mut st.dec_control,
                lost_flag,
                first_frame,
                &mut dec,
                pcm_silk_buf.get_mut(silk_off..).or_panic(silk_off),
                &mut silk_frame_size,
            );
            if silk_ret != 0 {
                if lost_flag != 0 {
                    // PLC failure should not be fatal
                    silk_frame_size = frame_size;
                    for v in pcm_silk_buf
                        .get_mut(silk_off..silk_off + (frame_size * channels) as usize)
                        .or_panic_dbg((silk_off, frame_size * channels))
                    {
                        *v = 0;
                    }
                } else {
                    return OPUS_INVALID_PACKET;
                }
            }
            decoded_samples += silk_frame_size;
            if decoded_samples >= frame_size {
                break;
            }
        }
    }

    let mut start_band = 0;
    if decode_fec == 0
        && mode != MODE_CELT_ONLY
        && data.is_some()
        && ec_tell(&dec) + 17 + 20 * (if st.mode == MODE_HYBRID { 1 } else { 0 }) <= 8 * len
    {
        // Check if we have a redundant 0-8 kHz band
        if mode == MODE_HYBRID {
            redundancy = ec_dec_bit_logp(&mut dec, 12);
        } else {
            redundancy = 1;
        }
        if redundancy != 0 {
            celt_to_silk = ec_dec_bit_logp(&mut dec, 1);
            // redundancy_bytes will be at least two, in the non-hybrid
            // case due to the ec_tell() check above
            redundancy_bytes =
                if mode == MODE_HYBRID { ec_dec_uint(&mut dec, 256) as i32 + 2 } else { len - ((ec_tell(&dec) + 7) >> 3) };
            len -= redundancy_bytes;
            // Sanity check
            if len * 8 < ec_tell(&dec) {
                len = 0;
                redundancy_bytes = 0;
                redundancy = 0;
            }
            // Shrink decoder because of raw bits
            dec.storage -= redundancy_bytes as u32;
        }
    }
    if mode != MODE_CELT_ONLY {
        start_band = 17;
    }

    {
        let endband: i32 = match st.bandwidth {
            OPUS_BANDWIDTH_NARROWBAND => 13,
            OPUS_BANDWIDTH_MEDIUMBAND | OPUS_BANDWIDTH_WIDEBAND => 17,
            OPUS_BANDWIDTH_SUPERWIDEBAND => 19,
            OPUS_BANDWIDTH_FULLBAND => 21,
            _ => 21,
        };
        let stream_channels = st.stream_channels;
        celt_decoder_ctl(&mut st.celt_dec, CeltDecCtl::SetEndBand(endband));
        celt_decoder_ctl(&mut st.celt_dec, CeltDecCtl::SetChannels(stream_channels));
    }

    if redundancy != 0 {
        transition = 0;
    }

    if transition != 0 && mode != MODE_CELT_ONLY {
        opus_decode_frame(st, None, 0, &mut pcm_transition_buf, f5.min(audiosize), 0);
    }

    // 5 ms redundant frame for CELT->SILK
    if redundancy != 0 && celt_to_silk != 0 {
        celt_decoder_ctl(&mut st.celt_dec, CeltDecCtl::SetStartBand(0));
        let d = data.or_panic("redundancy set but data is None");
        celt_decode_with_ec(
            &mut st.celt_dec,
            Some(d.get(len as usize..(len + redundancy_bytes) as usize).or_panic_dbg((len, redundancy_bytes))),
            redundancy_bytes,
            &mut redundant_audio_buf,
            f5,
            None,
        );
        redundant_rng = st.celt_dec.final_range();
    }

    // MUST be after PLC
    celt_decoder_ctl(&mut st.celt_dec, CeltDecCtl::SetStartBand(start_band));

    if mode != MODE_SILK_ONLY {
        let celt_frame_size = f20.min(frame_size);
        // Make sure to discard any previous CELT state
        if mode != st.prev_mode && st.prev_mode > 0 && st.prev_redundancy == 0 {
            celt_decoder_ctl(&mut st.celt_dec, CeltDecCtl::ResetState);
        }
        // Decode CELT
        let celt_data = if decode_fec != 0 { None } else { data.map(|d| d.get(..len as usize).or_panic(len)) };
        celt_ret = celt_decode_with_ec(&mut st.celt_dec, celt_data, len, pcm, celt_frame_size, Some(&mut dec));
    } else {
        let silence: [u8; 2] = [0xFF, 0xFF];
        for v in pcm.get_mut(..(frame_size * channels) as usize).or_panic(frame_size * channels) {
            *v = 0 as Val;
        }
        // For hybrid -> SILK transitions, we let the CELT MDCT
        // do a fade-out by decoding a silence frame
        if st.prev_mode == MODE_HYBRID && !(redundancy != 0 && celt_to_silk != 0 && st.prev_redundancy != 0) {
            celt_decoder_ctl(&mut st.celt_dec, CeltDecCtl::SetStartBand(0));
            celt_decode_with_ec(&mut st.celt_dec, Some(&silence), 2, pcm, f2_5, None);
        }
    }

    if mode != MODE_CELT_ONLY {
        let n = (frame_size * channels) as usize;
        let pcm_n = pcm.get_mut(..n).or_panic(n);
        let silk_n = pcm_silk_buf.get(..n).or_panic(n);
        #[cfg(feature = "fixed-point")]
        for (p, &s) in zip(pcm_n, silk_n) {
            *p = sat16(*p as i32 + s as i32);
        }
        #[cfg(not(feature = "fixed-point"))]
        for (p, &s) in zip(pcm_n, silk_n) {
            *p += (1.0 / 32768.0) * s as f32;
        }
    }

    let window = st.celt_dec.mode.window;

    // 5 ms redundant frame for SILK->CELT
    if redundancy != 0 && celt_to_silk == 0 {
        celt_decoder_ctl(&mut st.celt_dec, CeltDecCtl::ResetState);
        celt_decoder_ctl(&mut st.celt_dec, CeltDecCtl::SetStartBand(0));

        let d = data.or_panic("redundancy set but data is None");
        celt_decode_with_ec(
            &mut st.celt_dec,
            Some(d.get(len as usize..(len + redundancy_bytes) as usize).or_panic_dbg((len, redundancy_bytes))),
            redundancy_bytes,
            &mut redundant_audio_buf,
            f5,
            None,
        );
        redundant_rng = st.celt_dec.final_range();
        let off = (channels * (frame_size - f2_5)) as usize;
        smooth_fade(
            None,
            Some(redundant_audio_buf.get((channels * f2_5) as usize..).or_panic(channels * f2_5)),
            pcm.get_mut(off..).or_panic(off),
            f2_5,
            channels,
            window,
            st.fs,
        );
    }
    if redundancy != 0 && celt_to_silk != 0 {
        // Every index in [0, channels*f2_5) is written exactly once, so this is
        // a flat copy of that prefix.
        let k = (channels * f2_5) as usize;
        let dst = pcm.get_mut(..k).or_panic(k);
        dst.copy_from_slice(redundant_audio_buf.get(..k).or_panic(k));
        smooth_fade(
            Some(redundant_audio_buf.get((channels * f2_5) as usize..).or_panic(channels * f2_5)),
            None,
            pcm.get_mut((channels * f2_5) as usize..).or_panic(channels * f2_5),
            f2_5,
            channels,
            window,
            st.fs,
        );
    }
    if transition != 0 {
        if audiosize >= f5 {
            let k = (channels * f2_5) as usize;
            pcm.get_mut(..k).or_panic(k).copy_from_slice(pcm_transition_buf.get(..k).or_panic(k));
            smooth_fade(
                Some(pcm_transition_buf.get((channels * f2_5) as usize..).or_panic(channels * f2_5)),
                None,
                pcm.get_mut((channels * f2_5) as usize..).or_panic(channels * f2_5),
                f2_5,
                channels,
                window,
                st.fs,
            );
        } else {
            smooth_fade(Some(&pcm_transition_buf), None, pcm, f2_5, channels, window, st.fs);
        }
    }

    if len <= 1 {
        st.range_final = 0;
    } else {
        st.range_final = dec.rng ^ redundant_rng;
    }

    st.prev_mode = mode;
    st.prev_redundancy = if redundancy != 0 && celt_to_silk == 0 { 1 } else { 0 };
    if celt_ret < 0 { celt_ret } else { audiosize }
}
// -- opus_decode_native --

pub fn opus_decode_native(
    st: &mut Decoder,
    data: Option<&[u8]>,
    len: i32,
    pcm: &mut [Val],
    frame_size: i32,
    decode_fec: i32,
    self_delimited: i32,
    packet_offset: Option<&mut i32>,
) -> i32 {
    let mut offset: i32 = 0;
    let mut toc: u8 = 0;
    let mut size: [i16; 48] = [0i16; 48];

    if !(0..=1).contains(&decode_fec) {
        return OPUS_BAD_ARG;
    }
    let data = match data {
        Some(d) if len != 0 => d,
        _ => {
            return opus_decode_frame(st, None, 0, pcm, frame_size, 0);
        }
    };
    if len < 0 {
        return OPUS_BAD_ARG;
    }

    let mut tot_offset = 0;
    let toc_byte = *data.first().or_panic("opus_decode_native: empty packet");
    st.mode = packet_get_mode(toc_byte);
    st.bandwidth = packet_get_bandwidth(toc_byte);
    st.frame_size = packet_get_samples_per_frame(toc_byte, st.fs);
    st.stream_channels = packet_get_nb_channels(toc_byte);

    let count = opus_packet_parse_impl(
        data.get(..len as usize).or_panic(len),
        self_delimited,
        Some(&mut toc),
        None,
        &mut size,
        Some(&mut offset),
    );
    if count < 0 {
        return count;
    }

    let mut data_off = offset as usize;
    tot_offset += offset;

    if count * st.frame_size > frame_size {
        return OPUS_BUFFER_TOO_SMALL;
    }
    let channels = st.channels;
    let mut nb_samples = 0;
    let mut pcm_off = 0usize;
    let mut i = 0;
    while i < count {
        let sz = *size.get(i as usize).or_panic(i) as i32;
        let ret = opus_decode_frame(
            st,
            Some(data.get(data_off..data_off + sz as usize).or_panic_dbg((data_off, sz))),
            sz,
            pcm.get_mut(pcm_off..).or_panic(pcm_off),
            frame_size - nb_samples,
            decode_fec,
        );
        if ret < 0 {
            return ret;
        }
        data_off += sz as usize;
        tot_offset += sz;
        pcm_off += (ret * channels) as usize;
        nb_samples += ret;
        i += 1;
    }
    if let Some(po) = packet_offset {
        *po = tot_offset;
    }
    nb_samples
}
