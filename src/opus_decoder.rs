// Translated from c/src/opus_decoder.c (RFC 6716).
//
// Opus decoder: top-level wrapper that dispatches between SILK (narrowband/
// mediumband/wideband voice) and CELT (fullband audio), handling hybrid mode,
// redundancy frames, mode transitions, and packet loss concealment.
//
// SILK is kept as a C unit and called via FFI. CELT is fully in Rust.

use std::os::raw::c_int;

use crate::arch::*;
use crate::celt::{CELTDecoder, CeltDecCtl, celt_decode_with_ec, celt_decoder_ctl, celt_decoder_init};
use crate::entcode::EcCtx;
use crate::entdec::{ec_dec_bit_logp, ec_dec_init, ec_dec_uint, ec_tell};
use crate::packet::{
    opus_packet_get_nb_frames, opus_packet_get_samples_per_frame, opus_packet_parse_impl, packet_get_bandwidth,
    packet_get_mode, packet_get_nb_channels, packet_get_samples_per_frame,
};
use crate::silk::dec_api::{SilkDecControlStruct, SilkDecoder, silk_decode, silk_init_decoder};
use crate::util::{OrPanic, zip};

// -- Constants --

const OPUS_OK: c_int = 0;
const OPUS_BAD_ARG: c_int = -1;
const OPUS_BUFFER_TOO_SMALL: c_int = -2;
const OPUS_INTERNAL_ERROR: c_int = -3;
const OPUS_INVALID_PACKET: c_int = -4;
const OPUS_ALLOC_FAIL: c_int = -7;

const MODE_SILK_ONLY: c_int = 1000;
const MODE_HYBRID: c_int = 1001;
const MODE_CELT_ONLY: c_int = 1002;

const OPUS_BANDWIDTH_NARROWBAND: c_int = 1101;
const OPUS_BANDWIDTH_MEDIUMBAND: c_int = 1102;
const OPUS_BANDWIDTH_WIDEBAND: c_int = 1103;
const OPUS_BANDWIDTH_SUPERWIDEBAND: c_int = 1104;
const OPUS_BANDWIDTH_FULLBAND: c_int = 1105;

const OPUS_GET_BANDWIDTH_REQUEST: c_int = 4009;
const OPUS_GET_FINAL_RANGE_REQUEST: c_int = 4031;
const OPUS_RESET_STATE: c_int = 4028;
const OPUS_GET_PITCH_REQUEST: c_int = 4033;

#[cfg(not(feature = "fixed-point"))]
const CELT_SIG_SCALE: f32 = 32768.0;

// -- Alignment helper (matches C align()) --

pub(crate) fn align(i: usize) -> usize {
    (i + std::mem::size_of::<*const ()>() - 1) & !(std::mem::size_of::<*const ()>() - 1)
}

// -- OpusDecoder struct --

/// Top-level Opus decoder state.
///
/// The C version embeds SILK and CELT decoder sub-states at computed byte
/// offsets within a single allocation (`silk_dec_offset` /
/// `celt_dec_offset`). Here they are plain struct fields; the public
/// C-style API (get_size / init / create) still treats the whole thing
/// as one allocation, but no offset arithmetic remains.
pub struct OpusDecoder {
    pub channels: c_int,
    pub fs: i32,
    pub dec_control: SilkDecControlStruct,

    pub stream_channels: c_int,
    pub bandwidth: c_int,
    pub mode: c_int,
    pub prev_mode: c_int,
    pub frame_size: c_int,
    pub prev_redundancy: c_int,

    pub range_final: u32,

    /// Embedded SILK decoder state (C: blob region at silk_dec_offset).
    pub silk_dec: SilkDecoder,
    /// Embedded CELT decoder state (C: blob region at celt_dec_offset).
    pub celt_dec: CELTDecoder,
}

// -- Public API --

#[unsafe(no_mangle)]
pub extern "C" fn opus_decoder_get_size(channels: c_int) -> c_int {
    if !(1..=2).contains(&channels) {
        return 0;
    }
    std::mem::size_of::<OpusDecoder>() as c_int
}

/// Initialise a previously-allocated `OpusDecoder`.
///
/// The buffer pointed to by `st` must be [`opus_decoder_get_size`]
/// bytes. Everything is initialised explicitly: the header fields, the
/// SilkDecoder super-header (which the C left to the caller's zeroed
/// allocation), and the per-channel SILK / CELT sub-states.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_decoder_init(st: *mut OpusDecoder, fs: i32, channels: c_int) -> c_int {
    unsafe {
        if (fs != 48000 && fs != 24000 && fs != 16000 && fs != 12000 && fs != 8000) || (channels != 1 && channels != 2) {
            return OPUS_BAD_ARG;
        }

        (*st).channels = channels;
        (*st).fs = fs;
        (*st).dec_control = SilkDecControlStruct {
            n_channels_api: channels,
            n_channels_internal: 0,
            api_sample_rate: fs,
            internal_sample_rate: 0,
            payload_size_ms: 0,
            prev_pitch_lag: 0,
        };
        (*st).stream_channels = channels;
        (*st).bandwidth = 0;
        (*st).mode = 0;
        (*st).prev_mode = 0;
        (*st).frame_size = fs / 400;
        (*st).prev_redundancy = 0;
        (*st).range_final = 0;

        // SilkDecoder super-header: the C relied on the caller's zeroed
        // allocation for these (silk_init_decoder only resets the
        // per-channel states, and the runtime CELT→SILK re-init path
        // must not touch them either).
        (*st).silk_dec.s_stereo.pred_prev_q13 = [0; 2];
        (*st).silk_dec.s_stereo.s_mid = [0; 2];
        (*st).silk_dec.s_stereo.s_side = [0; 2];
        (*st).silk_dec.n_channels_api = 0;
        (*st).silk_dec.n_channels_internal = 0;
        (*st).silk_dec.prev_decode_only_middle = 0;

        let ret = silk_init_decoder(&mut (*st).silk_dec);
        if ret != 0 {
            return OPUS_INTERNAL_ERROR;
        }

        let ret = celt_decoder_init(&mut (*st).celt_dec, fs, channels);
        if ret != OPUS_OK {
            return OPUS_INTERNAL_ERROR;
        }

        celt_decoder_ctl(&mut (*st).celt_dec, CeltDecCtl::SetSignalling(0));

        OPUS_OK
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_decoder_create(fs: i32, channels: c_int, error: *mut c_int) -> *mut OpusDecoder {
    unsafe {
        if (fs != 48000 && fs != 24000 && fs != 16000 && fs != 12000 && fs != 8000) || (channels != 1 && channels != 2) {
            if !error.is_null() {
                *error = OPUS_BAD_ARG;
            }
            return std::ptr::null_mut();
        }
        let size = opus_decoder_get_size(channels) as usize;
        let layout = std::alloc::Layout::from_size_align(size, std::mem::align_of::<OpusDecoder>())
            .unwrap_or_else(|e| panic!("invalid layout for OpusDecoder: {e:?}"));
        let ptr = std::alloc::alloc_zeroed(layout) as *mut OpusDecoder;
        if ptr.is_null() {
            if !error.is_null() {
                *error = OPUS_ALLOC_FAIL;
            }
            return std::ptr::null_mut();
        }
        let ret = opus_decoder_init(ptr, fs, channels);
        if !error.is_null() {
            *error = ret;
        }
        if ret != OPUS_OK {
            std::alloc::dealloc(ptr as *mut u8, layout);
            return std::ptr::null_mut();
        }
        ptr
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_decoder_destroy(st: *mut OpusDecoder) {
    unsafe {
        if st.is_null() {
            return;
        }
        let channels = (*st).channels;
        let size = opus_decoder_get_size(channels) as usize;
        let layout = std::alloc::Layout::from_size_align(size, std::mem::align_of::<OpusDecoder>())
            .unwrap_or_else(|e| panic!("invalid layout for OpusDecoder: {e:?}"));
        std::alloc::dealloc(st as *mut u8, layout);
    }
}

// -- smooth_fade --

/// Cross-fade between two signals over the overlap region using a squared
/// window. `in1`/`in2` of `None` means that input aliases `out` — the C
/// calls this in place (out == in1 or out == in2). Each sample reads both
/// inputs at `idx` before writing `out[idx]`, so reading the aliased value
/// from `out` is identical to reading it through the original pointer.
fn smooth_fade(
    in1: Option<&[OpusVal16]>,
    in2: Option<&[OpusVal16]>,
    out: &mut [OpusVal16],
    overlap: c_int,
    channels: c_int,
    window: &[OpusVal16],
    fs: i32,
) {
    let inc = 48000 / fs;
    for c in 0..channels {
        for i in 0..overlap {
            let idx = (i * channels + c) as usize;
            let wv = *window.get((i * inc) as usize).or_panic(i * inc);
            let w: OpusVal16 = mult16_16_q15(wv, wv) as OpusVal16;
            let cur = *out.get(idx).or_panic(idx);
            let v1 = in1.map_or(cur, |s| *s.get(idx).or_panic(idx));
            let v2 = in2.map_or(cur, |s| *s.get(idx).or_panic(idx));
            *out.get_mut(idx).or_panic(idx) = shr32(mac16_16(mult16_16(w, v2), Q15ONE - w, v1), 15) as OpusVal16;
        }
    }
}

// -- FLOAT2INT16 (float mode only) --

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
    st: &mut OpusDecoder,
    data: Option<&[u8]>,
    len: c_int,
    pcm: &mut [OpusVal16],
    frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    let channels = st.channels;
    let mut silk_ret: c_int;
    let mut celt_ret: c_int = 0;
    let mut dec = EcCtx::empty();
    let mut silk_frame_size: i32 = 0;

    let audiosize: c_int;
    let mode: c_int;
    let mut transition: c_int = 0;
    let mut redundancy: c_int = 0;
    let mut redundancy_bytes: c_int = 0;
    let mut celt_to_silk: c_int = 0;

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
                *v = 0 as OpusVal16;
            }
            return audiosize;
        } else {
            mode = st.prev_mode;
        }
    }

    // For CELT/hybrid PLC of more than 20 ms, do multiple calls
    if data.is_none() && frame_size > f20 && mode != MODE_SILK_ONLY {
        let mut nb_samples: c_int = 0;
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

    let mut pcm_transition_buf: Vec<OpusVal16> = vec![0 as OpusVal16; (f5 * channels) as usize];

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

    let mut pcm_silk_buf: Vec<i16> = vec![0i16; (f10.max(frame_size) * channels) as usize];
    let mut redundant_audio_buf: Vec<OpusVal16> = vec![0 as OpusVal16; (f5 * channels) as usize];

    let mut redundant_rng: u32 = 0;

    // SILK processing
    if mode != MODE_CELT_ONLY {
        let lost_flag: c_int;
        let mut decoded_samples: c_int;

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
                if mode == MODE_HYBRID { ec_dec_uint(&mut dec, 256) as c_int + 2 } else { len - ((ec_tell(&dec) + 7) >> 3) };
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
        let endband: c_int = match st.bandwidth {
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
        let d = data.or_panic("redundancy is only set in the data-present branch");
        celt_decode_with_ec(
            &mut st.celt_dec,
            Some(d.get(len as usize..(len + redundancy_bytes) as usize).or_panic_dbg((len, redundancy_bytes))),
            redundancy_bytes,
            &mut redundant_audio_buf,
            f5,
            None,
        );
        celt_decoder_ctl(&mut st.celt_dec, CeltDecCtl::GetFinalRange(&mut redundant_rng));
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
            *v = 0 as OpusVal16;
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

        let d = data.or_panic("redundancy is only set in the data-present branch");
        celt_decode_with_ec(
            &mut st.celt_dec,
            Some(d.get(len as usize..(len + redundancy_bytes) as usize).or_panic_dbg((len, redundancy_bytes))),
            redundancy_bytes,
            &mut redundant_audio_buf,
            f5,
            None,
        );
        celt_decoder_ctl(&mut st.celt_dec, CeltDecCtl::GetFinalRange(&mut redundant_rng));
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
    st: &mut OpusDecoder,
    data: Option<&[u8]>,
    len: c_int,
    pcm: &mut [OpusVal16],
    frame_size: c_int,
    decode_fec: c_int,
    self_delimited: c_int,
    packet_offset: Option<&mut c_int>,
) -> c_int {
    let mut offset: c_int = 0;
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
        let sz = *size.get(i as usize).or_panic(i) as c_int;
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

// -- opus_decode --

/// Build an `Option<&[u8]>` view of a caller's `(data, len)`: `None` for a
/// null pointer (PLC), else a slice of `max(len, 0)` bytes (a negative len
/// yields an empty slice that the native decoder rejects before indexing).
///
/// # Safety
/// `data`, if non-null, must point to at least `len` readable bytes.
unsafe fn data_view<'a>(data: *const u8, len: c_int) -> Option<&'a [u8]> {
    if data.is_null() { None } else { Some(unsafe { core::slice::from_raw_parts(data, len.max(0) as usize) }) }
}

#[cfg(feature = "fixed-point")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_decode(
    st: *mut OpusDecoder,
    data: *const u8,
    len: c_int,
    pcm: *mut OpusVal16,
    frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    unsafe {
        let channels = (*st).channels;
        let pcm = core::slice::from_raw_parts_mut(pcm, (frame_size.max(0) * channels) as usize);
        opus_decode_native(&mut *st, data_view(data, len), len, pcm, frame_size, decode_fec, 0, None)
    }
}

#[cfg(not(feature = "fixed-point"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_decode(
    st: *mut OpusDecoder,
    data: *const u8,
    len: c_int,
    pcm: *mut i16,
    frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    unsafe {
        if frame_size < 0 {
            return OPUS_BAD_ARG;
        }

        let channels = (*st).channels;
        let mut out_buf: Vec<f32> = vec![0.0f32; (frame_size * channels) as usize];
        let ret = opus_decode_native(&mut *st, data_view(data, len), len, &mut out_buf, frame_size, decode_fec, 0, None);
        if ret > 0 {
            for (i, &v) in out_buf.get(..(ret * channels) as usize).or_panic(ret * channels).iter().enumerate() {
                *pcm.add(i) = float2int16(v);
            }
        }
        ret
    }
}

// -- opus_decode_float --

#[cfg(not(feature = "fixed-point"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_decode_float(
    st: *mut OpusDecoder,
    data: *const u8,
    len: c_int,
    pcm: *mut OpusVal16,
    frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    unsafe {
        let channels = (*st).channels;
        let pcm = core::slice::from_raw_parts_mut(pcm, (frame_size.max(0) * channels) as usize);
        opus_decode_native(&mut *st, data_view(data, len), len, pcm, frame_size, decode_fec, 0, None)
    }
}

#[cfg(feature = "fixed-point")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_decode_float(
    st: *mut OpusDecoder,
    data: *const u8,
    len: c_int,
    pcm: *mut f32,
    frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    unsafe {
        let channels = (*st).channels;
        let mut out_buf: Vec<i16> = vec![0i16; (frame_size * channels) as usize];
        let ret = opus_decode_native(&mut *st, data_view(data, len), len, &mut out_buf, frame_size, decode_fec, 0, None);
        if ret > 0 {
            let mut i = 0;
            while i < ret * channels {
                *pcm.offset(i as isize) = (1.0f32 / 32768.0f32) * out_buf[i as usize] as f32;
                i += 1;
            }
        }
        ret
    }
}

// -- opus_decoder_ctl --

/// FFI-safe tagged enum for Opus decoder CTL requests.
#[repr(C, i32)]
pub enum OpusDecCtl {
    GetBandwidth(*mut c_int) = OPUS_GET_BANDWIDTH_REQUEST,
    GetFinalRange(*mut u32) = OPUS_GET_FINAL_RANGE_REQUEST,
    ResetState = OPUS_RESET_STATE,
    GetPitch(*mut c_int) = OPUS_GET_PITCH_REQUEST,
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_decoder_ctl(st: *mut OpusDecoder, request: OpusDecCtl) -> c_int {
    unsafe {
        let silk_dec: *mut SilkDecoder = &raw mut (*st).silk_dec;
        let celt_dec: *mut CELTDecoder = &raw mut (*st).celt_dec;

        match request {
            OpusDecCtl::GetBandwidth(value) => {
                *value = (*st).bandwidth;
            }
            OpusDecCtl::GetFinalRange(value) => {
                *value = (*st).range_final;
            }
            OpusDecCtl::ResetState => {
                let OpusDecoder {
                    channels,
                    fs,
                    dec_control: _,
                    stream_channels,
                    bandwidth,
                    mode,
                    prev_mode,
                    frame_size,
                    prev_redundancy,
                    range_final,
                    silk_dec: _,
                    celt_dec: _,
                } = &mut *st;

                *bandwidth = 0;
                *mode = 0;
                *prev_mode = 0;
                *prev_redundancy = 0;
                *range_final = 0;
                *stream_channels = *channels;
                *frame_size = *fs / 400;

                celt_decoder_ctl(&mut *celt_dec, CeltDecCtl::ResetState);
                silk_init_decoder(&mut *silk_dec);
            }
            OpusDecCtl::GetPitch(value) => {
                if value.is_null() {
                    return OPUS_BAD_ARG;
                }
                if (*st).prev_mode == MODE_CELT_ONLY {
                    celt_decoder_ctl(&mut *celt_dec, CeltDecCtl::GetPitch(value));
                } else {
                    *value = (*st).dec_control.prev_pitch_lag;
                }
            }
        }
        OPUS_OK
    }
}

// -- opus_decoder_get_nb_samples --

#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_decoder_get_nb_samples(dec: *const OpusDecoder, packet: *const u8, len: c_int) -> c_int {
    unsafe {
        let count = opus_packet_get_nb_frames(packet, len);
        if count < 0 {
            return count;
        }
        let samples = count * opus_packet_get_samples_per_frame(packet, (*dec).fs);
        // Can't have more than 120 ms
        if samples * 25 > (*dec).fs * 3 { OPUS_INVALID_PACKET } else { samples }
    }
}
