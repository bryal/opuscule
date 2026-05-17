// Translated from c/src/opus_decoder.c (RFC 6716).
//
// Opus decoder: top-level wrapper that dispatches between SILK (narrowband/
// mediumband/wideband voice) and CELT (fullband audio), handling hybrid mode,
// redundancy frames, mode transitions, and packet loss concealment.
//
// SILK is kept as a C unit and called via FFI. CELT is fully in Rust.

use std::os::raw::c_int;

use crate::arch::*;
use crate::celt::{CELTDecoder, CeltDecCtl, celt_decode_with_ec, celt_decoder_ctl, celt_decoder_get_size, celt_decoder_init};
use crate::entcode::ec_ctx;
use crate::entdec::{ec_dec_bit_logp, ec_dec_init, ec_dec_uint, ec_tell};
use crate::modes::CELTMode;
use crate::packet::{
    opus_packet_get_bandwidth, opus_packet_get_mode, opus_packet_get_nb_channels, opus_packet_get_nb_frames,
    opus_packet_get_samples_per_frame, opus_packet_parse_impl,
};
use crate::silk::dec_API::{SilkDecControlStruct, silk_decode, silk_get_decoder_size, silk_init_decoder};

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
/// offsets within a single allocation. The offsets are stored in
/// `silk_dec_offset` / `celt_dec_offset`; this struct's own field order
/// is not load-bearing.
pub struct OpusDecoder {
    pub celt_dec_offset: c_int,
    pub silk_dec_offset: c_int,
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
}

// -- Helper to get sub-decoder pointers --

unsafe fn silk_dec_ptr(st: *mut OpusDecoder) -> *mut u8 {
    (st as *mut u8).add((*st).silk_dec_offset as usize)
}

unsafe fn celt_dec_ptr(st: *mut OpusDecoder) -> *mut CELTDecoder {
    (st as *mut u8).add((*st).celt_dec_offset as usize) as *mut CELTDecoder
}

// -- Public API --

#[unsafe(no_mangle)]
pub extern "C" fn opus_decoder_get_size(channels: c_int) -> c_int {
    if channels < 1 || channels > 2 {
        return 0;
    }
    let mut silk_dec_size_bytes: c_int = 0;
    let ret = unsafe { silk_get_decoder_size(&mut silk_dec_size_bytes) };
    if ret != 0 {
        return 0;
    }
    let silk_dec_size_bytes = align(silk_dec_size_bytes as usize);
    let celt_dec_size_bytes = celt_decoder_get_size(channels) as usize;
    (align(std::mem::size_of::<OpusDecoder>()) + silk_dec_size_bytes + celt_dec_size_bytes) as c_int
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_decoder_init(st: *mut OpusDecoder, fs: i32, channels: c_int) -> c_int {
    unsafe {
        if (fs != 48000 && fs != 24000 && fs != 16000 && fs != 12000 && fs != 8000) || (channels != 1 && channels != 2) {
            return OPUS_BAD_ARG;
        }

        let total_size = opus_decoder_get_size(channels) as usize;
        std::ptr::write_bytes(st as *mut u8, 0, total_size);

        let mut silk_dec_size_bytes: c_int = 0;
        let ret = silk_get_decoder_size(&mut silk_dec_size_bytes);
        if ret != 0 {
            return OPUS_INTERNAL_ERROR;
        }

        let silk_dec_size_bytes = align(silk_dec_size_bytes as usize);
        (*st).silk_dec_offset = align(std::mem::size_of::<OpusDecoder>()) as c_int;
        (*st).celt_dec_offset = (*st).silk_dec_offset + silk_dec_size_bytes as c_int;
        let silk_dec = silk_dec_ptr(st);
        let celt_dec = celt_dec_ptr(st);
        (*st).channels = channels;
        (*st).stream_channels = channels;

        (*st).fs = fs;
        (*st).dec_control.api_sample_rate = fs;
        (*st).dec_control.n_channels_api = channels;

        let ret = silk_init_decoder(silk_dec);
        if ret != 0 {
            return OPUS_INTERNAL_ERROR;
        }

        let ret = celt_decoder_init(celt_dec, fs, channels);
        if ret != OPUS_OK {
            return OPUS_INTERNAL_ERROR;
        }

        celt_decoder_ctl(celt_dec, CeltDecCtl::SetSignalling(0));

        (*st).prev_mode = 0;
        (*st).frame_size = fs / 400;
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
            .expect("invalid layout for OpusDecoder");
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
            .expect("invalid layout for OpusDecoder");
        std::alloc::dealloc(st as *mut u8, layout);
    }
}

// -- smooth_fade --

unsafe fn smooth_fade(
    in1: *const OpusVal16,
    in2: *const OpusVal16,
    out: *mut OpusVal16,
    overlap: c_int,
    channels: c_int,
    window: *const OpusVal16,
    fs: i32,
) {
    unsafe {
        let inc = 48000 / fs;
        for c in 0..channels {
            for i in 0..overlap {
                let w: OpusVal16 =
                    mult16_16_q15(*window.offset((i * inc) as isize), *window.offset((i * inc) as isize)) as OpusVal16;
                *out.offset((i * channels + c) as isize) = shr32(
                    mac16_16(
                        mult16_16(w, *in2.offset((i * channels + c) as isize)),
                        Q15ONE - w,
                        *in1.offset((i * channels + c) as isize),
                    ),
                    15,
                ) as OpusVal16;
            }
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

unsafe fn opus_decode_frame(
    st: *mut OpusDecoder,
    data: *const u8,
    len: c_int,
    pcm: *mut OpusVal16,
    frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    unsafe {
        let silk_dec = silk_dec_ptr(st);
        let celt_dec = celt_dec_ptr(st);
        let mut i: c_int;
        let mut silk_ret: c_int;
        let mut celt_ret: c_int = 0;
        let mut dec: ec_ctx = std::mem::zeroed();
        let mut silk_frame_size: i32 = 0;

        let audiosize: c_int;
        let mode: c_int;
        let mut transition: c_int = 0;
        let start_band: c_int;
        let mut redundancy: c_int = 0;
        let mut redundancy_bytes: c_int = 0;
        let mut celt_to_silk: c_int = 0;
        let mut c: c_int;

        let f20 = (*st).fs / 50;
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
            data = std::ptr::null();
            // In that case, don't conceal more than what the ToC says
            frame_size = frame_size.min((*st).frame_size);
        }

        if !data.is_null() {
            audiosize = (*st).frame_size;
            mode = (*st).mode;
            ec_dec_init(&mut dec, data as *mut u8, len as u32);
        } else {
            audiosize = frame_size;
            if (*st).prev_mode == 0 {
                // If we haven't got any packet yet, all we can do is return zeros
                i = 0;
                while i < audiosize * (*st).channels {
                    *pcm.offset(i as isize) = 0 as OpusVal16;
                    i += 1;
                }
                return audiosize;
            } else {
                mode = (*st).prev_mode;
            }
        }

        // For CELT/hybrid PLC of more than 20 ms, do multiple calls
        if data.is_null() && frame_size > f20 && mode != MODE_SILK_ONLY {
            let mut nb_samples: c_int = 0;
            loop {
                let ret = opus_decode_frame(
                    st,
                    std::ptr::null(),
                    0,
                    pcm.offset((nb_samples * (*st).channels) as isize) as *mut OpusVal16,
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

        let mut pcm_transition_buf: Vec<OpusVal16> = vec![0 as OpusVal16; (f5 * (*st).channels) as usize];
        let pcm_transition = pcm_transition_buf.as_mut_ptr();

        if !data.is_null()
            && (*st).prev_mode > 0
            && ((mode == MODE_CELT_ONLY && (*st).prev_mode != MODE_CELT_ONLY && (*st).prev_redundancy == 0)
                || (mode != MODE_CELT_ONLY && (*st).prev_mode == MODE_CELT_ONLY))
        {
            transition = 1;
            if mode == MODE_CELT_ONLY {
                opus_decode_frame(st, std::ptr::null(), 0, pcm_transition, f5.min(audiosize), 0);
            }
        }

        if audiosize > frame_size {
            return OPUS_BAD_ARG;
        } else {
            frame_size = audiosize;
        }

        let mut pcm_silk_buf: Vec<i16> = vec![0i16; (f10.max(frame_size) * (*st).channels) as usize];
        let pcm_silk = pcm_silk_buf.as_mut_ptr();
        let mut redundant_audio_buf: Vec<OpusVal16> = vec![0 as OpusVal16; (f5 * (*st).channels) as usize];
        let redundant_audio = redundant_audio_buf.as_mut_ptr();

        let mut redundant_rng: u32 = 0;

        // SILK processing
        if mode != MODE_CELT_ONLY {
            let lost_flag: c_int;
            let mut decoded_samples: c_int;
            let mut pcm_ptr = pcm_silk;

            if (*st).prev_mode == MODE_CELT_ONLY {
                silk_init_decoder(silk_dec);
            }

            // The SILK PLC cannot produce frames of less than 10 ms
            (*st).dec_control.payload_size_ms = (1000 * audiosize / (*st).fs).max(10);

            if !data.is_null() {
                (*st).dec_control.n_channels_internal = (*st).stream_channels;
                if mode == MODE_SILK_ONLY {
                    if (*st).bandwidth == OPUS_BANDWIDTH_NARROWBAND {
                        (*st).dec_control.internal_sample_rate = 8000;
                    } else if (*st).bandwidth == OPUS_BANDWIDTH_MEDIUMBAND {
                        (*st).dec_control.internal_sample_rate = 12000;
                    } else if (*st).bandwidth == OPUS_BANDWIDTH_WIDEBAND {
                        (*st).dec_control.internal_sample_rate = 16000;
                    } else {
                        (*st).dec_control.internal_sample_rate = 16000;
                    }
                } else {
                    // Hybrid mode
                    (*st).dec_control.internal_sample_rate = 16000;
                }
            }

            lost_flag = if data.is_null() { 1 } else { 2 * decode_fec };
            decoded_samples = 0;
            loop {
                // Call SILK decoder
                let first_frame = if decoded_samples == 0 { 1 } else { 0 };
                silk_ret = silk_decode(
                    silk_dec,
                    &mut (*st).dec_control,
                    lost_flag,
                    first_frame,
                    &mut dec,
                    pcm_ptr,
                    &mut silk_frame_size,
                );
                if silk_ret != 0 {
                    if lost_flag != 0 {
                        // PLC failure should not be fatal
                        silk_frame_size = frame_size;
                        i = 0;
                        while i < frame_size * (*st).channels {
                            *pcm_ptr.offset(i as isize) = 0;
                            i += 1;
                        }
                    } else {
                        return OPUS_INVALID_PACKET;
                    }
                }
                pcm_ptr = pcm_ptr.offset((silk_frame_size * (*st).channels) as isize);
                decoded_samples += silk_frame_size;
                if decoded_samples >= frame_size {
                    break;
                }
            }
        }

        start_band = 0;
        let mut start_band = start_band;
        if decode_fec == 0
            && mode != MODE_CELT_ONLY
            && !data.is_null()
            && ec_tell(&mut dec) + 17 + 20 * (if (*st).mode == MODE_HYBRID { 1 } else { 0 }) <= 8 * len
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
                redundancy_bytes = if mode == MODE_HYBRID {
                    ec_dec_uint(&mut dec, 256) as c_int + 2
                } else {
                    len - ((ec_tell(&mut dec) + 7) >> 3)
                };
                len -= redundancy_bytes;
                // Sanity check
                if len * 8 < ec_tell(&mut dec) {
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
            let endband: c_int = match (*st).bandwidth {
                OPUS_BANDWIDTH_NARROWBAND => 13,
                OPUS_BANDWIDTH_MEDIUMBAND | OPUS_BANDWIDTH_WIDEBAND => 17,
                OPUS_BANDWIDTH_SUPERWIDEBAND => 19,
                OPUS_BANDWIDTH_FULLBAND => 21,
                _ => 21,
            };
            celt_decoder_ctl(celt_dec, CeltDecCtl::SetEndBand(endband));
            celt_decoder_ctl(celt_dec, CeltDecCtl::SetChannels((*st).stream_channels));
        }

        if redundancy != 0 {
            transition = 0;
        }

        if transition != 0 && mode != MODE_CELT_ONLY {
            opus_decode_frame(st, std::ptr::null(), 0, pcm_transition, f5.min(audiosize), 0);
        }

        // 5 ms redundant frame for CELT->SILK
        if redundancy != 0 && celt_to_silk != 0 {
            celt_decoder_ctl(celt_dec, CeltDecCtl::SetStartBand(0));
            celt_decode_with_ec(
                celt_dec,
                data.offset(len as isize),
                redundancy_bytes,
                redundant_audio,
                f5,
                std::ptr::null_mut(),
            );
            celt_decoder_ctl(celt_dec, CeltDecCtl::GetFinalRange(&mut redundant_rng));
        }

        // MUST be after PLC
        celt_decoder_ctl(celt_dec, CeltDecCtl::SetStartBand(start_band));

        if mode != MODE_SILK_ONLY {
            let celt_frame_size = f20.min(frame_size);
            // Make sure to discard any previous CELT state
            if mode != (*st).prev_mode && (*st).prev_mode > 0 && (*st).prev_redundancy == 0 {
                celt_decoder_ctl(celt_dec, CeltDecCtl::ResetState);
            }
            // Decode CELT
            celt_ret = celt_decode_with_ec(
                celt_dec,
                if decode_fec != 0 { std::ptr::null() } else { data },
                len,
                pcm,
                celt_frame_size,
                &mut dec,
            );
        } else {
            let silence: [u8; 2] = [0xFF, 0xFF];
            i = 0;
            while i < frame_size * (*st).channels {
                *pcm.offset(i as isize) = 0 as OpusVal16;
                i += 1;
            }
            // For hybrid -> SILK transitions, we let the CELT MDCT
            // do a fade-out by decoding a silence frame
            if (*st).prev_mode == MODE_HYBRID && !(redundancy != 0 && celt_to_silk != 0 && (*st).prev_redundancy != 0) {
                celt_decoder_ctl(celt_dec, CeltDecCtl::SetStartBand(0));
                celt_decode_with_ec(celt_dec, silence.as_ptr(), 2, pcm, f2_5, std::ptr::null_mut());
            }
        }

        if mode != MODE_CELT_ONLY {
            #[cfg(feature = "fixed-point")]
            {
                i = 0;
                while i < frame_size * (*st).channels {
                    *pcm.offset(i as isize) = sat16(*pcm.offset(i as isize) as i32 + *pcm_silk.offset(i as isize) as i32);
                    i += 1;
                }
            }
            #[cfg(not(feature = "fixed-point"))]
            {
                i = 0;
                while i < frame_size * (*st).channels {
                    *pcm.offset(i as isize) = *pcm.offset(i as isize) + (1.0 / 32768.0) * *pcm_silk.offset(i as isize) as f32;
                    i += 1;
                }
            }
        }

        let window: *const OpusVal16;
        {
            let mut celt_mode: *const CELTMode = std::ptr::null();
            celt_decoder_ctl(celt_dec, CeltDecCtl::GetMode(&mut celt_mode));
            window = (*celt_mode).window.as_ptr();
        }

        // 5 ms redundant frame for SILK->CELT
        if redundancy != 0 && celt_to_silk == 0 {
            celt_decoder_ctl(celt_dec, CeltDecCtl::ResetState);
            celt_decoder_ctl(celt_dec, CeltDecCtl::SetStartBand(0));

            celt_decode_with_ec(
                celt_dec,
                data.offset(len as isize),
                redundancy_bytes,
                redundant_audio,
                f5,
                std::ptr::null_mut(),
            );
            celt_decoder_ctl(celt_dec, CeltDecCtl::GetFinalRange(&mut redundant_rng));
            smooth_fade(
                pcm.offset(((*st).channels * (frame_size - f2_5)) as isize),
                redundant_audio.offset(((*st).channels * f2_5) as isize),
                pcm.offset(((*st).channels * (frame_size - f2_5)) as isize),
                f2_5,
                (*st).channels,
                window,
                (*st).fs,
            );
        }
        if redundancy != 0 && celt_to_silk != 0 {
            c = 0;
            while c < (*st).channels {
                i = 0;
                while i < f2_5 {
                    *pcm.offset(((*st).channels * i + c) as isize) = *redundant_audio.offset(((*st).channels * i + c) as isize);
                    i += 1;
                }
                c += 1;
            }
            smooth_fade(
                redundant_audio.offset(((*st).channels * f2_5) as isize),
                pcm.offset(((*st).channels * f2_5) as isize),
                pcm.offset(((*st).channels * f2_5) as isize),
                f2_5,
                (*st).channels,
                window,
                (*st).fs,
            );
        }
        if transition != 0 {
            if audiosize >= f5 {
                i = 0;
                while i < (*st).channels * f2_5 {
                    *pcm.offset(i as isize) = *pcm_transition.offset(i as isize);
                    i += 1;
                }
                smooth_fade(
                    pcm_transition.offset(((*st).channels * f2_5) as isize),
                    pcm.offset(((*st).channels * f2_5) as isize),
                    pcm.offset(((*st).channels * f2_5) as isize),
                    f2_5,
                    (*st).channels,
                    window,
                    (*st).fs,
                );
            } else {
                smooth_fade(pcm_transition, pcm, pcm, f2_5, (*st).channels, window, (*st).fs);
            }
        }

        if len <= 1 {
            (*st).range_final = 0;
        } else {
            (*st).range_final = dec.rng ^ redundant_rng;
        }

        (*st).prev_mode = mode;
        (*st).prev_redundancy = if redundancy != 0 && celt_to_silk == 0 { 1 } else { 0 };
        if celt_ret < 0 { celt_ret } else { audiosize }
    }
}

// -- opus_decode_native --

pub unsafe fn opus_decode_native(
    st: *mut OpusDecoder,
    data: *const u8,
    len: c_int,
    mut pcm: *mut OpusVal16,
    frame_size: c_int,
    decode_fec: c_int,
    self_delimited: c_int,
    packet_offset: *mut c_int,
) -> c_int {
    unsafe {
        let mut i: c_int;
        let mut nb_samples: c_int;
        let count: c_int;
        let mut offset: c_int = 0;
        let mut toc: u8 = 0;
        let mut tot_offset: c_int;
        let mut size: [i16; 48] = [0i16; 48];

        if decode_fec < 0 || decode_fec > 1 {
            return OPUS_BAD_ARG;
        }
        if len == 0 || data.is_null() {
            return opus_decode_frame(st, std::ptr::null(), 0, pcm, frame_size, 0);
        } else if len < 0 {
            return OPUS_BAD_ARG;
        }

        tot_offset = 0;
        (*st).mode = opus_packet_get_mode(data);
        (*st).bandwidth = opus_packet_get_bandwidth(data);
        (*st).frame_size = opus_packet_get_samples_per_frame(data, (*st).fs);
        (*st).stream_channels = opus_packet_get_nb_channels(data);

        count =
            opus_packet_parse_impl(data, len, self_delimited, &mut toc, std::ptr::null_mut(), size.as_mut_ptr(), &mut offset);
        if count < 0 {
            return count;
        }

        let mut data = data.offset(offset as isize);
        tot_offset += offset;

        if count * (*st).frame_size > frame_size {
            return OPUS_BUFFER_TOO_SMALL;
        }
        nb_samples = 0;
        i = 0;
        while i < count {
            let ret = opus_decode_frame(st, data, size[i as usize] as c_int, pcm, frame_size - nb_samples, decode_fec);
            if ret < 0 {
                return ret;
            }
            data = data.offset(size[i as usize] as isize);
            tot_offset += size[i as usize] as c_int;
            pcm = pcm.offset((ret * (*st).channels) as isize);
            nb_samples += ret;
            i += 1;
        }
        if !packet_offset.is_null() {
            *packet_offset = tot_offset;
        }
        nb_samples
    }
}

// -- opus_decode --

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
    unsafe { opus_decode_native(st, data, len, pcm, frame_size, decode_fec, 0, std::ptr::null_mut()) }
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

        let mut out_buf: Vec<f32> = vec![0.0f32; (frame_size * (*st).channels) as usize];
        let ret = opus_decode_native(st, data, len, out_buf.as_mut_ptr(), frame_size, decode_fec, 0, std::ptr::null_mut());
        if ret > 0 {
            let mut i = 0;
            while i < ret * (*st).channels {
                *pcm.offset(i as isize) = float2int16(out_buf[i as usize]);
                i += 1;
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
    unsafe { opus_decode_native(st, data, len, pcm, frame_size, decode_fec, 0, std::ptr::null_mut()) }
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
        let mut out_buf: Vec<i16> = vec![0i16; (frame_size * (*st).channels) as usize];
        let ret = opus_decode_native(st, data, len, out_buf.as_mut_ptr(), frame_size, decode_fec, 0, std::ptr::null_mut());
        if ret > 0 {
            let mut i = 0;
            while i < ret * (*st).channels {
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
        let silk_dec = silk_dec_ptr(st);
        let celt_dec = celt_dec_ptr(st);

        match request {
            OpusDecCtl::GetBandwidth(value) => {
                *value = (*st).bandwidth;
            }
            OpusDecCtl::GetFinalRange(value) => {
                *value = (*st).range_final;
            }
            OpusDecCtl::ResetState => {
                let OpusDecoder {
                    celt_dec_offset: _,
                    silk_dec_offset: _,
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
                } = &mut *st;

                *bandwidth = 0;
                *mode = 0;
                *prev_mode = 0;
                *prev_redundancy = 0;
                *range_final = 0;
                *stream_channels = *channels;
                *frame_size = *fs / 400;

                celt_decoder_ctl(celt_dec, CeltDecCtl::ResetState);
                silk_init_decoder(silk_dec);
            }
            OpusDecCtl::GetPitch(value) => {
                if value.is_null() {
                    return OPUS_BAD_ARG;
                }
                if (*st).prev_mode == MODE_CELT_ONLY {
                    celt_decoder_ctl(celt_dec, CeltDecCtl::GetPitch(value));
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
