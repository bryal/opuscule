// Translated from c/src/opus_multistream.c (RFC 6716).
//
// Multistream Opus decoder: handles audio with more than 2 channels (e.g.
// 5.1 surround) by managing an array of regular OpusDecoder instances and
// routing PCM samples per a channel mapping table. Sub-decoders are
// embedded at byte offsets after the OpusMSDecoder header in a single
// allocation, matching the C layout.
//
// Decoder-only: the encoder half of opus_multistream.c is not translated.
// Nothing in our test harness exercises multistream — this exists so that
// the API surface is available, not because it has been validated.

use std::os::raw::c_int;

use crate::arch::*;
use crate::opus_decoder::{
    OpusDecCtl, OpusDecoder, align, opus_decode_native, opus_decoder_ctl, opus_decoder_get_size, opus_decoder_init,
};

#[cfg(not(feature = "fixed-point"))]
use crate::opus_decoder::float2int16;
use crate::util::OrPanic;

// -- Constants --

const OPUS_OK: c_int = 0;
const OPUS_BAD_ARG: c_int = -1;
const OPUS_BUFFER_TOO_SMALL: c_int = -2;
const OPUS_INVALID_PACKET: c_int = -4;
const OPUS_ALLOC_FAIL: c_int = -7;

const OPUS_GET_BANDWIDTH_REQUEST: c_int = 4009;
const OPUS_GET_FINAL_RANGE_REQUEST: c_int = 4031;
const OPUS_RESET_STATE: c_int = 4028;
const OPUS_MULTISTREAM_GET_DECODER_STATE_REQUEST: c_int = 5122;

// -- ChannelLayout --

pub struct ChannelLayout {
    pub nb_channels: c_int,
    pub nb_streams: c_int,
    pub nb_coupled_streams: c_int,
    pub mapping: [u8; 256],
}

// -- OpusMSDecoder --

/// Multistream Opus decoder state.
///
/// Sub-decoders (one OpusDecoder per stream) live at byte offsets after
/// this header within the same allocation, just as in the C version.
pub struct OpusMSDecoder {
    pub layout: ChannelLayout,
    // Decoder states go here
}

// -- Layout helpers --

fn validate_layout(layout: &ChannelLayout) -> c_int {
    let max_channel = layout.nb_streams + layout.nb_coupled_streams;
    if max_channel > 255 {
        return 0;
    }
    let mut i = 0;
    while i < layout.nb_channels {
        if *layout.mapping.get(i as usize).or_panic(i) as c_int >= max_channel
            && *layout.mapping.get(i as usize).or_panic(i) != 255
        {
            return 0;
        }
        i += 1;
    }
    1
}

fn get_left_channel(layout: &ChannelLayout, stream_id: c_int, prev: c_int) -> c_int {
    let mut i = if prev < 0 { 0 } else { prev + 1 };
    while i < layout.nb_channels {
        if *layout.mapping.get(i as usize).or_panic(i) as c_int == stream_id * 2 {
            return i;
        }
        i += 1;
    }
    -1
}

fn get_right_channel(layout: &ChannelLayout, stream_id: c_int, prev: c_int) -> c_int {
    let mut i = if prev < 0 { 0 } else { prev + 1 };
    while i < layout.nb_channels {
        if *layout.mapping.get(i as usize).or_panic(i) as c_int == stream_id * 2 + 1 {
            return i;
        }
        i += 1;
    }
    -1
}

fn get_mono_channel(layout: &ChannelLayout, stream_id: c_int, prev: c_int) -> c_int {
    let mut i = if prev < 0 { 0 } else { prev + 1 };
    while i < layout.nb_channels {
        if *layout.mapping.get(i as usize).or_panic(i) as c_int == stream_id + layout.nb_coupled_streams {
            return i;
        }
        i += 1;
    }
    -1
}

// -- get_size / init / create / destroy --

#[unsafe(no_mangle)]
pub extern "C" fn opus_multistream_decoder_get_size(nb_streams: c_int, nb_coupled_streams: c_int) -> i32 {
    if nb_streams < 1 || nb_coupled_streams > nb_streams || nb_coupled_streams < 0 {
        return 0;
    }
    let coupled_size = opus_decoder_get_size(2);
    let mono_size = opus_decoder_get_size(1);
    (align(std::mem::size_of::<OpusMSDecoder>())
        + nb_coupled_streams as usize * align(coupled_size as usize)
        + (nb_streams - nb_coupled_streams) as usize * align(mono_size as usize)) as i32
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_multistream_decoder_init(
    st: *mut OpusMSDecoder,
    fs: i32,
    channels: c_int,
    streams: c_int,
    coupled_streams: c_int,
    mapping: *const u8,
) -> c_int {
    unsafe {
        (*st).layout.nb_channels = channels;
        (*st).layout.nb_streams = streams;
        (*st).layout.nb_coupled_streams = coupled_streams;

        let mut i = 0;
        while i < (*st).layout.nb_channels {
            *(*st).layout.mapping.get_mut(i as usize).or_panic(i) = *mapping.offset(i as isize);
            i += 1;
        }
        if validate_layout(&(*st).layout) == 0 {
            return OPUS_BAD_ARG;
        }

        let mut ptr = (st as *mut u8).add(align(std::mem::size_of::<OpusMSDecoder>()));
        let coupled_size = opus_decoder_get_size(2);
        let mono_size = opus_decoder_get_size(1);

        let mut i = 0;
        while i < (*st).layout.nb_coupled_streams {
            let ret = opus_decoder_init(ptr as *mut OpusDecoder, fs, 2);
            if ret != OPUS_OK {
                return ret;
            }
            ptr = ptr.add(align(coupled_size as usize));
            i += 1;
        }
        while i < (*st).layout.nb_streams {
            let ret = opus_decoder_init(ptr as *mut OpusDecoder, fs, 1);
            if ret != OPUS_OK {
                return ret;
            }
            ptr = ptr.add(align(mono_size as usize));
            i += 1;
        }
        OPUS_OK
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_multistream_decoder_create(
    fs: i32,
    channels: c_int,
    streams: c_int,
    coupled_streams: c_int,
    mapping: *const u8,
    error: *mut c_int,
) -> *mut OpusMSDecoder {
    unsafe {
        let size = opus_multistream_decoder_get_size(streams, coupled_streams);
        if size == 0 {
            if !error.is_null() {
                *error = OPUS_BAD_ARG;
            }
            return std::ptr::null_mut();
        }
        let layout = std::alloc::Layout::from_size_align(size as usize, std::mem::align_of::<OpusMSDecoder>())
            .expect("invalid layout for OpusMSDecoder");
        let ptr = std::alloc::alloc_zeroed(layout) as *mut OpusMSDecoder;
        if ptr.is_null() {
            if !error.is_null() {
                *error = OPUS_ALLOC_FAIL;
            }
            return std::ptr::null_mut();
        }
        let ret = opus_multistream_decoder_init(ptr, fs, channels, streams, coupled_streams, mapping);
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
pub unsafe extern "C" fn opus_multistream_decoder_destroy(st: *mut OpusMSDecoder) {
    unsafe {
        if st.is_null() {
            return;
        }
        let size = opus_multistream_decoder_get_size((*st).layout.nb_streams, (*st).layout.nb_coupled_streams);
        let layout = std::alloc::Layout::from_size_align(size as usize, std::mem::align_of::<OpusMSDecoder>())
            .expect("invalid layout for OpusMSDecoder");
        std::alloc::dealloc(st as *mut u8, layout);
    }
}

// -- decode_native --

unsafe fn opus_multistream_decode_native(
    st: *mut OpusMSDecoder,
    mut data: *const u8,
    mut len: c_int,
    pcm: *mut OpusVal16,
    mut frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    unsafe {
        let mut do_plc = 0;
        let mut buf: Vec<OpusVal16> = vec![Default::default(); (2 * frame_size) as usize];
        let mut ptr = (st as *mut u8).add(align(std::mem::size_of::<OpusMSDecoder>()));
        let coupled_size = opus_decoder_get_size(2);
        let mono_size = opus_decoder_get_size(1);

        if len == 0 {
            do_plc = 1;
        }
        if len < 0 {
            return OPUS_BAD_ARG;
        }
        if do_plc == 0 && len < 2 * (*st).layout.nb_streams - 1 {
            return OPUS_INVALID_PACKET;
        }
        let mut s = 0;
        while s < (*st).layout.nb_streams {
            let dec = ptr as *mut OpusDecoder;
            ptr = ptr.add(if s < (*st).layout.nb_coupled_streams {
                align(coupled_size as usize)
            } else {
                align(mono_size as usize)
            });

            if do_plc == 0 && len <= 0 {
                return OPUS_INVALID_PACKET;
            }
            let mut packet_offset: c_int = 0;
            let data_view = if data.is_null() { None } else { Some(core::slice::from_raw_parts(data, len.max(0) as usize)) };
            let ret = opus_decode_native(
                &mut *dec,
                data_view,
                len,
                &mut buf,
                frame_size,
                decode_fec,
                if s != (*st).layout.nb_streams - 1 { 1 } else { 0 },
                Some(&mut packet_offset),
            );
            data = data.offset(packet_offset as isize);
            len -= packet_offset;
            if ret > frame_size {
                return OPUS_BUFFER_TOO_SMALL;
            }
            if s > 0 && ret != frame_size {
                return OPUS_INVALID_PACKET;
            }
            if ret <= 0 {
                return ret;
            }
            frame_size = ret;
            if s < (*st).layout.nb_coupled_streams {
                let mut prev = -1;
                // Copy "left" audio to the channel(s) where it belongs
                loop {
                    let chan = get_left_channel(&(*st).layout, s, prev);
                    if chan == -1 {
                        break;
                    }
                    let mut i = 0;
                    while i < frame_size {
                        *pcm.offset(((*st).layout.nb_channels * i + chan) as isize) =
                            *buf.get((2 * i) as usize).or_panic(2 * i);
                        i += 1;
                    }
                    prev = chan;
                }
                let mut prev = -1;
                // Copy "right" audio to the channel(s) where it belongs
                loop {
                    let chan = get_right_channel(&(*st).layout, s, prev);
                    if chan == -1 {
                        break;
                    }
                    let mut i = 0;
                    while i < frame_size {
                        *pcm.offset(((*st).layout.nb_channels * i + chan) as isize) =
                            *buf.get((2 * i + 1) as usize).or_panic(2 * i + 1);
                        i += 1;
                    }
                    prev = chan;
                }
            } else {
                let mut prev = -1;
                // Copy audio to the channel(s) where it belongs
                loop {
                    let chan = get_mono_channel(&(*st).layout, s, prev);
                    if chan == -1 {
                        break;
                    }
                    let mut i = 0;
                    while i < frame_size {
                        *pcm.offset(((*st).layout.nb_channels * i + chan) as isize) = *buf.get(i as usize).or_panic(i);
                        i += 1;
                    }
                    prev = chan;
                }
            }
            s += 1;
        }
        // Handle muted channels
        let mut c = 0;
        while c < (*st).layout.nb_channels {
            if *(*st).layout.mapping.get(c as usize).or_panic(c) == 255 {
                let mut i = 0;
                while i < frame_size {
                    *pcm.offset(((*st).layout.nb_channels * i + c) as isize) = Default::default();
                    i += 1;
                }
            }
            c += 1;
        }
        frame_size
    }
}

// -- decode / decode_float --

#[cfg(feature = "fixed-point")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_multistream_decode(
    st: *mut OpusMSDecoder,
    data: *const u8,
    len: c_int,
    pcm: *mut i16,
    frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    unsafe { opus_multistream_decode_native(st, data, len, pcm, frame_size, decode_fec) }
}

#[cfg(not(feature = "fixed-point"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_multistream_decode(
    st: *mut OpusMSDecoder,
    data: *const u8,
    len: c_int,
    pcm: *mut i16,
    frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    unsafe {
        let nb_channels = (*st).layout.nb_channels;
        let mut out: Vec<f32> = vec![0.0f32; (frame_size * nb_channels) as usize];
        let ret = opus_multistream_decode_native(st, data, len, out.as_mut_ptr(), frame_size, decode_fec);
        if ret > 0 {
            let mut i = 0;
            while i < ret * nb_channels {
                *pcm.offset(i as isize) = float2int16(*out.get(i as usize).or_panic(i));
                i += 1;
            }
        }
        ret
    }
}

#[cfg(not(feature = "fixed-point"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_multistream_decode_float(
    st: *mut OpusMSDecoder,
    data: *const u8,
    len: c_int,
    pcm: *mut f32,
    frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    unsafe { opus_multistream_decode_native(st, data, len, pcm, frame_size, decode_fec) }
}

#[cfg(feature = "fixed-point")]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_multistream_decode_float(
    st: *mut OpusMSDecoder,
    data: *const u8,
    len: c_int,
    pcm: *mut f32,
    frame_size: c_int,
    decode_fec: c_int,
) -> c_int {
    unsafe {
        let nb_channels = (*st).layout.nb_channels;
        let mut out: Vec<i16> = vec![0i16; (frame_size * nb_channels) as usize];
        let ret = opus_multistream_decode_native(st, data, len, out.as_mut_ptr(), frame_size, decode_fec);
        if ret > 0 {
            let mut i = 0;
            while i < ret * nb_channels {
                *pcm.offset(i as isize) = (1.0f32 / 32768.0f32) * out[i as usize] as f32;
                i += 1;
            }
        }
        ret
    }
}

// -- decoder_ctl --

/// FFI-safe tagged enum for multistream decoder CTL requests.
#[repr(C, i32)]
pub enum OpusMSDecCtl {
    GetBandwidth(*mut c_int) = OPUS_GET_BANDWIDTH_REQUEST,
    GetFinalRange(*mut u32) = OPUS_GET_FINAL_RANGE_REQUEST,
    ResetState = OPUS_RESET_STATE,
    GetDecoderState(c_int, *mut *mut OpusDecoder) = OPUS_MULTISTREAM_GET_DECODER_STATE_REQUEST,
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_multistream_decoder_ctl(st: *mut OpusMSDecoder, request: OpusMSDecCtl) -> c_int {
    unsafe {
        let coupled_size = opus_decoder_get_size(2);
        let mono_size = opus_decoder_get_size(1);
        let mut ptr = (st as *mut u8).add(align(std::mem::size_of::<OpusMSDecoder>()));

        match request {
            OpusMSDecCtl::GetBandwidth(value) => {
                // For int32* GET params, just query the first stream
                let dec = ptr as *mut OpusDecoder;
                opus_decoder_ctl(dec, OpusDecCtl::GetBandwidth(value))
            }
            OpusMSDecCtl::GetFinalRange(value) => {
                let mut tmp: u32 = 0;
                *value = 0;
                let mut s = 0;
                let mut ret = OPUS_OK;
                while s < (*st).layout.nb_streams {
                    let dec = ptr as *mut OpusDecoder;
                    ptr = ptr.add(if s < (*st).layout.nb_coupled_streams {
                        align(coupled_size as usize)
                    } else {
                        align(mono_size as usize)
                    });
                    ret = opus_decoder_ctl(dec, OpusDecCtl::GetFinalRange(&mut tmp));
                    if ret != OPUS_OK {
                        break;
                    }
                    *value ^= tmp;
                    s += 1;
                }
                ret
            }
            OpusMSDecCtl::ResetState => {
                let mut s = 0;
                let mut ret = OPUS_OK;
                while s < (*st).layout.nb_streams {
                    let dec = ptr as *mut OpusDecoder;
                    ptr = ptr.add(if s < (*st).layout.nb_coupled_streams {
                        align(coupled_size as usize)
                    } else {
                        align(mono_size as usize)
                    });
                    ret = opus_decoder_ctl(dec, OpusDecCtl::ResetState);
                    if ret != OPUS_OK {
                        break;
                    }
                    s += 1;
                }
                ret
            }
            OpusMSDecCtl::GetDecoderState(stream_id, value) => {
                let mut ret = OPUS_OK;
                if stream_id < 0 || stream_id >= (*st).layout.nb_streams {
                    ret = OPUS_BAD_ARG;
                }
                let mut s = 0;
                while s < stream_id {
                    ptr = ptr.add(if s < (*st).layout.nb_coupled_streams {
                        align(coupled_size as usize)
                    } else {
                        align(mono_size as usize)
                    });
                    s += 1;
                }
                *value = ptr as *mut OpusDecoder;
                ret
            }
        }
    }
}
