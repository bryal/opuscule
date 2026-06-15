// Translated from c/src/opus_decoder.c (RFC 6716 Section 3.1).
//
// Opus TOC byte query functions: mode, bandwidth, frame size,
// channel count, frame count. Pure bitfield reads with no
// float/fixed-point dependency.

use core::ffi::c_int;

use crate::util::{OrPanic, zip};

// -- Mode constants from opus_private.h --
const MODE_SILK_ONLY: c_int = 1000;
const MODE_HYBRID: c_int = 1001;
const MODE_CELT_ONLY: c_int = 1002;

// -- Bandwidth constants from opus_defines.h --
const OPUS_BANDWIDTH_NARROWBAND: c_int = 1101;
const OPUS_BANDWIDTH_MEDIUMBAND: c_int = 1102;
#[allow(dead_code)]
const OPUS_BANDWIDTH_WIDEBAND: c_int = 1103;
const OPUS_BANDWIDTH_SUPERWIDEBAND: c_int = 1104;
const OPUS_BANDWIDTH_FULLBAND: c_int = 1105;

// -- Error codes from opus_defines.h --
const OPUS_BAD_ARG: c_int = -1;
const OPUS_INVALID_PACKET: c_int = -4;

/// Extract the mode (SILK-only, Hybrid, or CELT-only) from the TOC byte.
/// RFC 6716 Section 3.1, Table 2. (Safe core of [`opus_packet_get_mode`].)
pub fn packet_get_mode(toc: u8) -> c_int {
    if toc & 0x80 != 0 {
        MODE_CELT_ONLY
    } else if toc & 0x60 == 0x60 {
        MODE_HYBRID
    } else {
        MODE_SILK_ONLY
    }
}

/// Extract the mode (SILK-only, Hybrid, or CELT-only) from the TOC byte.
/// RFC 6716 Section 3.1, Table 2.
///
/// # Safety
/// `data` must point to at least one readable byte (the TOC byte).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_mode(data: *const u8) -> c_int {
    // SAFETY: `data` points to at least the TOC byte, per the contract.
    packet_get_mode(unsafe { *data })
}

/// Return the bandwidth of an Opus packet from its TOC byte.
/// RFC 6716 Section 3.1, Table 2. (Safe core of [`opus_packet_get_bandwidth`].)
pub fn packet_get_bandwidth(toc: u8) -> c_int {
    if toc & 0x80 != 0 {
        let bandwidth = OPUS_BANDWIDTH_MEDIUMBAND + ((toc >> 5) & 0x3) as c_int;
        if bandwidth == OPUS_BANDWIDTH_MEDIUMBAND { OPUS_BANDWIDTH_NARROWBAND } else { bandwidth }
    } else if toc & 0x60 == 0x60 {
        if toc & 0x10 != 0 { OPUS_BANDWIDTH_FULLBAND } else { OPUS_BANDWIDTH_SUPERWIDEBAND }
    } else {
        OPUS_BANDWIDTH_NARROWBAND + ((toc >> 5) & 0x3) as c_int
    }
}

/// Return the bandwidth of an Opus packet from its TOC byte.
/// RFC 6716 Section 3.1, Table 2.
///
/// # Safety
/// `data` must point to at least one readable byte (the TOC byte).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_bandwidth(data: *const u8) -> c_int {
    // SAFETY: `data` points to at least the TOC byte, per the contract.
    packet_get_bandwidth(unsafe { *data })
}

/// Return the number of samples per frame from the TOC byte and sample rate.
/// RFC 6716 Section 3.1. (Safe core of [`opus_packet_get_samples_per_frame`].)
pub fn packet_get_samples_per_frame(toc: u8, fs: i32) -> c_int {
    if toc & 0x80 != 0 {
        let audiosize = ((toc >> 3) & 0x3) as i32;
        (fs << audiosize) / 400
    } else if toc & 0x60 == 0x60 {
        if toc & 0x08 != 0 { fs / 50 } else { fs / 100 }
    } else {
        let audiosize = ((toc >> 3) & 0x3) as i32;
        if audiosize == 3 { fs * 60 / 1000 } else { (fs << audiosize) / 100 }
    }
}

/// Return the number of samples per frame from the TOC byte and sample rate.
/// RFC 6716 Section 3.1.
///
/// # Safety
/// `data` must point to at least one readable byte (the TOC byte).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_samples_per_frame(data: *const u8, fs: i32) -> c_int {
    // SAFETY: `data` points to at least the TOC byte, per the contract.
    packet_get_samples_per_frame(unsafe { *data }, fs)
}

/// Return the number of channels encoded in a packet (1 or 2).
/// RFC 6716 Section 3.1 (stereo bit = bit 2 of TOC).
/// (Safe core of [`opus_packet_get_nb_channels`].)
pub fn packet_get_nb_channels(toc: u8) -> c_int {
    if toc & 0x4 != 0 { 2 } else { 1 }
}

/// Return the number of channels encoded in a packet (1 or 2).
/// RFC 6716 Section 3.1 (stereo bit = bit 2 of TOC).
///
/// # Safety
/// `data` must point to at least one readable byte (the TOC byte).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_nb_channels(data: *const u8) -> c_int {
    // SAFETY: `data` points to at least the TOC byte, per the contract.
    packet_get_nb_channels(unsafe { *data })
}

/// Return the number of frames in an Opus packet.
/// RFC 6716 Section 3.2.
///
/// # Safety
/// `packet` must be readable for `len` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_nb_frames(packet: *const u8, len: c_int) -> c_int {
    if len < 1 {
        return OPUS_BAD_ARG;
    }
    // SAFETY: `len >= 1` (checked), so the TOC byte is readable per the contract.
    let toc = unsafe { *packet };
    let count = toc & 0x3;
    if count == 0 {
        1
    } else if count != 3 {
        2
    } else if len < 2 {
        OPUS_INVALID_PACKET
    } else {
        // SAFETY: `len >= 2` (checked), so the second byte is readable.
        (unsafe { *packet.add(1) } & 0x3F) as c_int
    }
}

/// Parse a frame size from a VBR packet header.
/// Returns bytes consumed (1 or 2), or -1 on error.
/// Writes frame size to `*size` (-1 on error).
fn parse_size(data: &[u8], len: c_int, size: &mut i16) -> c_int {
    if len < 1 {
        *size = -1;
        return -1;
    }
    let b0 = i16::from(*data.first().or_panic("empty data in parse_size"));
    if b0 < 252 {
        *size = b0;
        1
    } else if len < 2 {
        *size = -1;
        -1
    } else {
        *size = 4 * i16::from(*data.get(1).or_panic("parse_size: missing second byte")) + b0;
        2
    }
}

/// Parse an Opus packet into TOC byte, frame offsets, sizes, and payload offset.
/// RFC 6716 Section 3.2.
///
/// This handles all four code types (0-3) including CBR/VBR multi-frame packets,
/// self-delimited framing, and padding. Called by both `opus_packet_parse`
/// (self_delimited=0) and `opus_decode_native` (self_delimited from packet).
///
/// Unlike the C, `frames` (when given) receives byte *offsets* into `data`
/// rather than pointers; the extern "C" wrapper [`opus_packet_parse`]
/// converts back to pointers for ABI compatibility. As in the C, the data
/// cursor only advances through the frame payloads when `frames` is
/// requested, which `payload_offset` reflects.
pub fn opus_packet_parse_impl(
    data: &[u8],
    self_delimited: c_int,
    out_toc: Option<&mut u8>,
    frames: Option<&mut [c_int]>,
    size: &mut [i16],
    payload_offset: Option<&mut c_int>,
) -> c_int {
    // `len` is the remaining-bytes counter, initialised from the packet length.
    let mut len = data.len() as c_int;

    let toc = *data.first().or_panic("empty packet");
    let framesize = packet_get_samples_per_frame(toc, 48000);

    let mut cbr: c_int = 0;
    let mut off = 1usize;
    len -= 1;
    let mut last_size = len;
    let count: c_int;

    match toc & 0x3 {
        // One frame
        0 => {
            count = 1;
        }
        // Two CBR frames
        1 => {
            count = 2;
            cbr = 1;
            if self_delimited == 0 {
                if len & 0x1 != 0 {
                    return OPUS_INVALID_PACKET;
                }
                last_size = len / 2;
                *size.first_mut().or_panic("empty size buffer") = last_size as i16;
            }
        }
        // Two VBR frames
        2 => {
            count = 2;
            let bytes = parse_size(
                data.get(off..).or_panic_dbg((off, data.len())),
                len,
                size.first_mut().or_panic("empty size buffer"),
            );
            len -= bytes;
            let s0 = *size.first().or_panic("empty size buffer");
            if s0 < 0 || i32::from(s0) > len {
                return OPUS_INVALID_PACKET;
            }
            off += bytes as usize;
            last_size = len - i32::from(s0);
        }
        // Multiple CBR/VBR frames (from 0 to 120 ms)
        _ => {
            if len < 1 {
                return OPUS_INVALID_PACKET;
            }
            // Number of frames encoded in bits 0 to 5
            let ch = *data.get(off).or_panic_dbg((off, data.len()));
            off += 1;
            count = (ch & 0x3F) as c_int;
            if count <= 0 || framesize * count > 5760 {
                return OPUS_INVALID_PACKET;
            }
            len -= 1;
            // Padding flag is bit 6
            if ch & 0x40 != 0 {
                let mut padding: c_int = 0;
                let mut p: c_int;
                loop {
                    if len <= 0 {
                        return OPUS_INVALID_PACKET;
                    }
                    p = i32::from(*data.get(off).or_panic_dbg((off, data.len())));
                    off += 1;
                    len -= 1;
                    padding += if p == 255 { 254 } else { p };
                    if p != 255 {
                        break;
                    }
                }
                len -= padding;
            }
            if len < 0 {
                return OPUS_INVALID_PACKET;
            }
            // VBR flag is bit 7
            cbr = (ch & 0x80 == 0) as c_int;
            if cbr == 0 {
                // VBR case
                last_size = len;
                for s in size.get_mut(..(count - 1) as usize).or_panic(count - 1) {
                    let bytes = parse_size(data.get(off..).or_panic_dbg((off, data.len())), len, s);
                    len -= bytes;
                    if *s < 0 || i32::from(*s) > len {
                        return OPUS_INVALID_PACKET;
                    }
                    off += bytes as usize;
                    last_size -= bytes + i32::from(*s);
                }
                if last_size < 0 {
                    return OPUS_INVALID_PACKET;
                }
            } else if self_delimited == 0 {
                // CBR case
                last_size = len / count;
                if last_size * count != len {
                    return OPUS_INVALID_PACKET;
                }
                size.get_mut(..(count - 1) as usize).or_panic(count - 1).fill(last_size as i16);
            }
        }
    }

    // Self-delimited framing has an extra size for the last frame.
    let last_idx = count as usize - 1;
    if self_delimited != 0 {
        let mut last = 0i16;
        let bytes = parse_size(data.get(off..).or_panic_dbg((off, data.len())), len, &mut last);
        *size.get_mut(last_idx).or_panic(last_idx) = last;
        len -= bytes;
        if last < 0 || i32::from(last) > len {
            return OPUS_INVALID_PACKET;
        }
        off += bytes as usize;
        // For CBR packets, apply the size to all the frames.
        if cbr != 0 {
            if i32::from(last) * count > len {
                return OPUS_INVALID_PACKET;
            }
            size.get_mut(..last_idx).or_panic(last_idx).fill(last);
        } else if i32::from(last) > last_size {
            return OPUS_INVALID_PACKET;
        }
    } else {
        // Because it's not encoded explicitly, it's possible the size of the
        // last packet (or all the packets, for the CBR case) is larger than
        // 1275. Reject them here.
        if last_size > 1275 {
            return OPUS_INVALID_PACKET;
        }
        *size.get_mut(last_idx).or_panic(last_idx) = last_size as i16;
    }

    if let Some(frames) = frames {
        for (f, &s) in zip(frames.iter_mut(), size.iter()).take(count as usize) {
            *f = off as c_int;
            off += s as usize;
        }
    }

    if let Some(out_toc) = out_toc {
        *out_toc = toc;
    }

    if let Some(payload_offset) = payload_offset {
        *payload_offset = off as c_int;
    }

    count
}

/// Parse an Opus packet (non-self-delimited), RFC 6716 Section 3.2.
///
/// Safe core of [`opus_packet_parse`]. `opus_packet_parse_impl` reports
/// frames as byte offsets into `data`; here we turn each offset back into a
/// pointer into `data` for the C-ABI caller, using `wrapping_add` (a safe
/// pointer op) rather than indexing.
fn opus_packet_parse_native(
    data: &[u8],
    out_toc: Option<&mut u8>,
    frames: Option<&mut [*const u8]>,
    size: &mut [i16],
    payload_offset: Option<&mut c_int>,
) -> c_int {
    let mut frame_offsets = [0 as c_int; 48];
    let count = opus_packet_parse_impl(
        data,
        0,
        out_toc,
        if frames.is_some() { Some(&mut frame_offsets[..]) } else { None },
        size,
        payload_offset,
    );
    if count > 0 {
        if let Some(frames) = frames {
            for (frame, &off) in frames.iter_mut().zip(&frame_offsets).take(count as usize) {
                *frame = data.as_ptr().wrapping_add(off as usize);
            }
        }
    }
    count
}

/// Parse an Opus packet (non-self-delimited), RFC 6716 Section 3.2.
///
/// Thin C-ABI shell: it only converts the raw pointer arguments into safe
/// references and slices, then defers to [`opus_packet_parse_native`].
///
/// # Safety
/// `data` must be readable for `len` bytes; `size` and (if non-null) `frames`
/// must be writable for 48 entries; `out_toc` and `payload_offset`, if
/// non-null, must be writable.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_parse(
    data: *const u8,
    len: c_int,
    out_toc: *mut u8,
    frames: *mut *const u8,
    size: *mut i16,
    payload_offset: *mut c_int,
) -> c_int {
    // SAFETY: the pointers satisfy the documented contract; `size`/`frames` hold
    // the 48-entry maximum the parser can write, and the out-params are
    // null-checked before being turned into references.
    unsafe {
        if size.is_null() {
            return OPUS_BAD_ARG;
        }
        // A negative length is meaningless; parse it into the slice length once.
        let Ok(len) = usize::try_from(len) else {
            return OPUS_BAD_ARG;
        };
        opus_packet_parse_native(
            core::slice::from_raw_parts(data, len),
            if out_toc.is_null() { None } else { Some(&mut *out_toc) },
            if frames.is_null() { None } else { Some(core::slice::from_raw_parts_mut(frames, 48)) },
            core::slice::from_raw_parts_mut(size, 48),
            if payload_offset.is_null() { None } else { Some(&mut *payload_offset) },
        )
    }
}

#[cfg(test)]
mod tests {
    // These tests call the `unsafe extern "C"` query functions with pointers to
    // local stack values / arrays whose validity is obvious at the call site; a
    // `// SAFETY:` comment on each would be pure noise.
    #![allow(clippy::undocumented_unsafe_blocks)]

    use super::*;

    #[test]
    fn test_get_mode() {
        // SILK-only: bit 7 = 0, bits 6..5 != 11
        assert_eq!(unsafe { opus_packet_get_mode(&0x00u8) }, MODE_SILK_ONLY);
        assert_eq!(unsafe { opus_packet_get_mode(&0x40u8) }, MODE_SILK_ONLY);
        // Hybrid: bits 7..5 = 011
        assert_eq!(unsafe { opus_packet_get_mode(&0x60u8) }, MODE_HYBRID);
        assert_eq!(unsafe { opus_packet_get_mode(&0x70u8) }, MODE_HYBRID);
        // CELT-only: bit 7 = 1
        assert_eq!(unsafe { opus_packet_get_mode(&0x80u8) }, MODE_CELT_ONLY);
        assert_eq!(unsafe { opus_packet_get_mode(&0xFFu8) }, MODE_CELT_ONLY);
    }

    #[test]
    fn test_get_nb_channels() {
        assert_eq!(unsafe { opus_packet_get_nb_channels(&0x00u8) }, 1);
        assert_eq!(unsafe { opus_packet_get_nb_channels(&0x04u8) }, 2);
        assert_eq!(unsafe { opus_packet_get_nb_channels(&0xFBu8) }, 1);
        assert_eq!(unsafe { opus_packet_get_nb_channels(&0xFFu8) }, 2);
    }

    #[test]
    fn test_get_nb_frames() {
        // Code 0: 1 frame
        assert_eq!(unsafe { opus_packet_get_nb_frames([0x00].as_ptr(), 1) }, 1);
        // Code 1: 2 frames
        assert_eq!(unsafe { opus_packet_get_nb_frames([0x01].as_ptr(), 1) }, 2);
        // Code 2: 2 frames
        assert_eq!(unsafe { opus_packet_get_nb_frames([0x02].as_ptr(), 1) }, 2);
        // Code 3: count from second byte
        assert_eq!(unsafe { opus_packet_get_nb_frames([0x03, 0x05].as_ptr(), 2) }, 5);
        // Code 3, len < 2
        assert_eq!(unsafe { opus_packet_get_nb_frames([0x03].as_ptr(), 1) }, OPUS_INVALID_PACKET);
        // Empty
        assert_eq!(unsafe { opus_packet_get_nb_frames([0x00].as_ptr(), 0) }, OPUS_BAD_ARG);
    }

    #[test]
    fn test_get_bandwidth() {
        // SILK NB
        assert_eq!(unsafe { opus_packet_get_bandwidth(&0x00u8) }, OPUS_BANDWIDTH_NARROWBAND);
        // SILK MB
        assert_eq!(unsafe { opus_packet_get_bandwidth(&0x20u8) }, OPUS_BANDWIDTH_MEDIUMBAND);
        // SILK WB
        assert_eq!(unsafe { opus_packet_get_bandwidth(&0x40u8) }, OPUS_BANDWIDTH_WIDEBAND);
        // Hybrid SWB
        assert_eq!(unsafe { opus_packet_get_bandwidth(&0x60u8) }, OPUS_BANDWIDTH_SUPERWIDEBAND);
        // Hybrid FB
        assert_eq!(unsafe { opus_packet_get_bandwidth(&0x70u8) }, OPUS_BANDWIDTH_FULLBAND);
        // CELT NB (bits 6..5 = 00 maps to MEDIUMBAND then corrected to NB)
        assert_eq!(unsafe { opus_packet_get_bandwidth(&0x80u8) }, OPUS_BANDWIDTH_NARROWBAND);
        // CELT WB
        assert_eq!(unsafe { opus_packet_get_bandwidth(&0xA0u8) }, OPUS_BANDWIDTH_WIDEBAND);
    }

    #[test]
    fn test_get_samples_per_frame() {
        // SILK 10ms @ 48kHz
        assert_eq!(unsafe { opus_packet_get_samples_per_frame(&0x00u8, 48000) }, 480);
        // SILK 20ms
        assert_eq!(unsafe { opus_packet_get_samples_per_frame(&0x08u8, 48000) }, 960);
        // SILK 40ms
        assert_eq!(unsafe { opus_packet_get_samples_per_frame(&0x10u8, 48000) }, 1920);
        // SILK 60ms
        assert_eq!(unsafe { opus_packet_get_samples_per_frame(&0x18u8, 48000) }, 2880);
        // CELT 2.5ms @ 48kHz
        assert_eq!(unsafe { opus_packet_get_samples_per_frame(&0x80u8, 48000) }, 120);
        // CELT 20ms
        assert_eq!(unsafe { opus_packet_get_samples_per_frame(&0x98u8, 48000) }, 960);
        // Hybrid 10ms
        assert_eq!(unsafe { opus_packet_get_samples_per_frame(&0x60u8, 48000) }, 480);
        // Hybrid 20ms
        assert_eq!(unsafe { opus_packet_get_samples_per_frame(&0x68u8, 48000) }, 960);
    }

    #[test]
    fn test_parse_size() {
        let mut size: i16 = 0;

        // Single-byte size (< 252)
        assert_eq!(parse_size(&[100u8], 1, &mut size), 1);
        assert_eq!(size, 100);

        // Two-byte size (>= 252): 4 * data[1] + data[0]
        assert_eq!(parse_size(&[252u8, 10], 2, &mut size), 2);
        assert_eq!(size, 4 * 10 + 252);

        // Empty input
        assert_eq!(parse_size(&[0u8], 0, &mut size), -1);
        assert_eq!(size, -1);

        // >= 252 but only 1 byte available
        size = 0;
        assert_eq!(parse_size(&[252u8], 1, &mut size), -1);
        assert_eq!(size, -1);
    }

    #[test]
    fn test_packet_parse_code0_one_frame() {
        // Code 0 = one frame, CELT 2.5ms NB (TOC = 0x80)
        let pkt = [0x80u8, 0xAA, 0xBB, 0xCC];
        let mut toc: u8 = 0;
        let mut frames: [*const u8; 48] = [core::ptr::null(); 48];
        let mut sizes: [i16; 48] = [0; 48];
        let mut offset: c_int = 0;

        let count = unsafe {
            opus_packet_parse(pkt.as_ptr(), pkt.len() as c_int, &mut toc, frames.as_mut_ptr(), sizes.as_mut_ptr(), &mut offset)
        };
        assert_eq!(count, 1);
        assert_eq!(toc, 0x80);
        assert_eq!(sizes[0], 3); // 4 bytes total - 1 byte TOC = 3
        assert_eq!(offset, 4); // all bytes consumed
    }

    #[test]
    fn test_packet_parse_code1_two_cbr() {
        // Code 1 = two CBR frames, CELT 2.5ms NB
        // TOC = 0x81 (CELT, code 1), 6 bytes payload → 3 bytes each
        let pkt = [0x81u8, 1, 2, 3, 4, 5, 6];
        let mut toc: u8 = 0;
        let mut frames: [*const u8; 48] = [core::ptr::null(); 48];
        let mut sizes: [i16; 48] = [0; 48];
        let mut offset: c_int = 0;

        let count = unsafe {
            opus_packet_parse(pkt.as_ptr(), pkt.len() as c_int, &mut toc, frames.as_mut_ptr(), sizes.as_mut_ptr(), &mut offset)
        };
        assert_eq!(count, 2);
        assert_eq!(toc, 0x81);
        assert_eq!(sizes[0], 3);
        assert_eq!(sizes[1], 3);
        assert_eq!(offset, 7);
    }

    #[test]
    fn test_packet_parse_code1_odd_length_rejected() {
        // Code 1 with odd payload length (3 bytes) → OPUS_INVALID_PACKET
        let pkt = [0x81u8, 1, 2, 3];
        let mut sizes: [i16; 48] = [0; 48];
        let count = unsafe {
            opus_packet_parse(
                pkt.as_ptr(),
                pkt.len() as c_int,
                core::ptr::null_mut(),
                core::ptr::null_mut(),
                sizes.as_mut_ptr(),
                core::ptr::null_mut(),
            )
        };
        assert_eq!(count, OPUS_INVALID_PACKET);
    }

    #[test]
    fn test_packet_parse_code2_two_vbr() {
        // Code 2 = two VBR frames
        // TOC = 0x82, size[0] = 2 (single-byte), remaining = last frame
        let pkt = [0x82u8, 2, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE];
        let mut toc: u8 = 0;
        let mut frames: [*const u8; 48] = [core::ptr::null(); 48];
        let mut sizes: [i16; 48] = [0; 48];
        let mut offset: c_int = 0;

        let count = unsafe {
            opus_packet_parse(pkt.as_ptr(), pkt.len() as c_int, &mut toc, frames.as_mut_ptr(), sizes.as_mut_ptr(), &mut offset)
        };
        assert_eq!(count, 2);
        assert_eq!(sizes[0], 2);
        // 7 total - 1 TOC - 1 size byte = 5 payload; 5 - 2 = 3 for last
        assert_eq!(sizes[1], 3);
        assert_eq!(offset, 7);
    }

    #[test]
    fn test_packet_parse_null_size_returns_bad_arg() {
        let pkt = [0x80u8, 0xAA];
        let count = unsafe {
            opus_packet_parse(
                pkt.as_ptr(),
                pkt.len() as c_int,
                core::ptr::null_mut(),
                core::ptr::null_mut(),
                core::ptr::null_mut(),
                core::ptr::null_mut(),
            )
        };
        assert_eq!(count, OPUS_BAD_ARG);
    }

    #[test]
    fn test_packet_parse_frames_null_no_advance() {
        // When frames is NULL, data should NOT be advanced past frame data,
        // so payload_offset only covers header bytes.
        let pkt = [0x80u8, 0xAA, 0xBB];
        let mut sizes: [i16; 48] = [0; 48];
        let mut offset: c_int = 0;

        let count = unsafe {
            opus_packet_parse(
                pkt.as_ptr(),
                pkt.len() as c_int,
                core::ptr::null_mut(),
                core::ptr::null_mut(),
                sizes.as_mut_ptr(),
                &mut offset,
            )
        };
        assert_eq!(count, 1);
        assert_eq!(sizes[0], 2);
        // frames=NULL → data not advanced past frames → offset = 1 (TOC only)
        assert_eq!(offset, 1);
    }
}
