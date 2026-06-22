// Translated from c/src/opus_decoder.c (RFC 6716 Section 3.1).
//
// Opus TOC byte query functions: mode, bandwidth, frame size,
// channel count, frame count. Pure bitfield reads with no
// float/fixed-point dependency.

use crate::error::OPUS_INVALID_PACKET;
use crate::util::{OrPanic, zip};

// Mode and bandwidth identifiers (opus_private.h / opus_defines.h). Derived
// from the TOC byte here and consumed by opus_decoder, so they live in one
// place and are shared `pub(crate)`.
pub(crate) const MODE_SILK_ONLY: i32 = 1000;
pub(crate) const MODE_HYBRID: i32 = 1001;
pub(crate) const MODE_CELT_ONLY: i32 = 1002;

pub(crate) const OPUS_BANDWIDTH_NARROWBAND: i32 = 1101;
pub(crate) const OPUS_BANDWIDTH_MEDIUMBAND: i32 = 1102;
pub(crate) const OPUS_BANDWIDTH_WIDEBAND: i32 = 1103;
pub(crate) const OPUS_BANDWIDTH_SUPERWIDEBAND: i32 = 1104;
pub(crate) const OPUS_BANDWIDTH_FULLBAND: i32 = 1105;

/// Extract the mode (SILK-only, Hybrid, or CELT-only) from the TOC byte.
/// RFC 6716 Section 3.1, Table 2. (Safe core of [`opus_packet_get_mode`].)
pub fn packet_get_mode(toc: u8) -> i32 {
    if toc & 0x80 != 0 {
        MODE_CELT_ONLY
    } else if toc & 0x60 == 0x60 {
        MODE_HYBRID
    } else {
        MODE_SILK_ONLY
    }
}

/// Return the bandwidth of an Opus packet from its TOC byte.
/// RFC 6716 Section 3.1, Table 2. (Safe core of [`opus_packet_get_bandwidth`].)
pub fn packet_get_bandwidth(toc: u8) -> i32 {
    if toc & 0x80 != 0 {
        let bandwidth = OPUS_BANDWIDTH_MEDIUMBAND + ((toc >> 5) & 0x3) as i32;
        if bandwidth == OPUS_BANDWIDTH_MEDIUMBAND { OPUS_BANDWIDTH_NARROWBAND } else { bandwidth }
    } else if toc & 0x60 == 0x60 {
        if toc & 0x10 != 0 { OPUS_BANDWIDTH_FULLBAND } else { OPUS_BANDWIDTH_SUPERWIDEBAND }
    } else {
        OPUS_BANDWIDTH_NARROWBAND + ((toc >> 5) & 0x3) as i32
    }
}

/// Return the number of samples per frame from the TOC byte and sample rate.
/// RFC 6716 Section 3.1. (Safe core of [`opus_packet_get_samples_per_frame`].)
pub fn packet_get_samples_per_frame(toc: u8, fs: i32) -> i32 {
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

/// Return the number of channels encoded in a packet (1 or 2).
/// RFC 6716 Section 3.1 (stereo bit = bit 2 of TOC).
/// (Safe core of [`opus_packet_get_nb_channels`].)
pub fn packet_get_nb_channels(toc: u8) -> i32 {
    if toc & 0x4 != 0 { 2 } else { 1 }
}

/// Parse a frame size from a VBR packet header.
/// Returns bytes consumed (1 or 2), or -1 on error.
/// Writes frame size to `*size` (-1 on error).
fn parse_size(data: &[u8], len: i32, size: &mut i16) -> i32 {
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
/// self-delimited framing, and padding. Called by `opus_decode_native`
/// (`self_delimited` taken from the packet) and the unit tests.
///
/// `frames` (when given) receives byte *offsets* into `data`, not pointers. The
/// data cursor only advances through the frame payloads when `frames` is
/// requested, which `payload_offset` reflects.
pub fn opus_packet_parse_impl(
    data: &[u8],
    self_delimited: i32,
    out_toc: Option<&mut u8>,
    frames: Option<&mut [i32]>,
    size: &mut [i16],
    payload_offset: Option<&mut i32>,
) -> i32 {
    // `len` is the remaining-bytes counter, initialised from the packet length.
    let mut len = data.len() as i32;

    let toc = *data.first().or_panic("empty packet");
    let framesize = packet_get_samples_per_frame(toc, 48000);

    let mut cbr: i32 = 0;
    let mut off = 1usize;
    len -= 1;
    let mut last_size = len;
    let count: i32;

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
            count = (ch & 0x3F) as i32;
            if count <= 0 || framesize * count > 5760 {
                return OPUS_INVALID_PACKET;
            }
            len -= 1;
            // Padding flag is bit 6
            if ch & 0x40 != 0 {
                let mut padding: i32 = 0;
                let mut p: i32;
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
            cbr = (ch & 0x80 == 0) as i32;
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
            *f = off as i32;
            off += s as usize;
        }
    }

    if let Some(out_toc) = out_toc {
        *out_toc = toc;
    }

    if let Some(payload_offset) = payload_offset {
        *payload_offset = off as i32;
    }

    count
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_get_mode() {
        // SILK-only: bit 7 = 0, bits 6..5 != 11
        assert_eq!(packet_get_mode(0x00), MODE_SILK_ONLY);
        assert_eq!(packet_get_mode(0x40), MODE_SILK_ONLY);
        // Hybrid: bits 7..5 = 011
        assert_eq!(packet_get_mode(0x60), MODE_HYBRID);
        assert_eq!(packet_get_mode(0x70), MODE_HYBRID);
        // CELT-only: bit 7 = 1
        assert_eq!(packet_get_mode(0x80), MODE_CELT_ONLY);
        assert_eq!(packet_get_mode(0xFF), MODE_CELT_ONLY);
    }

    #[test]
    fn test_get_nb_channels() {
        assert_eq!(packet_get_nb_channels(0x00), 1);
        assert_eq!(packet_get_nb_channels(0x04), 2);
        assert_eq!(packet_get_nb_channels(0xFB), 1);
        assert_eq!(packet_get_nb_channels(0xFF), 2);
    }

    #[test]
    fn test_get_bandwidth() {
        assert_eq!(packet_get_bandwidth(0x00), OPUS_BANDWIDTH_NARROWBAND); // SILK NB
        assert_eq!(packet_get_bandwidth(0x20), OPUS_BANDWIDTH_MEDIUMBAND); // SILK MB
        assert_eq!(packet_get_bandwidth(0x40), OPUS_BANDWIDTH_WIDEBAND); // SILK WB
        assert_eq!(packet_get_bandwidth(0x60), OPUS_BANDWIDTH_SUPERWIDEBAND); // Hybrid SWB
        assert_eq!(packet_get_bandwidth(0x70), OPUS_BANDWIDTH_FULLBAND); // Hybrid FB
        assert_eq!(packet_get_bandwidth(0x80), OPUS_BANDWIDTH_NARROWBAND); // CELT NB
        assert_eq!(packet_get_bandwidth(0xA0), OPUS_BANDWIDTH_WIDEBAND); // CELT WB
    }

    #[test]
    fn test_get_samples_per_frame() {
        assert_eq!(packet_get_samples_per_frame(0x00, 48000), 480); // SILK 10ms
        assert_eq!(packet_get_samples_per_frame(0x08, 48000), 960); // SILK 20ms
        assert_eq!(packet_get_samples_per_frame(0x10, 48000), 1920); // SILK 40ms
        assert_eq!(packet_get_samples_per_frame(0x18, 48000), 2880); // SILK 60ms
        assert_eq!(packet_get_samples_per_frame(0x80, 48000), 120); // CELT 2.5ms
        assert_eq!(packet_get_samples_per_frame(0x98, 48000), 960); // CELT 20ms
        assert_eq!(packet_get_samples_per_frame(0x60, 48000), 480); // Hybrid 10ms
        assert_eq!(packet_get_samples_per_frame(0x68, 48000), 960); // Hybrid 20ms
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
        let mut offsets = [0_i32; 48];
        let mut sizes = [0i16; 48];
        let mut offset: i32 = 0;

        let count = opus_packet_parse_impl(&pkt, 0, Some(&mut toc), Some(&mut offsets), &mut sizes, Some(&mut offset));
        assert_eq!(count, 1);
        assert_eq!(toc, 0x80);
        assert_eq!(sizes[0], 3); // 4 bytes total - 1 byte TOC = 3
        assert_eq!(offset, 4); // all bytes consumed
    }

    #[test]
    fn test_packet_parse_code1_two_cbr() {
        // Code 1 = two CBR frames, CELT 2.5ms NB; 6 bytes payload -> 3 each
        let pkt = [0x81u8, 1, 2, 3, 4, 5, 6];
        let mut toc: u8 = 0;
        let mut offsets = [0_i32; 48];
        let mut sizes = [0i16; 48];
        let mut offset: i32 = 0;

        let count = opus_packet_parse_impl(&pkt, 0, Some(&mut toc), Some(&mut offsets), &mut sizes, Some(&mut offset));
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
        let mut sizes = [0i16; 48];
        let count = opus_packet_parse_impl(&pkt, 0, None, None, &mut sizes, None);
        assert_eq!(count, OPUS_INVALID_PACKET);
    }

    #[test]
    fn test_packet_parse_code2_two_vbr() {
        // Code 2 = two VBR frames; size[0] = 2 (single-byte), remaining = last
        let pkt = [0x82u8, 2, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE];
        let mut toc: u8 = 0;
        let mut offsets = [0_i32; 48];
        let mut sizes = [0i16; 48];
        let mut offset: i32 = 0;

        let count = opus_packet_parse_impl(&pkt, 0, Some(&mut toc), Some(&mut offsets), &mut sizes, Some(&mut offset));
        assert_eq!(count, 2);
        assert_eq!(sizes[0], 2);
        // 7 total - 1 TOC - 1 size byte = 5 payload; 5 - 2 = 3 for last
        assert_eq!(sizes[1], 3);
        assert_eq!(offset, 7);
    }

    #[test]
    fn test_packet_parse_frames_none_no_advance() {
        // With no frames out-param, the cursor is not advanced past the frame
        // data, so payload_offset only covers header bytes.
        let pkt = [0x80u8, 0xAA, 0xBB];
        let mut sizes = [0i16; 48];
        let mut offset: i32 = 0;

        let count = opus_packet_parse_impl(&pkt, 0, None, None, &mut sizes, Some(&mut offset));
        assert_eq!(count, 1);
        assert_eq!(sizes[0], 2);
        // frames=None → data not advanced past frames → offset = 1 (TOC only)
        assert_eq!(offset, 1);
    }
}
