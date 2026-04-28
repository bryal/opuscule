// Translated from c/src/opus_decoder.c (RFC 6716 Section 3.1).
//
// Opus TOC byte query functions: mode, bandwidth, frame size,
// channel count, frame count. Pure bitfield reads with no
// float/fixed-point dependency.

use std::os::raw::c_int;

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
/// RFC 6716 Section 3.1, Table 2.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_mode(data: *const u8) -> c_int {
    let toc = unsafe { *data };
    if toc & 0x80 != 0 {
        MODE_CELT_ONLY
    } else if toc & 0x60 == 0x60 {
        MODE_HYBRID
    } else {
        MODE_SILK_ONLY
    }
}

/// Return the bandwidth of an Opus packet from its TOC byte.
/// RFC 6716 Section 3.1, Table 2.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_bandwidth(data: *const u8) -> c_int {
    let toc = unsafe { *data };
    if toc & 0x80 != 0 {
        let bandwidth = OPUS_BANDWIDTH_MEDIUMBAND + ((toc >> 5) & 0x3) as c_int;
        if bandwidth == OPUS_BANDWIDTH_MEDIUMBAND { OPUS_BANDWIDTH_NARROWBAND } else { bandwidth }
    } else if toc & 0x60 == 0x60 {
        if toc & 0x10 != 0 { OPUS_BANDWIDTH_FULLBAND } else { OPUS_BANDWIDTH_SUPERWIDEBAND }
    } else {
        OPUS_BANDWIDTH_NARROWBAND + ((toc >> 5) & 0x3) as c_int
    }
}

/// Return the number of samples per frame from the TOC byte and sample rate.
/// RFC 6716 Section 3.1.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_samples_per_frame(data: *const u8, fs: i32) -> c_int {
    let toc = unsafe { *data };
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
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_nb_channels(data: *const u8) -> c_int {
    if unsafe { *data } & 0x4 != 0 { 2 } else { 1 }
}

/// Return the number of frames in an Opus packet.
/// RFC 6716 Section 3.2.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_nb_frames(packet: *const u8, len: c_int) -> c_int {
    if len < 1 {
        return OPUS_BAD_ARG;
    }
    let toc = unsafe { *packet };
    let count = toc & 0x3;
    if count == 0 {
        1
    } else if count != 3 {
        2
    } else if len < 2 {
        OPUS_INVALID_PACKET
    } else {
        (unsafe { *packet.add(1) } & 0x3F) as c_int
    }
}

/// Parse a frame size from a VBR packet header.
/// Returns bytes consumed (1 or 2), or -1 on error.
/// Writes frame size to `*size` (-1 on error).
fn parse_size(data: *const u8, len: c_int, size: *mut i16) -> c_int {
    unsafe {
        if len < 1 {
            *size = -1;
            return -1;
        } else if (*data) < 252 {
            *size = (*data) as i16;
            return 1;
        } else if len < 2 {
            *size = -1;
            return -1;
        } else {
            *size = 4 * (*data.add(1)) as i16 + (*data) as i16;
            return 2;
        }
    }
}

/// Parse an Opus packet into TOC byte, frame pointers, sizes, and payload offset.
/// RFC 6716 Section 3.2.
///
/// This handles all four code types (0-3) including CBR/VBR multi-frame packets,
/// self-delimited framing, and padding. Called by both `opus_packet_parse`
/// (self_delimited=0) and `opus_decode_native` (self_delimited from packet).
pub unsafe fn opus_packet_parse_impl(
    data: *const u8,
    len: c_int,
    self_delimited: c_int,
    out_toc: *mut u8,
    frames: *mut *const u8,
    size: *mut i16,
    payload_offset: *mut c_int,
) -> c_int {
    unsafe {
        if size.is_null() {
            return OPUS_BAD_ARG;
        }

        let data0 = data;
        let mut data = data;
        let mut len = len;

        let framesize = opus_packet_get_samples_per_frame(data, 48000);

        let mut cbr: c_int = 0;
        let toc = *data;
        data = data.add(1);
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
                    *size.add(0) = last_size as i16;
                }
            }
            // Two VBR frames
            2 => {
                count = 2;
                let bytes = parse_size(data, len, size);
                len -= bytes;
                if *size < 0 || (*size) as c_int > len {
                    return OPUS_INVALID_PACKET;
                }
                data = data.add(bytes as usize);
                last_size = len - (*size) as c_int;
            }
            // Multiple CBR/VBR frames (from 0 to 120 ms)
            _ => {
                if len < 1 {
                    return OPUS_INVALID_PACKET;
                }
                // Number of frames encoded in bits 0 to 5
                let ch = *data;
                data = data.add(1);
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
                        p = (*data) as c_int;
                        data = data.add(1);
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
                    for i in 0..count - 1 {
                        let bytes = parse_size(data, len, size.add(i as usize));
                        len -= bytes;
                        if *size.add(i as usize) < 0 || (*size.add(i as usize)) as c_int > len {
                            return OPUS_INVALID_PACKET;
                        }
                        data = data.add(bytes as usize);
                        last_size -= bytes + (*size.add(i as usize)) as c_int;
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
                    for i in 0..count - 1 {
                        *size.add(i as usize) = last_size as i16;
                    }
                }
            }
        }

        // Self-delimited framing has an extra size for the last frame.
        if self_delimited != 0 {
            let bytes = parse_size(data, len, size.add(count as usize - 1));
            len -= bytes;
            if *size.add(count as usize - 1) < 0 || (*size.add(count as usize - 1)) as c_int > len {
                return OPUS_INVALID_PACKET;
            }
            data = data.add(bytes as usize);
            // For CBR packets, apply the size to all the frames.
            if cbr != 0 {
                if (*size.add(count as usize - 1)) as c_int * count > len {
                    return OPUS_INVALID_PACKET;
                }
                for i in 0..count - 1 {
                    *size.add(i as usize) = *size.add(count as usize - 1);
                }
            } else if (*size.add(count as usize - 1)) as c_int > last_size {
                return OPUS_INVALID_PACKET;
            }
        } else {
            // Because it's not encoded explicitly, it's possible the size of the
            // last packet (or all the packets, for the CBR case) is larger than
            // 1275. Reject them here.
            if last_size > 1275 {
                return OPUS_INVALID_PACKET;
            }
            *size.add(count as usize - 1) = last_size as i16;
        }

        if !frames.is_null() {
            for i in 0..count {
                *frames.add(i as usize) = data;
                data = data.add((*size.add(i as usize)) as usize);
            }
        }

        if !out_toc.is_null() {
            *out_toc = toc;
        }

        if !payload_offset.is_null() {
            *payload_offset = data.offset_from(data0) as c_int;
        }

        count
    }
}

/// Parse an Opus packet (non-self-delimited).
/// RFC 6716 Section 3.2.
///
/// Public entry point that wraps `opus_packet_parse_impl` with
/// `self_delimited = 0`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_parse(
    data: *const u8,
    len: c_int,
    out_toc: *mut u8,
    frames: *mut *const u8,
    size: *mut i16,
    payload_offset: *mut c_int,
) -> c_int {
    unsafe { opus_packet_parse_impl(data, len, 0, out_toc, frames, size, payload_offset) }
}

#[cfg(test)]
mod tests {
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
        assert_eq!(parse_size([100u8].as_ptr(), 1, &mut size), 1);
        assert_eq!(size, 100);

        // Two-byte size (>= 252): 4 * data[1] + data[0]
        assert_eq!(parse_size([252u8, 10].as_ptr(), 2, &mut size), 2);
        assert_eq!(size, 4 * 10 + 252);

        // Empty input
        assert_eq!(parse_size([0u8].as_ptr(), 0, &mut size), -1);
        assert_eq!(size, -1);

        // >= 252 but only 1 byte available
        size = 0;
        assert_eq!(parse_size([252u8].as_ptr(), 1, &mut size), -1);
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
