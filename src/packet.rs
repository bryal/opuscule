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
        if bandwidth == OPUS_BANDWIDTH_MEDIUMBAND {
            OPUS_BANDWIDTH_NARROWBAND
        } else {
            bandwidth
        }
    } else if toc & 0x60 == 0x60 {
        if toc & 0x10 != 0 {
            OPUS_BANDWIDTH_FULLBAND
        } else {
            OPUS_BANDWIDTH_SUPERWIDEBAND
        }
    } else {
        OPUS_BANDWIDTH_NARROWBAND + ((toc >> 5) & 0x3) as c_int
    }
}

/// Return the number of samples per frame from the TOC byte and sample rate.
/// RFC 6716 Section 3.1.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_samples_per_frame(
    data: *const u8,
    fs: i32,
) -> c_int {
    let toc = unsafe { *data };
    if toc & 0x80 != 0 {
        let audiosize = ((toc >> 3) & 0x3) as i32;
        (fs << audiosize) / 400
    } else if toc & 0x60 == 0x60 {
        if toc & 0x08 != 0 {
            fs / 50
        } else {
            fs / 100
        }
    } else {
        let audiosize = ((toc >> 3) & 0x3) as i32;
        if audiosize == 3 {
            fs * 60 / 1000
        } else {
            (fs << audiosize) / 100
        }
    }
}

/// Return the number of channels encoded in a packet (1 or 2).
/// RFC 6716 Section 3.1 (stereo bit = bit 2 of TOC).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_nb_channels(data: *const u8) -> c_int {
    if unsafe { *data } & 0x4 != 0 {
        2
    } else {
        1
    }
}

/// Return the number of frames in an Opus packet.
/// RFC 6716 Section 3.2.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_packet_get_nb_frames(
    packet: *const u8,
    len: c_int,
) -> c_int {
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
        assert_eq!(
            unsafe { opus_packet_get_nb_frames([0x03, 0x05].as_ptr(), 2) },
            5
        );
        // Code 3, len < 2
        assert_eq!(
            unsafe { opus_packet_get_nb_frames([0x03].as_ptr(), 1) },
            OPUS_INVALID_PACKET
        );
        // Empty
        assert_eq!(
            unsafe { opus_packet_get_nb_frames([0x00].as_ptr(), 0) },
            OPUS_BAD_ARG
        );
    }

    #[test]
    fn test_get_bandwidth() {
        // SILK NB
        assert_eq!(
            unsafe { opus_packet_get_bandwidth(&0x00u8) },
            OPUS_BANDWIDTH_NARROWBAND
        );
        // SILK MB
        assert_eq!(
            unsafe { opus_packet_get_bandwidth(&0x20u8) },
            OPUS_BANDWIDTH_MEDIUMBAND
        );
        // SILK WB
        assert_eq!(
            unsafe { opus_packet_get_bandwidth(&0x40u8) },
            OPUS_BANDWIDTH_WIDEBAND
        );
        // Hybrid SWB
        assert_eq!(
            unsafe { opus_packet_get_bandwidth(&0x60u8) },
            OPUS_BANDWIDTH_SUPERWIDEBAND
        );
        // Hybrid FB
        assert_eq!(
            unsafe { opus_packet_get_bandwidth(&0x70u8) },
            OPUS_BANDWIDTH_FULLBAND
        );
        // CELT NB (bits 6..5 = 00 maps to MEDIUMBAND then corrected to NB)
        assert_eq!(
            unsafe { opus_packet_get_bandwidth(&0x80u8) },
            OPUS_BANDWIDTH_NARROWBAND
        );
        // CELT WB
        assert_eq!(
            unsafe { opus_packet_get_bandwidth(&0xA0u8) },
            OPUS_BANDWIDTH_WIDEBAND
        );
    }

    #[test]
    fn test_get_samples_per_frame() {
        // SILK 10ms @ 48kHz
        assert_eq!(
            unsafe { opus_packet_get_samples_per_frame(&0x00u8, 48000) },
            480
        );
        // SILK 20ms
        assert_eq!(
            unsafe { opus_packet_get_samples_per_frame(&0x08u8, 48000) },
            960
        );
        // SILK 40ms
        assert_eq!(
            unsafe { opus_packet_get_samples_per_frame(&0x10u8, 48000) },
            1920
        );
        // SILK 60ms
        assert_eq!(
            unsafe { opus_packet_get_samples_per_frame(&0x18u8, 48000) },
            2880
        );
        // CELT 2.5ms @ 48kHz
        assert_eq!(
            unsafe { opus_packet_get_samples_per_frame(&0x80u8, 48000) },
            120
        );
        // CELT 20ms
        assert_eq!(
            unsafe { opus_packet_get_samples_per_frame(&0x98u8, 48000) },
            960
        );
        // Hybrid 10ms
        assert_eq!(
            unsafe { opus_packet_get_samples_per_frame(&0x60u8, 48000) },
            480
        );
        // Hybrid 20ms
        assert_eq!(
            unsafe { opus_packet_get_samples_per_frame(&0x68u8, 48000) },
            960
        );
    }
}
