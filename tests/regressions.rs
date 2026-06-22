//! Decoder regression and contract tests. Each test decodes a captured packet
//! or exercises a decode-dispatch contract that the standard RFC test vectors
//! don't cover. Add new tests as further functions here, not new files.

use opuscule::{Channels, Decoder, Error, MsDecoder, SampleRate, Val};

/// A real 20 ms CELT fullband stereo frame (TOC 0xfc), lifted from an
/// ffmpeg-encoded `.opus` file. Used to prime a decoder with real state and as
/// a known CELT-only packet. 960 samples per channel when decoded.
#[rustfmt::skip]
const CELT_FB_STEREO_FRAME: &[u8] = &[
    0xfc, 0xb0, 0x1f, 0xc9, 0xaa, 0x9e, 0xf0, 0xb8, 0xa4, 0x94, 0xd9, 0xa1,
    0xd8, 0x75, 0xa1, 0x9e, 0x2d, 0xb4, 0x71, 0x72, 0x76, 0xdd, 0xdb, 0x3e,
    0xd6, 0x6d, 0x65, 0x19, 0x38, 0xd1, 0x8f, 0xab, 0xa9, 0x37, 0x04, 0xe7,
    0x52, 0x3e, 0x75, 0xcf, 0xca, 0x8f, 0x97, 0x7f, 0x4b, 0xb4, 0xa7, 0xd4,
    0xcc, 0x8b, 0xe9, 0x02, 0xc4, 0xea, 0x29, 0xbf, 0x35, 0x24, 0x1a, 0x3f,
    0x76, 0xac, 0xb2, 0xc9, 0x57, 0xa3, 0x57, 0x6c, 0xf7, 0x12, 0x61, 0xf2,
    0x3e, 0x75, 0xa8, 0x74, 0xb3, 0x9a, 0x60, 0x83, 0x22, 0x49, 0xe0, 0x45,
    0x2c, 0x0f, 0x4b, 0xe1, 0xc4, 0xdc, 0xfd, 0x2f, 0x6e, 0x26, 0xfa, 0x1c,
    0x00, 0x4e, 0xca, 0x2d, 0xfc, 0x54, 0x88, 0x4f, 0xf0, 0x77, 0xe7, 0xac,
    0x5f, 0xb7, 0xa4, 0x67, 0x94, 0x34, 0xb6, 0xa6, 0x6c, 0x99, 0x68, 0xa6,
    0xd0, 0x94, 0x19, 0x5a, 0xa1, 0xc4, 0xab, 0x9f, 0x02, 0xcf, 0x1c, 0xf7,
    0x1e, 0x67, 0x96, 0x4e, 0x43, 0x1b, 0x14, 0x11, 0xb7, 0x17, 0xbd, 0x9a,
    0x77, 0x19, 0x8f, 0x17, 0x26, 0x37, 0x68, 0xe8, 0xe3, 0xd2, 0xf7, 0x3d,
    0x59, 0xda, 0x31, 0xc2, 0x96, 0x10, 0x5d, 0xeb, 0x55, 0xe5, 0x1c, 0x45,
    0xf9, 0x85, 0xa8, 0x49, 0x1a, 0xe2, 0x41, 0x31, 0xab, 0x20, 0x3e, 0x18,
    0xfd, 0x34, 0x00, 0x81, 0x91, 0xa5, 0x27, 0xaf, 0xf8, 0x07, 0x54, 0xbc,
    0x9d, 0xc0, 0x17, 0x66, 0x6f, 0x78, 0x43, 0xee,
];

/// A long (non-transient) CELT block underflowed the inverse-MDCT output base
/// index (`out_off - ((n2 - overlap) >> 1)` with `out_off == 0`), panicking in
/// debug at mdct.rs with "attempt to subtract with overflow". Decoding it must
/// complete and yield one 20 ms frame.
#[test]
fn celt_long_block_imdct_no_overflow() {
    let mut dec = Decoder::new(SampleRate::Hz48000, Channels::Stereo);
    // 120 ms at 48 kHz, stereo - the largest frame the decoder can emit.
    let mut pcm = vec![0 as Val; 5760 * 2];
    // A 20 ms frame is 960 samples per channel.
    assert_eq!(dec.decode(Some(CELT_FB_STEREO_FRAME), &mut pcm, false), Ok(960));
}

/// RFC 8251 7fcd66c4 contract: a PLC call (`packet == None`) conceals exactly
/// the requested duration, looping internally for buffers longer than 20 ms,
/// rather than returning just one 20 ms frame.
#[test]
fn plc_returns_exactly_requested_frame_size() {
    let mut dec = Decoder::new(SampleRate::Hz48000, Channels::Stereo);
    // Prime the decoder with a real (CELT) packet so the PLC has prior state.
    let mut pcm = vec![0 as Val; 5760 * 2];
    assert_eq!(dec.decode(Some(CELT_FB_STEREO_FRAME), &mut pcm, false), Ok(960));

    // Conceal 60 ms (2880 samples/channel) = three 20 ms PLC frames. The whole
    // buffer must be filled and the full count returned.
    let mut plc = vec![0 as Val; 2880 * 2];
    assert_eq!(dec.decode(None, &mut plc, false), Ok(2880));
}

/// RFC 8251 7fcd66c4 contract: for PLC/FEC the frame size must be a multiple of
/// 2.5 ms; a non-multiple output buffer is rejected rather than silently mishandled.
#[test]
fn plc_frame_size_must_be_2p5ms_multiple() {
    let mut dec = Decoder::new(SampleRate::Hz48000, Channels::Mono);
    // 200 samples is not a multiple of 2.5 ms (120 samples at 48 kHz).
    let mut pcm = vec![0 as Val; 200];
    assert_eq!(dec.decode(None, &mut pcm, false), Err(Error::BadArg));
}

/// RFC 8251 7fcd66c4 FEC dispatch: CELT-only packets carry no in-band FEC, so a
/// FEC-requested decode of one falls back to PLC and still returns frame_size.
/// (The LBRR decode path for SILK/hybrid FEC reuses the existing decode_fec
/// machinery and isn't covered here - it needs an FEC-encoded SILK packet.)
#[test]
fn fec_request_on_celt_packet_falls_back_to_plc() {
    let mut dec = Decoder::new(SampleRate::Hz48000, Channels::Stereo);
    let mut pcm = vec![0 as Val; 960 * 2];
    assert_eq!(dec.decode(Some(CELT_FB_STEREO_FRAME), &mut pcm, true), Ok(960));
}

/// libopus ed463234: a multistream packet is validated up front, so a malformed
/// packet is rejected before any sub-decoder runs, leaving them undisturbed for
/// the next (valid) call. A single coupled stream is just a stereo decoder, so
/// the CELT stereo frame above serves as a valid one-stream multistream packet.
#[test]
fn multistream_validates_packet_before_decoding() {
    let mut ms = MsDecoder::new(2, 1, 1, &[0, 1]).expect("valid layout");
    let mut decoders = [Decoder::new(SampleRate::Hz48000, ms.stream_channels(0))];
    let mut pcm = vec![0 as Val; 960 * 2];

    // A malformed packet (TOC code 3 claiming far more frames than can fit) is
    // rejected up front, not partially decoded.
    assert!(ms.decode(&mut decoders, Some(&[0xff, 0xff]), &mut pcm, false).is_err());

    // The sub-decoder is untouched, so a valid packet still decodes normally.
    assert_eq!(ms.decode(&mut decoders, Some(CELT_FB_STEREO_FRAME), &mut pcm, false), Ok(960));
}
