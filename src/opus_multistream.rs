// Translated from c/src/opus_multistream.c (RFC 6716), then reworked off the
// C ABI into idiomatic Rust.
//
// Multistream Opus decoder: handles audio with more than 2 channels (e.g.
// 5.1 surround) by driving an array of regular `OpusDecoder`s and routing PCM
// samples per a channel mapping table.
//
// Unlike the C (and our earlier translation), the sub-decoders are NOT embedded
// in one allocation. The stream count is a runtime value (up to 255), so an
// inline array would be enormous and a `Vec` would defeat the no-alloc goal;
// instead the caller owns the `[OpusDecoder]` storage and passes it in. The
// `OpusMSDecoder` itself is just the channel layout.
//
// Decoder-only: the encoder half of opus_multistream.c is not translated.

use core::ffi::c_int;

use crate::arch::*;
use crate::opus_decoder::{OpusDecoder, opus_decode_native};
use crate::util::OrPanic;

// -- Constants --

const OPUS_BAD_ARG: c_int = -1;
const OPUS_BUFFER_TOO_SMALL: c_int = -2;
const OPUS_INVALID_PACKET: c_int = -4;

/// Largest decodable frame, in samples per channel (120 ms @ 48 kHz). Sizes the
/// per-stream decode scratch.
const MAX_FRAME: usize = 5760;

// -- ChannelLayout --

pub struct ChannelLayout {
    pub nb_channels: c_int,
    pub nb_streams: c_int,
    pub nb_coupled_streams: c_int,
    pub mapping: [u8; 256],
}

// -- OpusMSDecoder --

/// Multistream Opus decoder: the channel layout plus routing logic. The
/// per-stream sub-decoders are supplied by the caller (see [`OpusMSDecoder::decode`]).
pub struct OpusMSDecoder {
    pub layout: ChannelLayout,
}

// -- Layout helpers --

fn validate_layout(layout: &ChannelLayout) -> bool {
    let max_channel = layout.nb_streams + layout.nb_coupled_streams;
    if max_channel > 255 {
        return false;
    }
    let mut i = 0;
    while i < layout.nb_channels {
        let m = *layout.mapping.get(i as usize).or_panic(i) as c_int;
        if m >= max_channel && m != 255 {
            return false;
        }
        i += 1;
    }
    true
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

impl OpusMSDecoder {
    /// Build a multistream decoder from a channel mapping. `mapping` gives, for
    /// each of `channels` output channels, the sub-stream channel index that
    /// feeds it (255 = silence). The caller must then create `streams`
    /// sub-decoders (see [`OpusMSDecoder::stream_channels`]) to pass to
    /// [`decode`](OpusMSDecoder::decode).
    pub fn new(channels: c_int, streams: c_int, coupled_streams: c_int, mapping: &[u8]) -> Result<OpusMSDecoder, c_int> {
        if streams < 1 || coupled_streams > streams || coupled_streams < 0 || channels < 1 {
            return Err(OPUS_BAD_ARG);
        }
        let n = channels as usize;
        let mut layout = ChannelLayout {
            nb_channels: channels,
            nb_streams: streams,
            nb_coupled_streams: coupled_streams,
            mapping: [0u8; 256],
        };
        layout.mapping.get_mut(..n).or_panic(n).copy_from_slice(mapping.get(..n).or_panic(n));
        if !validate_layout(&layout) {
            return Err(OPUS_BAD_ARG);
        }
        Ok(OpusMSDecoder { layout })
    }

    /// Number of sub-streams (the required length of the `decoders` slice).
    pub fn nb_streams(&self) -> usize {
        self.layout.nb_streams as usize
    }

    /// Channel count for sub-decoder stream `s`: 2 for a coupled stream, else 1.
    /// Use this to construct each sub-decoder, e.g.
    /// `OpusDecoder::new(fs, ms.stream_channels(s))`.
    pub fn stream_channels(&self, s: usize) -> c_int {
        if (s as c_int) < self.layout.nb_coupled_streams { 2 } else { 1 }
    }

    /// Decode one multistream packet (`None` = packet loss) into `pcm`
    /// (native sample type, interleaved by output channel). `decoders` is the
    /// caller-owned per-stream sub-decoder array; it must hold at least
    /// [`nb_streams`](OpusMSDecoder::nb_streams) decoders, each created with the
    /// matching [`stream_channels`](OpusMSDecoder::stream_channels). The output
    /// capacity in samples-per-channel is taken from `pcm.len()`. Returns the
    /// number of samples decoded per channel, or a negative Opus error code.
    pub fn decode(
        &mut self,
        decoders: &mut [OpusDecoder],
        packet: Option<&[u8]>,
        pcm: &mut [OpusVal16],
        fec: bool,
    ) -> Result<usize, c_int> {
        let nb_channels = self.layout.nb_channels;
        let nb_streams = self.layout.nb_streams;
        if (decoders.len() as c_int) < nb_streams {
            return Err(OPUS_BAD_ARG);
        }
        let frame_size_cap = pcm.len() / nb_channels.max(1) as usize;
        if frame_size_cap > MAX_FRAME {
            return Err(OPUS_BAD_ARG);
        }
        let mut frame_size = frame_size_cap as c_int;
        let decode_fec = fec as c_int;

        let total_len = packet.map_or(0, |p| p.len()) as c_int;
        let do_plc = total_len == 0;
        if !do_plc && total_len < 2 * nb_streams - 1 {
            return Err(OPUS_INVALID_PACKET);
        }

        // Per-stream decode scratch (one stream's 1- or 2-channel output),
        // sliced to the original frame size (each stream decodes the same).
        let mut buf = [0 as OpusVal16; 2 * MAX_FRAME];
        let buf = buf.get_mut(..2 * frame_size_cap).or_panic(frame_size_cap);

        let mut off = 0usize; // bytes consumed from `packet`
        let mut len = total_len; // bytes remaining
        let mut s = 0;
        while s < nb_streams {
            if !do_plc && len <= 0 {
                return Err(OPUS_INVALID_PACKET);
            }
            // All but the last stream are self-delimited within the packet.
            let self_delimited = if s != nb_streams - 1 { 1 } else { 0 };
            let data_view: Option<&[u8]> = packet.map(|p| p.get(off..).or_panic(off));
            let mut packet_offset: c_int = 0;
            let ret = opus_decode_native(
                decoders.get_mut(s as usize).or_panic(s),
                data_view,
                len,
                buf,
                frame_size,
                decode_fec,
                self_delimited,
                Some(&mut packet_offset),
            );
            off += packet_offset as usize;
            len -= packet_offset;
            if ret > frame_size {
                return Err(OPUS_BUFFER_TOO_SMALL);
            }
            if s > 0 && ret != frame_size {
                return Err(OPUS_INVALID_PACKET);
            }
            if ret <= 0 {
                return Err(ret);
            }
            frame_size = ret;

            if s < self.layout.nb_coupled_streams {
                // Coupled: scatter the stream's interleaved L/R to its channels.
                let mut prev = -1;
                loop {
                    let chan = get_left_channel(&self.layout, s, prev);
                    if chan == -1 {
                        break;
                    }
                    for i in 0..frame_size {
                        let idx = (nb_channels * i + chan) as usize;
                        *pcm.get_mut(idx).or_panic(idx) = *buf.get((2 * i) as usize).or_panic(2 * i);
                    }
                    prev = chan;
                }
                let mut prev = -1;
                loop {
                    let chan = get_right_channel(&self.layout, s, prev);
                    if chan == -1 {
                        break;
                    }
                    for i in 0..frame_size {
                        let idx = (nb_channels * i + chan) as usize;
                        *pcm.get_mut(idx).or_panic(idx) = *buf.get((2 * i + 1) as usize).or_panic(2 * i + 1);
                    }
                    prev = chan;
                }
            } else {
                // Mono: copy the stream to its channel(s).
                let mut prev = -1;
                loop {
                    let chan = get_mono_channel(&self.layout, s, prev);
                    if chan == -1 {
                        break;
                    }
                    for i in 0..frame_size {
                        let idx = (nb_channels * i + chan) as usize;
                        *pcm.get_mut(idx).or_panic(idx) = *buf.get(i as usize).or_panic(i);
                    }
                    prev = chan;
                }
            }
            s += 1;
        }

        // Silence any muted output channels (mapping == 255).
        let mut c = 0;
        while c < nb_channels {
            if *self.layout.mapping.get(c as usize).or_panic(c) == 255 {
                for i in 0..frame_size {
                    let idx = (nb_channels * i + c) as usize;
                    *pcm.get_mut(idx).or_panic(idx) = 0 as OpusVal16;
                }
            }
            c += 1;
        }
        Ok(frame_size as usize)
    }

    /// Combined final range-coder state across all sub-streams (XOR), for the
    /// encoder/decoder consistency check.
    pub fn final_range(&self, decoders: &[OpusDecoder]) -> u32 {
        decoders.iter().take(self.nb_streams()).fold(0u32, |acc, d| acc ^ d.final_range())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::opus_decoder::{Channels, OpusDecoder, SampleRate};

    #[test]
    fn new_validates_args() {
        assert!(OpusMSDecoder::new(2, 1, 1, &[0, 1]).is_ok());
        assert_eq!(OpusMSDecoder::new(2, 0, 0, &[0, 1]).err(), Some(OPUS_BAD_ARG)); // streams < 1
        assert_eq!(OpusMSDecoder::new(2, 1, 2, &[0, 1]).err(), Some(OPUS_BAD_ARG)); // coupled > streams
        // mapping references a channel beyond the stream count
        assert_eq!(OpusMSDecoder::new(2, 1, 0, &[0, 9]).err(), Some(OPUS_BAD_ARG));
    }

    #[test]
    fn stream_layout_queries() {
        // 3 channels: 1 coupled stream (2 ch) + 1 mono stream.
        let ms = OpusMSDecoder::new(3, 2, 1, &[0, 1, 2]).unwrap();
        assert_eq!(ms.nb_streams(), 2);
        assert_eq!(ms.stream_channels(0), 2); // coupled
        assert_eq!(ms.stream_channels(1), 1); // mono
    }

    #[test]
    fn ms_coupled_plc_matches_plain_stereo() {
        // One coupled stream with identity mapping is just a stereo decoder, so
        // a PLC frame must come out byte-identical to a plain stereo decoder's
        // PLC frame (no encoder needed to produce a packet).
        let mut ms = OpusMSDecoder::new(2, 1, 1, &[0, 1]).unwrap();
        let mut decoders =
            [OpusDecoder::new(SampleRate::Hz48000, Channels::from_count(ms.stream_channels(0) as usize).unwrap())];
        let mut plain = OpusDecoder::new(SampleRate::Hz48000, Channels::Stereo);

        let mut ms_pcm = [0 as OpusVal16; 960 * 2];
        let mut plain_pcm = [0 as OpusVal16; 960 * 2];

        let ms_ret = ms.decode(&mut decoders, None, &mut ms_pcm, false).unwrap();
        let plain_ret = plain.decode(None, &mut plain_pcm, false).unwrap();

        // A fresh-decoder PLC yields the default frame (fs/400); the exact size
        // doesn't matter here — what matters is that the multistream routing
        // produces byte-identical output to the plain stereo decoder.
        assert!(ms_ret > 0);
        assert_eq!(ms_ret, plain_ret);
        assert_eq!(ms_pcm, plain_pcm);
        assert_eq!(ms.final_range(&decoders), plain.final_range());
    }
}
