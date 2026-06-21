//! Minimal OGG Opus player: decode an `.opus` file and play it start to finish.
//!
//! Usage: `cargo run --example play -- path/to/file.opus`
//!
//! Demuxes the Ogg container with the `ogg` crate, decodes each Opus packet
//! with `opuscule`, and plays the PCM through the default output device via
//! `rodio`. Both are dev-dependencies, so neither reaches library consumers.

// Demo code: the byte-level header parsing reads cleaner with direct indexing.
#![allow(clippy::indexing_slicing)]

use std::error::Error;
use std::fs::File;
use std::num::{NonZeroU16, NonZeroU32};
use std::process::ExitCode;

use ogg::PacketReader;
use opuscule::{Channels, Decoder, SampleRate, Val, sample_to_i16};
use rodio::buffer::SamplesBuffer;

/// Opus always decodes at 48 kHz internally.
const SAMPLE_RATE: u32 = 48_000;
/// Largest decodable frame: 120 ms at 48 kHz, per channel.
const MAX_FRAME: usize = 5760;

fn main() -> ExitCode {
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("usage: play <file.opus>");
        return ExitCode::FAILURE;
    };
    if let Err(e) = play(&path) {
        eprintln!("error: {e}");
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}

fn play(path: &str) -> Result<(), Box<dyn Error>> {
    let mut packets = PacketReader::new(File::open(path)?);

    // The first two packets are the Opus headers: OpusHead, then OpusTags.
    let head = packets.read_packet_expected()?;
    let (channels, pre_skip) = parse_opus_head(&head.data)?;
    packets.read_packet_expected()?; // OpusTags (metadata) — ignored.

    // Decode every audio packet into one interleaved PCM buffer.
    let mut dec = Decoder::new(SampleRate::Hz48000, channels);
    let ch = channels.count();
    let mut frame = vec![0 as Val; MAX_FRAME * ch];
    let mut pcm: Vec<f32> = Vec::new();
    while let Some(packet) = packets.read_packet()? {
        let samples = dec.decode(Some(&packet.data), &mut frame, false)?;
        // Convert the decoder's native sample type to f32 for rodio. This works
        // for both builds: `Val` is f32 (float) or i16 (fixed-point), and
        // sample_to_i16 normalises either to clamped 16-bit PCM.
        pcm.extend(frame[..samples * ch].iter().map(|&v| f32::from(sample_to_i16(v)) / 32768.0));
    }

    // Drop the encoder delay (pre-skip) from the front.
    let skip = (pre_skip as usize * ch).min(pcm.len());
    let pcm = pcm.split_off(skip);

    println!("Playing {} ({ch} ch, {} samples)...", path, pcm.len() / ch);

    // Play it, blocking until the whole buffer has finished. `stream` owns the
    // output device and must stay alive until playback ends.
    let channels = NonZeroU16::new(ch as u16).ok_or("invalid channel count")?;
    let rate = NonZeroU32::new(SAMPLE_RATE).ok_or("invalid sample rate")?;
    let stream = rodio::DeviceSinkBuilder::open_default_sink()?;
    let player = rodio::Player::connect_new(stream.mixer());
    player.append(SamplesBuffer::new(channels, rate, pcm));
    player.sleep_until_end();
    Ok(())
}

/// Parse an `OpusHead` identification header (RFC 7845 §5.1): returns the
/// channel layout and the pre-skip sample count. Only channel mapping family 0
/// (mono/stereo) is handled — surround would need the multistream decoder.
fn parse_opus_head(data: &[u8]) -> Result<(Channels, u16), Box<dyn Error>> {
    if data.len() < 19 || &data[..8] != b"OpusHead" {
        return Err("not an OpusHead packet".into());
    }
    let pre_skip = u16::from_le_bytes([data[10], data[11]]);
    let mapping_family = data[18];
    if mapping_family != 0 {
        return Err("only channel mapping family 0 (mono/stereo) is supported".into());
    }
    let channels = match data[9] {
        1 => Channels::Mono,
        2 => Channels::Stereo,
        n => return Err(format!("unsupported channel count: {n}").into()),
    };
    Ok((channels, pre_skip))
}
