//! What it costs to decode, measured against real bitstreams.
//!
//! Reported as **samples per second of wall clock**, which converts straight into the number a
//! player cares about: at 48 kHz, `48000 / (samples per second)` is the fraction of one CPU core
//! that continuous playback spends in here. That is the same unit a per-thread CPU census reports,
//! so the two can be checked against each other instead of each being believed on its own.
//!
//! The inputs are the RFC reference vectors already in the tree (`opus_testvectors/`),
//! parsed the same way as in `src/bin/opus_demo.rs`.
//! Real bitstreams rather than synthesised ones because we don't have access to an Opus *encoder*.
//!
//! Which vector matters depends on what you are asking:
//!
//! - `celt_fb_stereo_20ms` (vector 11) is 553 packets of CELT-only fullband stereo at 20 ms and
//!   nothing else, which is what a music library encoded by opusenc decodes as. The headline.
//! - `celt_fb_mixed` (vector 09) adds short blocks, so it walks the transient path where the inverse
//!   MDCT runs sixteen times a packet instead of twice.
//! - `hybrid_fb` (vector 06) and `silk_wb` (vector 04) are speech-shaped, and are here to show which
//!   paths a change did *not* touch.

#![allow(clippy::unwrap_used, clippy::indexing_slicing)]

use criterion::{Criterion, Throughput, criterion_group, criterion_main};
use opuscule::{Channels, Decoder, SampleRate, Val, sample_to_i16};
use std::hint::black_box;
use std::path::PathBuf;

/// The largest frame Opus can carry: 120 ms at 48 kHz, both channels.
const MAX_FRAME: usize = 5760 * 2;

fn vector_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("opus_testvectors").join(name)
}

/// Splits an `opus_demo` container into its packets.
fn packets(raw: &[u8]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let mut at = 0;
    while at + 8 <= raw.len() {
        let len = u32::from_be_bytes(raw[at..at + 4].try_into().unwrap()) as usize;
        let body = at + 8;
        if body + len > raw.len() {
            break; // exit early rather than error on truncation, same as `opus_demo`
        }
        out.push(raw[body..body + len].to_vec());
        at = body + len;
    }
    out
}

/// Decodes every packet once, returning the samples-per-channel produced.
/// Used both to size the throughput before timing starts and as the timed routine itself.
fn decode_all(dec: &mut Decoder, packets: &[Vec<u8>], pcm: &mut [Val]) -> usize {
    let mut total = 0;
    for packet in packets {
        total += dec.decode(Some(packet), pcm, false).unwrap();
    }
    total
}

fn decode(c: &mut Criterion) {
    let mut group = c.benchmark_group("decode");

    for (name, file) in [
        ("celt_fb_stereo_20ms", "testvector11.bit"),
        ("celt_fb_mixed", "testvector09.bit"),
        ("hybrid_fb", "testvector06.bit"),
        ("silk_wb", "testvector04.bit"),
    ] {
        let path = vector_path(file);
        let Ok(raw) = std::fs::read(&path) else {
            println!("skipping decode/{name}: no {}", path.display());
            continue;
        };
        let packets = packets(&raw);
        assert!(!packets.is_empty(), "no packets in {}", path.display());

        // One decoder for the whole run rather than a fresh one per iteration.
        // The seam where the vector wraps back to its first packet is one packet in several hundred.
        let mut pcm = vec![0 as Val; MAX_FRAME];
        let mut dec = Decoder::new(SampleRate::Hz48000, Channels::Stereo);
        let samples = decode_all(&mut dec, &packets, &mut pcm);

        // Samples per channel, so 48000 divided by elements-per-second is the fraction of a core.
        group.throughput(Throughput::Elements(samples as u64));
        group.bench_function(name, |b| {
            b.iter(|| black_box(decode_all(&mut dec, black_box(&packets), &mut pcm)));
        });
    }

    group.finish();
}

/// One sample of something shaped like decoded audio,
/// near enough to full scale that `float2int16`'s clamp is a live branch rather than one never taken.
fn wave(t: f32) -> Val {
    cfg_select! {
        feature = "fixed-point" => (29_490.0 * (t * 220.0).sin()) as Val,
        _ => 0.9 * (t * 220.0).sin(),
    }
}

/// The conversion a player does on every decoded sample, timed from *outside* the crate - which is
/// the only place it can be timed honestly. `sample_to_i16` is a four-instruction body behind a
/// cross-crate call, so whether it costs anything at all comes down to it being inlinable from here,
/// and a bench living inside the crate would inline it regardless and measure nothing.
///
/// One second of 48 kHz stereo is 96,000 of these, which is the rate a player pays.
fn conversion(c: &mut Criterion) {
    let mut group = c.benchmark_group("conversion");

    let frame: Vec<Val> = (0..96_000).map(|i| wave(i as f32 / 96_000.0)).collect();

    group.throughput(Throughput::Elements(frame.len() as u64));
    group.bench_function("sample_to_i16_96k", |b| {
        b.iter(|| {
            let mut acc = 0i32;
            for &s in black_box(&frame) {
                acc = acc.wrapping_add(sample_to_i16(s) as i32);
            }
            black_box(acc)
        });
    });

    group.finish();
}

criterion_group!(benches, decode, conversion);
criterion_main!(benches);
