# Opuscule

A pure-Rust, memory-safe, `no_std`-capable **Opus audio decoder**.

> **Disclaimer: nearly vibe-coded.** Most of this code was produced by an AI agent
> translating the C reference. I, the human author, don't understand much of the
> signal-processing internals. Correctness rests on bit-exact verification
> against the reference vectors, not on line-by-line human review.

Opuscule decodes [Opus](https://opus-codec.org/) - the SILK, CELT, and hybrid
modes, mono, stereo, and surround. It is a function-by-function translation of
the libopus reference decoder (RFC 6716 with the RFC 8251 updates), verified
bit-exact against the reference test vectors at every step. See [Scope](#scope)
for exactly what is and isn't implemented.

## Highlights

- **Safe.** `#![forbid(unsafe_code)]` - there is no `unsafe` anywhere in the crate.
- **No allocation.** The decode path allocates nothing on the heap; you supply
  the output buffer.
- **`no_std`.** Runs without the standard library (see the feature matrix below).
- **Bit-exact.** Matches the reference on the RFC 8251 test vectors in both the
  floating-point and fixed-point builds.

## Status

A complete, working decoder - it plays real-world Ogg Opus files start to
finish, and builds on stable Rust.

## Scope

Implements the **Opus decoder** of RFC 6716 with the RFC 8251 corrections applied:

- SILK, CELT, and hybrid modes; all bandwidths (narrowband through fullband).
- Mono, stereo, and multistream/surround (via `MsDecoder`).
- Packet loss concealment (PLC) and in-band forward error correction (FEC).
- All RFC 8251 fixes, including hybrid folding (§9) and the mono-downmix
  phase-inversion option (§10).

Not included:

- **No encoder** - decoding only.
- **No container parsing** - you feed it raw Opus packets and demux Ogg / WebM /
  RTP yourself (the `play` example uses the `ogg` crate for this).
- **No non-RFC libopus extensions** - the 24-bit / 96 kHz work ("QEXT") and the
  neural tools (DRED, Deep PLC, OSCE) are out of scope.

## Usage

```rust
use opuscule::{Channels, Decoder, SampleRate};

// Opus always decodes at 48 kHz internally; choose the channel layout.
let mut decoder = Decoder::new(SampleRate::Hz48000, Channels::Stereo);

// `packet` is one Opus packet from your container (Ogg, WebM, RTP, ...).
// The output buffer holds up to 120 ms per channel at 48 kHz.
let mut pcm = vec![0.0f32; 5760 * 2];
let samples = decoder.decode(Some(packet), &mut pcm, false)?;
```

`examples/play.rs` is a complete CLI player: it demuxes an `.opus` file with the
`ogg` crate and plays it through `rodio`.

```sh
cargo run --example play -- path/to/file.opus
```

## Feature flags

The crate is floating-point + `std` by default. Cargo features are additive, so
a `no_std` build means turning off the default `std` feature:

| Build | Flags | Notes |
|-------|-------|-------|
| float + std (default) | (none) | system math; the bit-exact baseline |
| fixed-point, `no_std` | `--no-default-features --features fixed-point` | integer kernels, no math dependency |
| float, `no_std` | `--no-default-features --features libm` | float math via the pure-Rust `libm` crate |
| float, no math lib | `--no-default-features` | a `compile_error!` explains the options |

The `libm` float path is not vector-checked and may differ by an ULP from the
system libm; the `std`-float and fixed-point builds are bit-exact.

## License

This software is released under the **Mozilla Public License 2.0** (see
[`LICENSE`](LICENSE)). Copyright (c) 2026 Jojo <jo@jo.zone>.

Opuscule is a derivative work: its decoder was translated, function by function,
from the [libopus](https://gitlab.xiph.org/xiph/opus) reference implementation
of RFC 6716, which is distributed under the 3-clause BSD license. That upstream
license, its copyright notices, and the Opus patent grants are retained verbatim
in [`LICENSE-XIPH`](LICENSE-XIPH).
