# Opuscule

A pure-Rust, memory-safe, `no_std`-capable **Opus audio decoder**.

Opuscule decodes [Opus](https://opus-codec.org/) (RFC 6716) - the SILK, CELT,
and hybrid modes, mono or stereo, at the standard sample rates. It was
translated function by function from the libopus reference decoder and verified
bit-exact against the RFC test vectors at every step.

## Highlights

- **Safe.** `#![forbid(unsafe_code)]` - there is no `unsafe` anywhere in the crate.
- **No allocation.** The decode path allocates nothing on the heap; you supply
  the output buffer.
- **`no_std`.** Runs without the standard library (see the feature matrix below).
- **Bit-exact.** Matches the libopus reference on the RFC 6716 vectors in both
  the floating-point and fixed-point builds.

## Status

v0.1: a complete, working decoder - it plays real-world Ogg Opus files start to
finish. **Requires a nightly toolchain** (the crate uses the unstable
`const_trait_impl` and `const_array` features); a stable build is a goal for a
later release.

It is a **decoder only** - there is no Opus encoder here.

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
