//! Opuscule: a pure-Rust, memory-safe, `no_std`-capable Opus audio decoder.
//!
//! It decodes [Opus](https://opus-codec.org/) (RFC 6716) - SILK, CELT, and
//! hybrid, mono or stereo - translated function by function from the libopus
//! reference and verified bit-exact against the RFC test vectors. The crate is
//! `#![forbid(unsafe_code)]` and allocates nothing on the decode path. It is a
//! decoder only; there is no encoder.
//!
//! # Example
//!
//! ```no_run
//! use opuscule::{Channels, Decoder, SampleRate, Val};
//!
//! # fn next_packet() -> Option<&'static [u8]> { None }
//! // Opus always decodes at 48 kHz; pick the channel layout of your stream.
//! let mut decoder = Decoder::new(SampleRate::Hz48000, Channels::Stereo);
//!
//! // Output holds up to 120 ms per channel at 48 kHz. `Val` is the decoder's
//! // sample type: f32 by default, or i16 in the fixed-point build.
//! let mut pcm = vec![0 as Val; 5760 * 2];
//!
//! // Feed one Opus packet at a time from your container (Ogg, WebM, RTP, ...).
//! while let Some(packet) = next_packet() {
//!     let samples = decoder.decode(Some(packet), &mut pcm, false)?;
//!     // `pcm[..samples * 2]` now holds interleaved stereo samples.
//! }
//! # Ok::<(), opuscule::Error>(())
//! ```
//!
//! See `examples/play.rs` for a complete Ogg Opus file player.

#![cfg_attr(not(feature = "std"), no_std)]
#![feature(const_trait_impl, const_array)]
// The decoder is implemented entirely in safe Rust. `forbid` (not `deny`)
// makes reintroducing `unsafe` a hard error that cannot be locally
// overridden; relax to `deny` if a vetted unsafe block ever earns its place.
#![forbid(unsafe_code)]

// The floating-point decoder needs a math library (sqrt/cos/ln/exp/floor):
// `std` provides it, or the `libm` feature pulls in the pure-Rust `libm` crate
// for no_std. The fixed-point decoder uses only integer kernels, so it needs
// neither and is no_std for free.
#[cfg(all(not(feature = "std"), not(feature = "fixed-point"), not(feature = "libm")))]
compile_error!(
    "the floating-point build is not no_std on its own: keep the default `std` feature, \
     add the `libm` feature for a no_std float build, or build `--features fixed-point`"
);

// Everything below is decoder internals - private to the crate. The public
// API is the curated re-export list at the bottom of this file.
mod arch;
mod bands;
mod celt;
mod celt_lpc;
mod cwrs;
mod entcode;
mod entdec;
mod error;
mod kiss_fft;
mod laplace;
mod mathops;
mod mdct;
mod modes;
mod opus_decoder;
mod opus_multistream;
mod packet;
mod pitch;
mod quant_bands;
mod rate;
mod silk;
mod util;
mod vq;

#[cfg(feature = "fixed-point")]
mod static_modes_fixed;
#[cfg(not(feature = "fixed-point"))]
mod static_modes_float;

// -- Public API --
//
// The decoder's native sample type (`f32` for the float build, `i16` for
// fixed-point); decode writes interleaved samples of this type.
pub use arch::Val;
pub use error::Error;
pub use opus_decoder::{Channels, Decoder, SampleRate, sample_to_i16};
pub use opus_multistream::MsDecoder;
