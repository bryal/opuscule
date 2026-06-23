//! Opuscule: a pure-Rust, memory-safe, `no_std`-capable Opus audio decoder.
//!
//! > **Disclaimer: nearly vibe-coded.** Most of this code was produced by an AI
//! > agent translating the C reference. I, the human author, don't understand
//! > much of the signal-processing internals. Correctness rests on bit-exact
//! > verification against the reference vectors, not on line-by-line human review.
//!
//! It decodes [Opus](https://opus-codec.org/) - SILK, CELT, and hybrid modes,
//! mono, stereo, and surround - as a function-by-function translation of the
//! libopus reference decoder (RFC 6716 with the RFC 8251 updates), verified
//! bit-exact against the reference test vectors. The crate is
//! `#![forbid(unsafe_code)]` and allocates nothing on the decode path.

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
