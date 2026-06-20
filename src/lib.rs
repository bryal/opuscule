//! Opuscule: a pure-Rust Opus decoder, incrementally translated from the RFC 6716 reference C implementation.

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

pub mod arch;
pub mod bands;
pub mod celt;
pub mod celt_lpc;
pub mod cwrs;
pub mod entcode;
pub mod entdec;
pub mod kiss_fft;
pub mod laplace;
pub mod mathops;
pub mod mdct;
pub mod modes;
pub mod opus_decoder;
pub mod opus_multistream;
pub mod packet;
pub mod pitch;
pub mod quant_bands;
pub mod rate;
pub mod silk;
pub mod util;
pub mod vq;

#[cfg(feature = "fixed-point")]
pub mod static_modes_fixed;
#[cfg(not(feature = "fixed-point"))]
pub mod static_modes_float;
