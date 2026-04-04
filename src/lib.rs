//! Opuscule: a pure-Rust Opus decoder, incrementally translated from the RFC 6716 reference C implementation.

#![feature(const_trait_impl, const_array)]

pub mod arch;
pub mod celt_lpc;
pub mod cwrs;
pub mod entcode;
pub mod entdec;
pub mod ffi;
pub mod kiss_fft;
pub mod laplace;
pub mod mathops;
pub mod mdct;
pub mod modes;
pub mod packet;
pub mod pitch;
pub mod quant_bands;
pub mod rate;
pub mod vq;

#[cfg(feature = "fixed-point")]
pub mod static_modes_fixed;
#[cfg(not(feature = "fixed-point"))]
pub mod static_modes_float;
