// Opuscule: a pure-Rust Opus decoder, incrementally translated from the
// RFC 6716 reference C implementation.

pub mod arch;
pub mod celt_lpc;
pub mod cwrs;
pub mod entcode;
pub mod entdec;
pub mod ffi;
pub mod laplace;
pub mod mathops;
pub mod packet;
pub mod quant_bands;
pub mod rate;
pub mod vq;
