//! SILK decoder, translated from c/silk/* (RFC 6716).
//!
//! SILK is a fixed-point codec at the source level. Unlike CELT, the SILK
//! decoder uses the same `silk_int32` arithmetic regardless of whether the
//! parent build is the float or fixed-point variant — the float vs fixed
//! split in the original C only affected the SILK *encoder* (silk/float vs
//! silk/fixed). This module is therefore feature-agnostic.

pub mod bwexpander;
pub mod bwexpander_32;
pub mod gain_quant;
pub mod inner_prod_aligned;
pub mod interpolate;
pub mod lin2log;
pub mod log2lin;
pub mod macros;
pub mod sigm_q15;
pub mod sort;
pub mod sum_sqr_shift;
