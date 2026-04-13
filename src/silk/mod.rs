//! SILK decoder, translated from c/silk/* (RFC 6716).
//!
//! SILK is a fixed-point codec at the source level. Unlike CELT, the SILK
//! decoder uses the same `silk_int32` arithmetic regardless of whether the
//! parent build is the float or fixed-point variant — the float vs fixed
//! split in the original C only affected the SILK *encoder* (silk/float vs
//! silk/fixed). This module is therefore feature-agnostic.

pub mod biquad_alt;
pub mod bwexpander;
pub mod bwexpander_32;
pub mod gain_quant;
pub mod inner_prod_aligned;
pub mod interpolate;
pub mod lin2log;
pub mod log2lin;
pub mod lpc_analysis_filter;
pub mod lpc_inv_pred_gain;
pub mod macros;
pub mod nlsf_stabilize;
pub mod nlsf_unpack;
pub mod nlsf_vq_weights_laroia;
pub mod sigm_q15;
pub mod sort;
pub mod structs;
pub mod sum_sqr_shift;
pub mod table_lsf_cos;
