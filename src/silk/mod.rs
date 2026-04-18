//! SILK decoder, translated from c/silk/* (RFC 6716).
//!
//! SILK is a fixed-point codec at the source level. Unlike CELT, the SILK
//! decoder uses the same `silk_int32` arithmetic regardless of whether the
//! parent build is the float or fixed-point variant — the float vs fixed
//! split in the original C only affected the SILK *encoder* (silk/float vs
//! silk/fixed). This module is therefore feature-agnostic.

pub mod NLSF2A;
pub mod NLSF_decode;
pub mod biquad_alt;
pub mod bwexpander;
pub mod bwexpander_32;
pub mod code_signs;
pub mod decode_pitch;
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
pub mod pitch_est_tables;
pub mod resampler_down2;
pub mod resampler_down2_3;
pub mod resampler_private_AR2;
pub mod resampler_private_IIR_FIR;
pub mod resampler_private_down_FIR;
pub mod resampler_private_up2_HQ;
pub mod resampler_rom;
pub mod shell_coder;
pub mod sigm_q15;
pub mod sort;
pub mod sqrt_approx;
pub mod stereo_decode_pred;
pub mod stereo_ms_to_lr;
pub mod structs;
pub mod sum_sqr_shift;
pub mod table_lsf_cos;
pub mod tables_gain;
pub mod tables_ltp;
pub mod tables_nlsf_cb_nb_mb;
pub mod tables_nlsf_cb_wb;
pub mod tables_other;
pub mod tables_pitch_lag;
pub mod tables_pulses_per_block;
