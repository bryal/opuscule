//! Translated from `c/silk/init_decoder.c` (RFC 6716).
//!
//! Initialises a `SilkDecoderState` to its reset value. The reset value
//! lives in the `Default` impl on `SilkDecoderState` (see `structs.rs`),
//! shared with the placeholder the parent `OpusDecoder` constructs.

use super::structs::SilkDecoderState;

/// `silk_init_decoder` — initialise the decoder state struct.
pub fn silk_init_decoder(ps_dec: &mut SilkDecoderState) -> i32 {
    *ps_dec = SilkDecoderState::default();
    0
}
