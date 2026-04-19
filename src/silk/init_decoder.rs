//! Translated from `c/silk/init_decoder.c` (RFC 6716).
//!
//! Zero-initialises a `SilkDecoderState` and arms the CNG / PLC
//! sub-state. Called from `silk_Decoder`'s per-channel setup and again
//! from `silk_decoder_set_fs` when the internal sample rate changes.

use super::CNG::silk_CNG_Reset;
use super::PLC::silk_PLC_Reset;
use super::structs::SilkDecoderState;

/// `silk_init_decoder` — initialise the decoder state struct.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_init_decoder(ps_dec: *mut SilkDecoderState) -> i32 {
    unsafe {
        /* Clear the entire decoder state, except anything copied */
        core::ptr::write_bytes(ps_dec, 0, 1);

        /* Used to deactivate LSF interpolation */
        (*ps_dec).first_frame_after_reset = 1;
        (*ps_dec).prev_gain_q16 = 65536;

        /* Reset CNG state */
        silk_CNG_Reset(ps_dec);

        /* Reset PLC state */
        silk_PLC_Reset(ps_dec);

        0
    }
}
