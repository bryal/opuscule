//! Translated from `c/silk/resampler.c` (RFC 6716).
//!
//! Resampler initialisation and top-level dispatcher. Supports the
//! five Opus-internal sampling rates (8/12/16/24/48 kHz) via a small
//! table of sub-sampler selections — see the matrix at the top of the
//! C source for the combinations each pair of rates uses.

use super::macros::{silk_lshift, silk_smulww};
use super::resampler_private_down_fir::silk_resampler_private_down_fir;
use super::resampler_private_iir_fir::silk_resampler_private_iir_fir;
use super::resampler_private_up2_hq::silk_resampler_private_up2_hq_wrapper;
use super::resampler_rom::{
    RESAMPLER_DOWN_ORDER_FIR0, RESAMPLER_DOWN_ORDER_FIR1, RESAMPLER_DOWN_ORDER_FIR2, SILK_RESAMPLER_1_2_COEFS,
    SILK_RESAMPLER_1_3_COEFS, SILK_RESAMPLER_1_4_COEFS, SILK_RESAMPLER_1_6_COEFS, SILK_RESAMPLER_2_3_COEFS,
    SILK_RESAMPLER_3_4_COEFS,
};
use super::structs::{SILK_RESAMPLER_MAX_FIR_ORDER, SILK_RESAMPLER_MAX_IIR_ORDER, SilkResamplerStateStruct};

const RESAMPLER_MAX_BATCH_SIZE_MS: i32 = 10;

/* Tables with delay compensation values to equalize total delay for different modes */
static DELAY_MATRIX_ENC: [[i8; 3]; 5] = [
    /* in  \ out  8  12  16 */
    /*  8 */ [6, 0, 3],
    /* 12 */ [0, 7, 3],
    /* 16 */ [0, 1, 10],
    /* 24 */ [0, 2, 6],
    /* 48 */ [18, 10, 12],
];

static DELAY_MATRIX_DEC: [[i8; 5]; 3] = [
    /* in  \ out  8  12  16  24  48 */
    /*  8 */ [4, 0, 2, 0, 0],
    /* 12 */ [0, 9, 4, 7, 4],
    /* 16 */ [0, 3, 12, 7, 7],
];

/// Simple way to map [8000, 12000, 16000, 24000, 48000] to [0, 1, 2, 3, 4].
#[inline]
fn rate_id(r: i32) -> i32 {
    (((r >> 12) - i32::from(r > 16000)) >> i32::from(r > 24000)) - 1
}

const USE_SILK_RESAMPLER_COPY: i32 = 0;
const USE_SILK_RESAMPLER_PRIVATE_UP2_HQ_WRAPPER: i32 = 1;
const USE_SILK_RESAMPLER_PRIVATE_IIR_FIR: i32 = 2;
const USE_SILK_RESAMPLER_PRIVATE_DOWN_FIR: i32 = 3;

/// `silk_resampler_init` — initialise/reset resampler state for a given
/// pair of input/output sampling rates.
pub fn silk_resampler_init(s: &mut SilkResamplerStateStruct, fs_hz_in: i32, fs_hz_out: i32, for_enc: i32) -> i32 {
    /* Clear state */
    *s = SilkResamplerStateStruct {
        s_iir: [0; SILK_RESAMPLER_MAX_IIR_ORDER],
        s_fir: [0; SILK_RESAMPLER_MAX_FIR_ORDER],
        delay_buf: [0; 48],
        resampler_function: 0,
        batch_size: 0,
        inv_ratio_q16: 0,
        fir_order: 0,
        fir_fracs: 0,
        fs_in_khz: 0,
        fs_out_khz: 0,
        input_delay: 0,
        coefs: None,
    };

    /* Input checking */
    if for_enc != 0 {
        if (fs_hz_in != 8000 && fs_hz_in != 12000 && fs_hz_in != 16000 && fs_hz_in != 24000 && fs_hz_in != 48000)
            || (fs_hz_out != 8000 && fs_hz_out != 12000 && fs_hz_out != 16000)
        {
            return -1;
        }
        s.input_delay = DELAY_MATRIX_ENC[rate_id(fs_hz_in) as usize][rate_id(fs_hz_out) as usize] as i32;
    } else {
        if (fs_hz_in != 8000 && fs_hz_in != 12000 && fs_hz_in != 16000)
            || (fs_hz_out != 8000 && fs_hz_out != 12000 && fs_hz_out != 16000 && fs_hz_out != 24000 && fs_hz_out != 48000)
        {
            return -1;
        }
        s.input_delay = DELAY_MATRIX_DEC[rate_id(fs_hz_in) as usize][rate_id(fs_hz_out) as usize] as i32;
    }

    s.fs_in_khz = fs_hz_in / 1000;
    s.fs_out_khz = fs_hz_out / 1000;

    /* Number of samples processed per batch */
    s.batch_size = s.fs_in_khz * RESAMPLER_MAX_BATCH_SIZE_MS;

    /* Find resampler with the right sampling ratio */
    let mut up2x = 0i32;
    if fs_hz_out > fs_hz_in {
        /* Upsample */
        if fs_hz_out == fs_hz_in * 2 {
            /* Special case: directly use 2x upsampler */
            s.resampler_function = USE_SILK_RESAMPLER_PRIVATE_UP2_HQ_WRAPPER;
        } else {
            /* Default resampler */
            s.resampler_function = USE_SILK_RESAMPLER_PRIVATE_IIR_FIR;
            up2x = 1;
        }
    } else if fs_hz_out < fs_hz_in {
        /* Downsample */
        s.resampler_function = USE_SILK_RESAMPLER_PRIVATE_DOWN_FIR;
        if fs_hz_out * 4 == fs_hz_in * 3 {
            s.fir_fracs = 3;
            s.fir_order = RESAMPLER_DOWN_ORDER_FIR0 as i32;
            s.coefs = Some(&SILK_RESAMPLER_3_4_COEFS);
        } else if fs_hz_out * 3 == fs_hz_in * 2 {
            s.fir_fracs = 2;
            s.fir_order = RESAMPLER_DOWN_ORDER_FIR0 as i32;
            s.coefs = Some(&SILK_RESAMPLER_2_3_COEFS);
        } else if fs_hz_out * 2 == fs_hz_in {
            s.fir_fracs = 1;
            s.fir_order = RESAMPLER_DOWN_ORDER_FIR1 as i32;
            s.coefs = Some(&SILK_RESAMPLER_1_2_COEFS);
        } else if fs_hz_out * 3 == fs_hz_in {
            s.fir_fracs = 1;
            s.fir_order = RESAMPLER_DOWN_ORDER_FIR2 as i32;
            s.coefs = Some(&SILK_RESAMPLER_1_3_COEFS);
        } else if fs_hz_out * 4 == fs_hz_in {
            s.fir_fracs = 1;
            s.fir_order = RESAMPLER_DOWN_ORDER_FIR2 as i32;
            s.coefs = Some(&SILK_RESAMPLER_1_4_COEFS);
        } else if fs_hz_out * 6 == fs_hz_in {
            s.fir_fracs = 1;
            s.fir_order = RESAMPLER_DOWN_ORDER_FIR2 as i32;
            s.coefs = Some(&SILK_RESAMPLER_1_6_COEFS);
        } else {
            /* None available */
            return -1;
        }
    } else {
        /* Input and output sampling rates are equal: copy */
        s.resampler_function = USE_SILK_RESAMPLER_COPY;
    }

    /* Ratio of input/output samples */
    s.inv_ratio_q16 = silk_lshift(silk_lshift(fs_hz_in, 14 + up2x) / fs_hz_out, 2);
    /* Make sure the ratio is rounded up */
    while silk_smulww(s.inv_ratio_q16, fs_hz_out) < silk_lshift(fs_hz_in, up2x) {
        s.inv_ratio_q16 += 1;
    }

    0
}

/// `silk_resampler` — convert from one sampling rate to another.
/// Input and output sampling rates are at most 48000 Hz.
pub fn silk_resampler(s: &mut SilkResamplerStateStruct, out: &mut [i16], in_: &[i16], in_len: i32) -> i32 {
    let n_samples = (s.fs_in_khz - s.input_delay) as usize;
    let input_delay = s.input_delay as usize;
    let fs_in_khz = s.fs_in_khz;
    let fs_out_khz = s.fs_out_khz;

    /* Copy to delay buffer */
    s.delay_buf[input_delay..input_delay + n_samples].copy_from_slice(&in_[..n_samples]);

    /* Snapshot the delay buffer ([i16; 48], a cheap copy): the C passes a
     * pointer into the state struct it also hands to the sub-resamplers,
     * which never touch delay_buf — but the borrow checker can't see that
     * through &mut SilkResamplerStateStruct. */
    let delay_buf = s.delay_buf;

    let out_tail = fs_out_khz as usize;
    let in_tail = n_samples;
    let tail_len = in_len - fs_in_khz;

    match s.resampler_function {
        USE_SILK_RESAMPLER_PRIVATE_UP2_HQ_WRAPPER => {
            silk_resampler_private_up2_hq_wrapper(s, out, &delay_buf, fs_in_khz);
            silk_resampler_private_up2_hq_wrapper(s, &mut out[out_tail..], &in_[in_tail..], tail_len);
        }
        USE_SILK_RESAMPLER_PRIVATE_IIR_FIR => {
            silk_resampler_private_iir_fir(s, out, &delay_buf, fs_in_khz);
            silk_resampler_private_iir_fir(s, &mut out[out_tail..], &in_[in_tail..], tail_len);
        }
        USE_SILK_RESAMPLER_PRIVATE_DOWN_FIR => {
            silk_resampler_private_down_fir(s, out, &delay_buf, fs_in_khz);
            silk_resampler_private_down_fir(s, &mut out[out_tail..], &in_[in_tail..], tail_len);
        }
        _ => {
            out[..fs_in_khz as usize].copy_from_slice(&delay_buf[..fs_in_khz as usize]);
            out[out_tail..out_tail + tail_len as usize].copy_from_slice(&in_[in_tail..in_tail + tail_len as usize]);
        }
    }

    /* Copy to delay buffer */
    s.delay_buf[..input_delay].copy_from_slice(&in_[in_len as usize - input_delay..in_len as usize]);

    0
}
