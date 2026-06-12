//! Translated from `c/silk/dec_API.c` (RFC 6716).
//!
//! Top-level SILK decoder entry points:
//! - [`silk_Get_Decoder_Size`] — report sizeof(silk_decoder) to callers
//! - [`silk_InitDecoder`] — reset all channel states
//! - [`silk_Decode`] — decode one frame (possibly stereo, possibly lost)
//! - [`silk_get_TOC`] — extract per-packet VAD/FEC flags

use core::ffi::c_int;

use crate::entcode::ec_dec;
use crate::entdec::{ec_dec_bit_logp, ec_dec_icdf};

use super::decode_frame::silk_decode_frame;
use super::decode_indices::silk_decode_indices;
use super::decode_pulses::silk_decode_pulses;
use super::decoder_set_fs::silk_decoder_set_fs;
use super::macros::{silk_lshift, silk_smulbb};
use super::resampler::silk_resampler;
use super::stereo_decode_pred::{silk_stereo_decode_mid_only, silk_stereo_decode_pred};
use super::stereo_ms_to_lr::silk_stereo_ms_to_lr;
use super::structs::{MAX_FRAMES_PER_PACKET, SilkDecoderState, SilkResamplerStateStruct, StereoDecState};
use super::tables_other::SILK_LBRR_FLAGS_ICDF_PTR;

const DECODER_NUM_CHANNELS: usize = 2;
const MAX_API_FS_KHZ: i32 = 48;
const MAX_FS_KHZ: usize = 16;
const SUB_FRAME_LENGTH_MS: usize = 5;
const MAX_NB_SUBFR: usize = 4;
const MAX_FRAME_LENGTH_MS: usize = SUB_FRAME_LENGTH_MS * MAX_NB_SUBFR;
const MAX_FRAME_LENGTH: usize = MAX_FRAME_LENGTH_MS * MAX_FS_KHZ;

const SILK_NO_ERROR: i32 = 0;
const SILK_DEC_INVALID_SAMPLING_FREQUENCY: i32 = -200;
const SILK_DEC_INVALID_FRAME_SIZE: i32 = -203;

const FLAG_PACKET_LOST: i32 = 1;
const FLAG_DECODE_NORMAL: i32 = 0;
const FLAG_DECODE_LBRR: i32 = 2;

const CODE_INDEPENDENTLY: i32 = 0;
const CODE_INDEPENDENTLY_NO_LTP_SCALING: i32 = 1;
const CODE_CONDITIONALLY: i32 = 2;

const TYPE_NO_VOICE_ACTIVITY: i32 = 0;
const TYPE_VOICED: i32 = 2;

const SILK_MAX_FRAMES_PER_PACKET: usize = 3;

/// `silk_DecControlStruct` — decoder control / status block
/// (`c/silk/control.h:119`).
pub struct SilkDecControlStruct {
    pub n_channels_api: i32,
    pub n_channels_internal: i32,
    pub api_sample_rate: i32,
    pub internal_sample_rate: i32,
    pub payload_size_ms: c_int,
    pub prev_pitch_lag: c_int,
}

/// `silk_TOC_struct` — table of contents for one Opus packet
/// (`c/silk/API.h:49`).
pub struct SilkTocStruct {
    pub vad_flag: c_int,
    pub vad_flags: [c_int; SILK_MAX_FRAMES_PER_PACKET],
    pub inband_fec_flag: c_int,
}

/// `silk_decoder` super-struct wrapping the N per-channel states plus
/// shared stereo state and channel counts. The parent `opus_decoder`
/// treats this as an opaque blob sized by [`silk_Get_Decoder_Size`].
pub struct SilkDecoder {
    pub channel_state: [SilkDecoderState; DECODER_NUM_CHANNELS],
    pub s_stereo: StereoDecState,
    pub n_channels_api: c_int,
    pub n_channels_internal: c_int,
    pub prev_decode_only_middle: c_int,
}

/// `silk_Get_Decoder_Size` — report sizeof(silk_decoder).
pub fn silk_get_decoder_size(dec_size_bytes: &mut c_int) -> c_int {
    *dec_size_bytes = core::mem::size_of::<SilkDecoder>() as c_int;
    SILK_NO_ERROR
}

/// `silk_InitDecoder` — reset the per-channel states.
pub fn silk_init_decoder(ps_dec: &mut SilkDecoder) -> c_int {
    let mut ret = SILK_NO_ERROR;
    for channel in ps_dec.channel_state.iter_mut().take(DECODER_NUM_CHANNELS) {
        ret = super::init_decoder::silk_init_decoder(channel);
    }
    ret
}

/// `silk_Decode` — decode a SILK frame (mono or stereo, normal/LBRR/PLC).
pub fn silk_decode(
    ps_dec: &mut SilkDecoder,
    dec_control: &mut SilkDecControlStruct,
    lost_flag: i32,
    new_packet_flag: i32,
    ps_range_dec: &mut ec_dec,
    samples_out: &mut [i16],
    n_samples_out: &mut i32,
) -> i32 {
    let mut decode_only_middle: i32 = 0;
    let mut ret = SILK_NO_ERROR;
    let mut n_samples_out_dec: i32 = 0;
    let mut samples_out1_tmp = [[0i16; MAX_FS_KHZ * MAX_FRAME_LENGTH_MS + 2]; 2];
    let mut samples_out2_tmp = [0i16; MAX_API_FS_KHZ as usize * MAX_FRAME_LENGTH_MS];
    let mut ms_pred_q13 = [0i32; 2];

    /**********************************/
    /* Test if first frame in payload */
    /**********************************/
    if new_packet_flag != 0 {
        for n in 0..dec_control.n_channels_internal as usize {
            ps_dec.channel_state[n].n_frames_decoded = 0;
        }
    }

    /* If Mono -> Stereo transition in bitstream: init state of second channel */
    if dec_control.n_channels_internal > ps_dec.n_channels_internal {
        ret += super::init_decoder::silk_init_decoder(&mut ps_dec.channel_state[1]);
    }

    let stereo_to_mono = (dec_control.n_channels_internal == 1
        && ps_dec.n_channels_internal == 2
        && (dec_control.internal_sample_rate == 1000 * ps_dec.channel_state[0].fs_khz)) as i32;

    if ps_dec.channel_state[0].n_frames_decoded == 0 {
        for n in 0..dec_control.n_channels_internal as usize {
            let cs = &mut ps_dec.channel_state[n];
            if dec_control.payload_size_ms == 0 {
                /* Assuming packet loss, use 10 ms */
                cs.n_frames_per_packet = 1;
                cs.nb_subfr = 2;
            } else if dec_control.payload_size_ms == 10 {
                cs.n_frames_per_packet = 1;
                cs.nb_subfr = 2;
            } else if dec_control.payload_size_ms == 20 {
                cs.n_frames_per_packet = 1;
                cs.nb_subfr = 4;
            } else if dec_control.payload_size_ms == 40 {
                cs.n_frames_per_packet = 2;
                cs.nb_subfr = 4;
            } else if dec_control.payload_size_ms == 60 {
                cs.n_frames_per_packet = 3;
                cs.nb_subfr = 4;
            } else {
                return SILK_DEC_INVALID_FRAME_SIZE;
            }
            let fs_khz_dec = (dec_control.internal_sample_rate >> 10) + 1;
            if fs_khz_dec != 8 && fs_khz_dec != 12 && fs_khz_dec != 16 {
                return SILK_DEC_INVALID_SAMPLING_FREQUENCY;
            }
            ret += silk_decoder_set_fs(cs, fs_khz_dec, dec_control.api_sample_rate);
        }
    }

    if dec_control.n_channels_api == 2
        && dec_control.n_channels_internal == 2
        && (ps_dec.n_channels_api == 1 || ps_dec.n_channels_internal == 1)
    {
        ps_dec.s_stereo.pred_prev_q13.fill(0);
        ps_dec.s_stereo.s_side.fill(0);
        /* C: whole-struct memcpy of the channel-0 resampler state */
        let rs = ps_dec.channel_state[0].resampler_state.clone();
        ps_dec.channel_state[1].resampler_state = rs;
    }
    ps_dec.n_channels_api = dec_control.n_channels_api;
    ps_dec.n_channels_internal = dec_control.n_channels_internal;

    if dec_control.api_sample_rate > MAX_API_FS_KHZ * 1000 || dec_control.api_sample_rate < 8000 {
        return SILK_DEC_INVALID_SAMPLING_FREQUENCY;
    }

    if lost_flag != FLAG_PACKET_LOST && ps_dec.channel_state[0].n_frames_decoded == 0 {
        /* First decoder call for this payload */
        /* Decode VAD flags and LBRR flag */
        for n in 0..dec_control.n_channels_internal as usize {
            let cs = &mut ps_dec.channel_state[n];
            for i in 0..cs.n_frames_per_packet as usize {
                cs.vad_flags[i] = ec_dec_bit_logp(ps_range_dec, 1);
            }
            cs.lbrr_flag = ec_dec_bit_logp(ps_range_dec, 1);
        }
        /* Decode LBRR flags */
        for n in 0..dec_control.n_channels_internal as usize {
            let cs = &mut ps_dec.channel_state[n];
            for v in cs.lbrr_flags.iter_mut() {
                *v = 0;
            }
            if cs.lbrr_flag != 0 {
                if cs.n_frames_per_packet == 1 {
                    cs.lbrr_flags[0] = 1;
                } else {
                    let lbrr_symbol =
                        ec_dec_icdf(ps_range_dec, SILK_LBRR_FLAGS_ICDF_PTR[(cs.n_frames_per_packet - 2) as usize], 8) + 1;
                    for i in 0..cs.n_frames_per_packet as usize {
                        cs.lbrr_flags[i] = (lbrr_symbol >> i as i32) & 1;
                    }
                }
            }
        }

        if lost_flag == FLAG_DECODE_NORMAL {
            /* Regular decoding: skip all LBRR data */
            let n_frames_per_packet0 = ps_dec.channel_state[0].n_frames_per_packet;
            for i in 0..n_frames_per_packet0 as usize {
                for n in 0..dec_control.n_channels_internal as usize {
                    if ps_dec.channel_state[n].lbrr_flags[i] != 0 {
                        let mut pulses = [0i32; MAX_FRAME_LENGTH];
                        if dec_control.n_channels_internal == 2 && n == 0 {
                            silk_stereo_decode_pred(ps_range_dec, &mut ms_pred_q13);
                            if ps_dec.channel_state[1].lbrr_flags[i] == 0 {
                                silk_stereo_decode_mid_only(ps_range_dec, &mut decode_only_middle);
                            }
                        }
                        /* Use conditional coding if previous frame available */
                        let cond_coding = if i > 0 && ps_dec.channel_state[n].lbrr_flags[i - 1] != 0 {
                            CODE_CONDITIONALLY
                        } else {
                            CODE_INDEPENDENTLY
                        };
                        silk_decode_indices(&mut ps_dec.channel_state[n], ps_range_dec, i as i32, 1, cond_coding);
                        let cs = &ps_dec.channel_state[n];
                        let (signal_type, quant_offset_type, frame_length) =
                            (cs.indices.signal_type as i32, cs.indices.quant_offset_type as i32, cs.frame_length);
                        silk_decode_pulses(ps_range_dec, &mut pulses, signal_type, quant_offset_type, frame_length);
                    }
                }
            }
        }
    }

    /* Get MS predictor index */
    if dec_control.n_channels_internal == 2 {
        if lost_flag == FLAG_DECODE_NORMAL
            || (lost_flag == FLAG_DECODE_LBRR
                && ps_dec.channel_state[0].lbrr_flags[ps_dec.channel_state[0].n_frames_decoded as usize] == 1)
        {
            silk_stereo_decode_pred(ps_range_dec, &mut ms_pred_q13);
            /* For LBRR data, decode mid-only flag only if side-channel's LBRR flag is false */
            if (lost_flag == FLAG_DECODE_NORMAL
                && ps_dec.channel_state[1].vad_flags[ps_dec.channel_state[0].n_frames_decoded as usize] == 0)
                || (lost_flag == FLAG_DECODE_LBRR
                    && ps_dec.channel_state[1].lbrr_flags[ps_dec.channel_state[0].n_frames_decoded as usize] == 0)
            {
                silk_stereo_decode_mid_only(ps_range_dec, &mut decode_only_middle);
            } else {
                decode_only_middle = 0;
            }
        } else {
            for n in 0..2 {
                ms_pred_q13[n] = ps_dec.s_stereo.pred_prev_q13[n] as i32;
            }
        }
    }

    /* Reset side channel decoder prediction memory for first frame with side coding */
    if dec_control.n_channels_internal == 2 && decode_only_middle == 0 && ps_dec.prev_decode_only_middle == 1 {
        let cs1 = &mut ps_dec.channel_state[1];
        cs1.out_buf.fill(0);
        cs1.s_lpc_q14_buf.fill(0);
        cs1.lag_prev = 100;
        cs1.last_gain_index = 10;
        cs1.prev_signal_type = TYPE_NO_VOICE_ACTIVITY;
        cs1.first_frame_after_reset = 1;
    }

    let has_side = if lost_flag == FLAG_DECODE_NORMAL {
        (decode_only_middle == 0) as i32
    } else {
        (ps_dec.prev_decode_only_middle == 0
            || (dec_control.n_channels_internal == 2
                && lost_flag == FLAG_DECODE_LBRR
                && ps_dec.channel_state[1].lbrr_flags[ps_dec.channel_state[1].n_frames_decoded as usize] == 1)) as i32
    };

    /* Call decoder for one frame */
    for n in 0..dec_control.n_channels_internal as usize {
        if n == 0 || has_side != 0 {
            let frame_index = ps_dec.channel_state[0].n_frames_decoded - n as i32;
            let cond_coding = if frame_index <= 0 {
                CODE_INDEPENDENTLY
            } else if lost_flag == FLAG_DECODE_LBRR {
                if ps_dec.channel_state[n].lbrr_flags[(frame_index - 1) as usize] != 0 {
                    CODE_CONDITIONALLY
                } else {
                    CODE_INDEPENDENTLY
                }
            } else if n > 0 && ps_dec.prev_decode_only_middle != 0 {
                /* If we skipped a side frame in this packet, we don't
                need LTP scaling; the LTP state is well-defined. */
                CODE_INDEPENDENTLY_NO_LTP_SCALING
            } else {
                CODE_CONDITIONALLY
            };
            ret += silk_decode_frame(
                &mut ps_dec.channel_state[n],
                ps_range_dec,
                &mut samples_out1_tmp[n][2..],
                &mut n_samples_out_dec,
                lost_flag,
                cond_coding,
            );
        } else {
            samples_out1_tmp[n][2..2 + n_samples_out_dec as usize].fill(0);
        }
        ps_dec.channel_state[n].n_frames_decoded += 1;
    }

    if dec_control.n_channels_api == 2 && dec_control.n_channels_internal == 2 {
        /* Convert Mid/Side to Left/Right */
        let fs_khz = ps_dec.channel_state[0].fs_khz;
        let (ch_mid, ch_side) = samples_out1_tmp.split_at_mut(1);
        silk_stereo_ms_to_lr(&mut ps_dec.s_stereo, &mut ch_mid[0], &mut ch_side[0], &ms_pred_q13, fs_khz, n_samples_out_dec);
    } else {
        /* Buffering */
        samples_out1_tmp[0][..2].copy_from_slice(&ps_dec.s_stereo.s_mid);
        ps_dec.s_stereo.s_mid.copy_from_slice(&samples_out1_tmp[0][n_samples_out_dec as usize..n_samples_out_dec as usize + 2]);
    }

    /* Number of output samples */
    *n_samples_out = n_samples_out_dec * dec_control.api_sample_rate / silk_smulbb(ps_dec.channel_state[0].fs_khz, 1000);

    /* Resample decoded signal to API_sampleRate; for stereo API output the
     * resampler writes into samples_out2_tmp and gets interleaved, exactly
     * the C's resample_out_ptr selection. */
    let stereo_api = dec_control.n_channels_api == 2;

    let loop_n = dec_control.n_channels_api.min(dec_control.n_channels_internal);
    for n in 0..loop_n as usize {
        /* Resample decoded signal to API_sampleRate */
        if stereo_api {
            ret += silk_resampler(
                &mut ps_dec.channel_state[n].resampler_state,
                &mut samples_out2_tmp[..*n_samples_out as usize],
                &samples_out1_tmp[n][1..],
                n_samples_out_dec,
            );
            /* Interleave if stereo output and stereo stream */
            for i in 0..*n_samples_out as usize {
                samples_out[n + 2 * i] = samples_out2_tmp[i];
            }
        } else {
            ret += silk_resampler(
                &mut ps_dec.channel_state[n].resampler_state,
                &mut samples_out[..*n_samples_out as usize],
                &samples_out1_tmp[n][1..],
                n_samples_out_dec,
            );
        }
    }

    /* Create two channel output from mono stream */
    if dec_control.n_channels_api == 2 && dec_control.n_channels_internal == 1 {
        if stereo_to_mono != 0 {
            /* Resample right channel for newly collapsed stereo just in case
            we weren't doing collapsing when switching to mono */
            ret += silk_resampler(
                &mut ps_dec.channel_state[1].resampler_state,
                &mut samples_out2_tmp[..*n_samples_out as usize],
                &samples_out1_tmp[0][1..],
                n_samples_out_dec,
            );
            for i in 0..*n_samples_out as usize {
                samples_out[1 + 2 * i] = samples_out2_tmp[i];
            }
        } else {
            for i in 0..*n_samples_out as usize {
                samples_out[1 + 2 * i] = samples_out[2 * i];
            }
        }
    }

    /* Export pitch lag, measured at 48 kHz sampling rate */
    if ps_dec.channel_state[0].prev_signal_type == TYPE_VOICED {
        let mult_tab = [6i32, 4, 3];
        dec_control.prev_pitch_lag =
            ps_dec.channel_state[0].lag_prev * mult_tab[((ps_dec.channel_state[0].fs_khz - 8) >> 2) as usize];
    } else {
        dec_control.prev_pitch_lag = 0;
    }

    if lost_flag == FLAG_PACKET_LOST {
        /* On packet loss, remove the gain clamping to prevent having the energy "bounce back"
        if we lose packets when the energy is going down */
        for i in 0..ps_dec.n_channels_internal as usize {
            ps_dec.channel_state[i].last_gain_index = 10;
        }
    } else {
        ps_dec.prev_decode_only_middle = decode_only_middle;
    }
    ret
}

/// `silk_get_TOC` — extract per-packet VAD / inband-FEC flags.
pub fn silk_get_toc(payload: &[u8], n_bytes_in: i32, n_frames_per_payload: i32, silk_toc: &mut SilkTocStruct) -> i32 {
    let ret = SILK_NO_ERROR;
    if n_bytes_in < 1 {
        return -1;
    }
    if !(0..=3).contains(&n_frames_per_payload) {
        return -1;
    }

    /* C: silk_memset( Silk_TOC, 0, sizeof( Silk_TOC ) )
     * — this is an RFC bug: Silk_TOC is a pointer, so sizeof yields the
     * pointer size (typically 8), not the struct size. Only the first 8
     * bytes — vad_flag and vad_flags[0] in the C layout — get cleared;
     * the rest of the struct keeps whatever the caller had in it. We
     * mirror that field-for-field. */
    silk_toc.vad_flag = 0;
    silk_toc.vad_flags[0] = 0;

    /* For stereo, extract the flags for the mid channel */
    let mut flags = (payload[0] as i32 >> (7 - n_frames_per_payload)) & (silk_lshift(1, n_frames_per_payload + 1) - 1);

    silk_toc.inband_fec_flag = flags & 1;
    let mut i = n_frames_per_payload - 1;
    while i >= 0 {
        flags >>= 1;
        silk_toc.vad_flags[i as usize] = flags & 1;
        silk_toc.vad_flag |= flags & 1;
        i -= 1;
    }

    ret
}

const _: () = {
    assert!(MAX_FRAMES_PER_PACKET == SILK_MAX_FRAMES_PER_PACKET);
};
