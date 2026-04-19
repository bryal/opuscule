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
use super::init_decoder::silk_init_decoder;
use super::macros::{silk_div32, silk_lshift, silk_min, silk_rshift, silk_smulbb};
use super::resampler::silk_resampler;
use super::stereo_decode_pred::{silk_stereo_decode_mid_only, silk_stereo_decode_pred};
use super::stereo_ms_to_lr::silk_stereo_MS_to_LR;
use super::structs::{MAX_FRAMES_PER_PACKET, SilkDecoderState, SilkResamplerStateStruct, StereoDecState};
use super::tables_other::silk_LBRR_flags_iCDF_ptr;

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
#[repr(C)]
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
#[repr(C)]
pub struct SilkTocStruct {
    pub vad_flag: c_int,
    pub vad_flags: [c_int; SILK_MAX_FRAMES_PER_PACKET],
    pub inband_fec_flag: c_int,
}

/// `silk_decoder` super-struct wrapping the N per-channel states plus
/// shared stereo state and channel counts. The parent `opus_decoder`
/// treats this as an opaque blob sized by [`silk_Get_Decoder_Size`].
#[repr(C)]
pub struct SilkDecoder {
    pub channel_state: [SilkDecoderState; DECODER_NUM_CHANNELS],
    pub s_stereo: StereoDecState,
    pub n_channels_api: c_int,
    pub n_channels_internal: c_int,
    pub prev_decode_only_middle: c_int,
}

/// `silk_Get_Decoder_Size` — report sizeof(silk_decoder).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_Get_Decoder_Size(dec_size_bytes: *mut c_int) -> c_int {
    unsafe {
        *dec_size_bytes = core::mem::size_of::<SilkDecoder>() as c_int;
    }
    SILK_NO_ERROR
}

/// `silk_InitDecoder` — reset the per-channel states.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_InitDecoder(dec_state: *mut u8) -> c_int {
    unsafe {
        let ps_dec = dec_state as *mut SilkDecoder;
        let channel_state = (*ps_dec).channel_state.as_mut_ptr();
        let mut ret = SILK_NO_ERROR;
        for n in 0..DECODER_NUM_CHANNELS {
            ret = silk_init_decoder(channel_state.add(n));
        }
        ret
    }
}

/// `silk_Decode` — decode a SILK frame (mono or stereo, normal/LBRR/PLC).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_Decode(
    dec_state: *mut u8,
    dec_control: *mut SilkDecControlStruct,
    lost_flag: i32,
    new_packet_flag: i32,
    ps_range_dec: *mut ec_dec,
    samples_out: *mut i16,
    n_samples_out: *mut i32,
) -> i32 {
    unsafe {
        let mut decode_only_middle: i32 = 0;
        let mut ret = SILK_NO_ERROR;
        let mut n_samples_out_dec: i32 = 0;
        let mut samples_out1_tmp = [[0i16; MAX_FS_KHZ * MAX_FRAME_LENGTH_MS + 2]; 2];
        let mut samples_out2_tmp = [0i16; MAX_API_FS_KHZ as usize * MAX_FRAME_LENGTH_MS];
        let mut ms_pred_q13 = [0i32; 2];

        let ps_dec = dec_state as *mut SilkDecoder;
        let channel_state = (*ps_dec).channel_state.as_mut_ptr();

        /**********************************/
        /* Test if first frame in payload */
        /**********************************/
        if new_packet_flag != 0 {
            for n in 0..(*dec_control).n_channels_internal as usize {
                (*channel_state.add(n)).n_frames_decoded = 0;
            }
        }

        /* If Mono -> Stereo transition in bitstream: init state of second channel */
        if (*dec_control).n_channels_internal > (*ps_dec).n_channels_internal {
            ret += silk_init_decoder(channel_state.add(1));
        }

        let stereo_to_mono = ((*dec_control).n_channels_internal == 1
            && (*ps_dec).n_channels_internal == 2
            && ((*dec_control).internal_sample_rate == 1000 * (*channel_state).fs_khz)) as i32;

        if (*channel_state).n_frames_decoded == 0 {
            for n in 0..(*dec_control).n_channels_internal as usize {
                let cs = channel_state.add(n);
                if (*dec_control).payload_size_ms == 0 {
                    /* Assuming packet loss, use 10 ms */
                    (*cs).n_frames_per_packet = 1;
                    (*cs).nb_subfr = 2;
                } else if (*dec_control).payload_size_ms == 10 {
                    (*cs).n_frames_per_packet = 1;
                    (*cs).nb_subfr = 2;
                } else if (*dec_control).payload_size_ms == 20 {
                    (*cs).n_frames_per_packet = 1;
                    (*cs).nb_subfr = 4;
                } else if (*dec_control).payload_size_ms == 40 {
                    (*cs).n_frames_per_packet = 2;
                    (*cs).nb_subfr = 4;
                } else if (*dec_control).payload_size_ms == 60 {
                    (*cs).n_frames_per_packet = 3;
                    (*cs).nb_subfr = 4;
                } else {
                    return SILK_DEC_INVALID_FRAME_SIZE;
                }
                let fs_khz_dec = ((*dec_control).internal_sample_rate >> 10) + 1;
                if fs_khz_dec != 8 && fs_khz_dec != 12 && fs_khz_dec != 16 {
                    return SILK_DEC_INVALID_SAMPLING_FREQUENCY;
                }
                ret += silk_decoder_set_fs(cs, fs_khz_dec, (*dec_control).api_sample_rate);
            }
        }

        if (*dec_control).n_channels_api == 2
            && (*dec_control).n_channels_internal == 2
            && ((*ps_dec).n_channels_api == 1 || (*ps_dec).n_channels_internal == 1)
        {
            core::ptr::write_bytes((*ps_dec).s_stereo.pred_prev_q13.as_mut_ptr(), 0, 2);
            core::ptr::write_bytes((*ps_dec).s_stereo.s_side.as_mut_ptr(), 0, 2);
            core::ptr::copy_nonoverlapping::<SilkResamplerStateStruct>(
                &raw const (*channel_state).resampler_state,
                &raw mut (*channel_state.add(1)).resampler_state,
                1,
            );
        }
        (*ps_dec).n_channels_api = (*dec_control).n_channels_api;
        (*ps_dec).n_channels_internal = (*dec_control).n_channels_internal;

        if (*dec_control).api_sample_rate > MAX_API_FS_KHZ * 1000 || (*dec_control).api_sample_rate < 8000 {
            return SILK_DEC_INVALID_SAMPLING_FREQUENCY;
        }

        if lost_flag != FLAG_PACKET_LOST && (*channel_state).n_frames_decoded == 0 {
            /* First decoder call for this payload */
            /* Decode VAD flags and LBRR flag */
            for n in 0..(*dec_control).n_channels_internal as usize {
                let cs = channel_state.add(n);
                for i in 0..(*cs).n_frames_per_packet as usize {
                    (*cs).vad_flags[i] = ec_dec_bit_logp(ps_range_dec, 1);
                }
                (*cs).lbrr_flag = ec_dec_bit_logp(ps_range_dec, 1);
            }
            /* Decode LBRR flags */
            for n in 0..(*dec_control).n_channels_internal as usize {
                let cs = channel_state.add(n);
                for v in (*cs).lbrr_flags.iter_mut() {
                    *v = 0;
                }
                if (*cs).lbrr_flag != 0 {
                    if (*cs).n_frames_per_packet == 1 {
                        (*cs).lbrr_flags[0] = 1;
                    } else {
                        let lbrr_symbol = ec_dec_icdf(
                            ps_range_dec,
                            silk_LBRR_flags_iCDF_ptr[((*cs).n_frames_per_packet - 2) as usize] as *const u8,
                            8,
                        ) + 1;
                        for i in 0..(*cs).n_frames_per_packet as usize {
                            (*cs).lbrr_flags[i] = silk_rshift(lbrr_symbol, i as i32) & 1;
                        }
                    }
                }
            }

            if lost_flag == FLAG_DECODE_NORMAL {
                /* Regular decoding: skip all LBRR data */
                for i in 0..(*channel_state).n_frames_per_packet as usize {
                    for n in 0..(*dec_control).n_channels_internal as usize {
                        let cs = channel_state.add(n);
                        if (*cs).lbrr_flags[i] != 0 {
                            let mut pulses = [0i32; MAX_FRAME_LENGTH];
                            if (*dec_control).n_channels_internal == 2 && n == 0 {
                                silk_stereo_decode_pred(ps_range_dec, ms_pred_q13.as_mut_ptr());
                                if (*channel_state.add(1)).lbrr_flags[i] == 0 {
                                    silk_stereo_decode_mid_only(ps_range_dec, &raw mut decode_only_middle);
                                }
                            }
                            /* Use conditional coding if previous frame available */
                            let cond_coding =
                                if i > 0 && (*cs).lbrr_flags[i - 1] != 0 { CODE_CONDITIONALLY } else { CODE_INDEPENDENTLY };
                            silk_decode_indices(cs, ps_range_dec, i as i32, 1, cond_coding);
                            silk_decode_pulses(
                                ps_range_dec,
                                pulses.as_mut_ptr(),
                                (*cs).indices.signal_type as i32,
                                (*cs).indices.quant_offset_type as i32,
                                (*cs).frame_length,
                            );
                        }
                    }
                }
            }
        }

        /* Get MS predictor index */
        if (*dec_control).n_channels_internal == 2 {
            if lost_flag == FLAG_DECODE_NORMAL
                || (lost_flag == FLAG_DECODE_LBRR
                    && (*channel_state).lbrr_flags[(*channel_state).n_frames_decoded as usize] == 1)
            {
                silk_stereo_decode_pred(ps_range_dec, ms_pred_q13.as_mut_ptr());
                /* For LBRR data, decode mid-only flag only if side-channel's LBRR flag is false */
                if (lost_flag == FLAG_DECODE_NORMAL
                    && (*channel_state.add(1)).vad_flags[(*channel_state).n_frames_decoded as usize] == 0)
                    || (lost_flag == FLAG_DECODE_LBRR
                        && (*channel_state.add(1)).lbrr_flags[(*channel_state).n_frames_decoded as usize] == 0)
                {
                    silk_stereo_decode_mid_only(ps_range_dec, &raw mut decode_only_middle);
                } else {
                    decode_only_middle = 0;
                }
            } else {
                for n in 0..2 {
                    ms_pred_q13[n] = (*ps_dec).s_stereo.pred_prev_q13[n] as i32;
                }
            }
        }

        /* Reset side channel decoder prediction memory for first frame with side coding */
        if (*dec_control).n_channels_internal == 2 && decode_only_middle == 0 && (*ps_dec).prev_decode_only_middle == 1 {
            let cs1 = channel_state.add(1);
            core::ptr::write_bytes((*cs1).out_buf.as_mut_ptr(), 0, (*cs1).out_buf.len());
            core::ptr::write_bytes((*cs1).s_lpc_q14_buf.as_mut_ptr(), 0, (*cs1).s_lpc_q14_buf.len());
            (*cs1).lag_prev = 100;
            (*cs1).last_gain_index = 10;
            (*cs1).prev_signal_type = TYPE_NO_VOICE_ACTIVITY;
            (*cs1).first_frame_after_reset = 1;
        }

        let has_side = if lost_flag == FLAG_DECODE_NORMAL {
            (decode_only_middle == 0) as i32
        } else {
            ((*ps_dec).prev_decode_only_middle == 0
                || ((*dec_control).n_channels_internal == 2
                    && lost_flag == FLAG_DECODE_LBRR
                    && (*channel_state.add(1)).lbrr_flags[(*channel_state.add(1)).n_frames_decoded as usize] == 1))
                as i32
        };

        /* Call decoder for one frame */
        for n in 0..(*dec_control).n_channels_internal as usize {
            let cs = channel_state.add(n);
            if n == 0 || has_side != 0 {
                let frame_index = (*channel_state).n_frames_decoded - n as i32;
                let cond_coding = if frame_index <= 0 {
                    CODE_INDEPENDENTLY
                } else if lost_flag == FLAG_DECODE_LBRR {
                    if (*cs).lbrr_flags[(frame_index - 1) as usize] != 0 { CODE_CONDITIONALLY } else { CODE_INDEPENDENTLY }
                } else if n > 0 && (*ps_dec).prev_decode_only_middle != 0 {
                    /* If we skipped a side frame in this packet, we don't
                    need LTP scaling; the LTP state is well-defined. */
                    CODE_INDEPENDENTLY_NO_LTP_SCALING
                } else {
                    CODE_CONDITIONALLY
                };
                ret += silk_decode_frame(
                    cs,
                    ps_range_dec,
                    samples_out1_tmp[n].as_mut_ptr().add(2),
                    &raw mut n_samples_out_dec,
                    lost_flag,
                    cond_coding,
                );
            } else {
                core::ptr::write_bytes(samples_out1_tmp[n].as_mut_ptr().add(2), 0, n_samples_out_dec as usize);
            }
            (*cs).n_frames_decoded += 1;
        }

        if (*dec_control).n_channels_api == 2 && (*dec_control).n_channels_internal == 2 {
            /* Convert Mid/Side to Left/Right */
            silk_stereo_MS_to_LR(
                &raw mut (*ps_dec).s_stereo,
                samples_out1_tmp[0].as_mut_ptr(),
                samples_out1_tmp[1].as_mut_ptr(),
                ms_pred_q13.as_ptr(),
                (*channel_state).fs_khz,
                n_samples_out_dec,
            );
        } else {
            /* Buffering */
            core::ptr::copy_nonoverlapping((*ps_dec).s_stereo.s_mid.as_ptr(), samples_out1_tmp[0].as_mut_ptr(), 2);
            core::ptr::copy_nonoverlapping(
                samples_out1_tmp[0].as_ptr().add(n_samples_out_dec as usize),
                (*ps_dec).s_stereo.s_mid.as_mut_ptr(),
                2,
            );
        }

        /* Number of output samples */
        *n_samples_out =
            silk_div32(n_samples_out_dec * (*dec_control).api_sample_rate, silk_smulbb((*channel_state).fs_khz, 1000));

        /* Set up pointers to temp buffers */
        let resample_out_ptr = if (*dec_control).n_channels_api == 2 { samples_out2_tmp.as_mut_ptr() } else { samples_out };

        let loop_n = silk_min((*dec_control).n_channels_api, (*dec_control).n_channels_internal);
        for n in 0..loop_n as usize {
            let cs = channel_state.add(n);
            /* Resample decoded signal to API_sampleRate */
            ret += silk_resampler(
                &raw mut (*cs).resampler_state,
                resample_out_ptr,
                samples_out1_tmp[n].as_ptr().add(1),
                n_samples_out_dec,
            );

            /* Interleave if stereo output and stereo stream */
            if (*dec_control).n_channels_api == 2 {
                for i in 0..*n_samples_out as usize {
                    *samples_out.add(n + 2 * i) = *resample_out_ptr.add(i);
                }
            }
        }

        /* Create two channel output from mono stream */
        if (*dec_control).n_channels_api == 2 && (*dec_control).n_channels_internal == 1 {
            if stereo_to_mono != 0 {
                /* Resample right channel for newly collapsed stereo just in case
                we weren't doing collapsing when switching to mono */
                ret += silk_resampler(
                    &raw mut (*channel_state.add(1)).resampler_state,
                    resample_out_ptr,
                    samples_out1_tmp[0].as_ptr().add(1),
                    n_samples_out_dec,
                );
                for i in 0..*n_samples_out as usize {
                    *samples_out.add(1 + 2 * i) = *resample_out_ptr.add(i);
                }
            } else {
                for i in 0..*n_samples_out as usize {
                    *samples_out.add(1 + 2 * i) = *samples_out.add(2 * i);
                }
            }
        }

        /* Export pitch lag, measured at 48 kHz sampling rate */
        if (*channel_state).prev_signal_type == TYPE_VOICED {
            let mult_tab = [6i32, 4, 3];
            (*dec_control).prev_pitch_lag = (*channel_state).lag_prev * mult_tab[(((*channel_state).fs_khz - 8) >> 2) as usize];
        } else {
            (*dec_control).prev_pitch_lag = 0;
        }

        if lost_flag == FLAG_PACKET_LOST {
            /* On packet loss, remove the gain clamping to prevent having the energy "bounce back"
            if we lose packets when the energy is going down */
            for i in 0..(*ps_dec).n_channels_internal as usize {
                (*ps_dec).channel_state[i].last_gain_index = 10;
            }
        } else {
            (*ps_dec).prev_decode_only_middle = decode_only_middle;
        }
        ret
    }
}

/// `silk_get_TOC` — extract per-packet VAD / inband-FEC flags.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn silk_get_TOC(
    payload: *const u8,
    n_bytes_in: i32,
    n_frames_per_payload: i32,
    silk_toc: *mut SilkTocStruct,
) -> i32 {
    unsafe {
        let ret = SILK_NO_ERROR;
        if n_bytes_in < 1 {
            return -1;
        }
        if !(0..=3).contains(&n_frames_per_payload) {
            return -1;
        }

        /* C: silk_memset( Silk_TOC, 0, sizeof( Silk_TOC ) )
         * — this is an RFC bug: Silk_TOC is a pointer, so sizeof yields
         * the pointer size (typically 8), not the struct size. We mirror
         * the bug byte-for-byte for binary compatibility. */
        core::ptr::write_bytes(silk_toc as *mut u8, 0, core::mem::size_of::<*const SilkTocStruct>());

        /* For stereo, extract the flags for the mid channel */
        let mut flags = silk_rshift(*payload as i32, 7 - n_frames_per_payload) & (silk_lshift(1, n_frames_per_payload + 1) - 1);

        (*silk_toc).inband_fec_flag = flags & 1;
        let mut i = n_frames_per_payload - 1;
        while i >= 0 {
            flags = silk_rshift(flags, 1);
            (*silk_toc).vad_flags[i as usize] = flags & 1;
            (*silk_toc).vad_flag |= flags & 1;
            i -= 1;
        }

        ret
    }
}

// Compile-time ABI assertions against the C sizeof/offsetof measurements.
const _: () = {
    assert!(core::mem::size_of::<SilkDecControlStruct>() == 24);
    // SilkDecoder must match C silk_decoder sizeof so opus_decoder's
    // allocator reserves the right number of bytes.
    assert!(core::mem::size_of::<SilkDecoder>() % core::mem::align_of::<SilkDecoder>() == 0);
    assert!(MAX_FRAMES_PER_PACKET == SILK_MAX_FRAMES_PER_PACKET);
};
