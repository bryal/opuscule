// Translated from c/src/opus_demo.c (RFC 6716 reference).
//
// Demo/test application for the Opus codec. Only the decode path is
// implemented; encoding requests exit with an error. The decode path
// reads a custom bitstream format (4-byte length, 4-byte final range,
// then payload) and writes raw PCM16 to the output file.

// Dev/test driver, not the decoder library: the index-heavy buffer juggling
// translated from the C reference isn't worth hardening like the core.
#![allow(clippy::indexing_slicing)]

use opuscule::{OpusDecoder, OpusVal16, opus_get_version_string, opus_strerror, sample_to_i16};

use std::env;
use std::fs::File;
use std::io::{Read, Write};
use std::os::raw::c_int;
use std::process;

const MAX_PACKET: usize = 1500;

fn char_to_int(ch: &[u8; 4]) -> u32 {
    (ch[0] as u32) << 24 | (ch[1] as u32) << 16 | (ch[2] as u32) << 8 | ch[3] as u32
}

fn print_usage(program: &str) {
    eprintln!(
        "Usage: {} [-e] <application> <sampling rate (Hz)> <channels (1/2)> \
         <bits per second>  [options] <input> <output>",
        program
    );
    eprintln!(
        "       {} -d <sampling rate (Hz)> <channels (1/2)> \
         [options] <input> <output>\n",
        program
    );
    eprintln!("mode: voip | audio | restricted-lowdelay");
    eprintln!("options:");
    eprintln!("-e                   : only runs the encoder (output the bit-stream)");
    eprintln!("-d                   : only runs the decoder (reads the bit-stream as input)");
    eprintln!("-cbr                 : enable constant bitrate; default: variable bitrate");
    eprintln!("-cvbr                : enable constrained variable bitrate; default: unconstrained");
    eprintln!("-bandwidth <NB|MB|WB|SWB|FB> : audio bandwidth (from narrowband to fullband); default: sampling rate");
    eprintln!("-framesize <2.5|5|10|20|40|60> : frame size in ms; default: 20 ");
    eprintln!("-max_payload <bytes> : maximum payload size in bytes, default: 1024");
    eprintln!("-complexity <comp>   : complexity, 0 (lowest) ... 10 (highest); default: 10");
    eprintln!("-inbandfec           : enable SILK inband FEC");
    eprintln!("-forcemono           : force mono encoding, even for stereo input");
    eprintln!("-dtx                 : enable SILK DTX");
    eprintln!("-loss <perc>         : simulate packet loss, in percent (0-100); default: 0");
}

fn main() {
    let argv: Vec<String> = env::args().collect();
    let argc = argv.len();

    if argc < 5 {
        print_usage(&argv[0]);
        process::exit(1);
    }

    eprintln!("{}", opus_get_version_string());

    let mut args = 1;
    let encode_only;
    let decode_only;

    if argv[args] == "-e" {
        encode_only = true;
        decode_only = false;
        args += 1;
    } else if argv[args] == "-d" {
        encode_only = false;
        decode_only = true;
        args += 1;
    } else {
        encode_only = false;
        decode_only = false;
    }

    if encode_only || !decode_only {
        eprintln!("Error: this build only supports decoding (-d).");
        process::exit(1);
    }

    let sampling_rate: i32 = argv[args].parse().unwrap_or_else(|e| panic!("invalid sampling rate: {e}"));
    args += 1;
    let channels: i32 = argv[args].parse().unwrap_or_else(|e| panic!("invalid channel count: {e}"));
    args += 1;

    if ![8000, 12000, 16000, 24000, 48000].contains(&sampling_rate) {
        eprintln!("Supported sampling rates are 8000, 12000, 16000, 24000 and 48000.");
        process::exit(1);
    }

    let max_payload_bytes = MAX_PACKET;
    let mut use_inbandfec = false;
    let mut packet_loss_perc = 0i32;
    let max_frame_size: usize = 960 * 6;

    // Parse decoder options
    while args < argc - 2 {
        let opt = argv[args].as_str();
        match opt {
            "-loss" => {
                packet_loss_perc = argv[args + 1].parse().unwrap_or_else(|e| panic!("invalid loss percentage: {e}"));
                args += 2;
            }
            "-inbandfec" => {
                use_inbandfec = true;
                args += 1;
            }
            other => {
                // Encoder-only options -- reject in decode mode
                let encoder_opts = [
                    "-cbr",
                    "-cvbr",
                    "-bandwidth",
                    "-framesize",
                    "-max_payload",
                    "-complexity",
                    "-forcemono",
                    "-dtx",
                    "-sweep",
                    "-random_framesize",
                    "-sweep_max",
                    "-random_fec",
                    "-silk8k_test",
                    "-silk12k_test",
                    "-silk16k_test",
                    "-hybrid24k_test",
                    "-hybrid48k_test",
                    "-celt_test",
                    "-celt_hq_test",
                ];
                if encoder_opts.iter().any(|e| e.eq_ignore_ascii_case(other)) {
                    eprintln!("option {} is only for encoding", other);
                    process::exit(1);
                }
                eprintln!("Error: unrecognized setting: {}\n", other);
                print_usage(&argv[0]);
                process::exit(1);
            }
        }
    }

    let in_file = &argv[argc - 2];
    let out_file = &argv[argc - 1];

    let mut fin = File::open(in_file).unwrap_or_else(|_| {
        eprintln!("Could not open input file {}", in_file);
        process::exit(1);
    });
    let mut fout = File::create(out_file).unwrap_or_else(|_| {
        eprintln!("Could not open output file {}", out_file);
        process::exit(1);
    });

    // Create decoder
    let mut dec = OpusDecoder::new(sampling_rate, channels).unwrap_or_else(|err| {
        eprintln!("Cannot create decoder: {}", opus_strerror(err));
        process::exit(1);
    });

    eprintln!("Decoding with {} Hz output ({} channels)", sampling_rate, channels);

    let mut out_buf = vec![0 as OpusVal16; max_frame_size * channels as usize];
    let mut fbytes = vec![0u8; max_frame_size * channels as usize * 2];
    let mut data: [Vec<u8>; 2] =
        [vec![0u8; max_payload_bytes], if use_inbandfec { vec![0u8; max_payload_bytes] } else { Vec::new() }];
    let mut len = [0i32; 2];
    let mut enc_final_range = [0u32; 2];
    let mut toggle: usize = 0;
    let mut count: i64 = 0;
    let mut skip: i32 = 0;
    let mut lost_prev = true;
    let mut bits = 0.0f64;
    let mut bits_max = 0.0f64;
    let mut bits2 = 0.0f64;
    let frame_size = sampling_rate / 50;
    // Simple seeded RNG matching C rand() behavior isn't needed for exact
    // match -- packet_loss_perc is 0 in run_vectors.sh. We use a basic LCG
    // only if loss simulation is requested.
    let mut rng_state: u32 = 0;
    let simple_rand = |state: &mut u32| -> u32 {
        *state = state.wrapping_mul(1103515245).wrapping_add(12345);
        (*state >> 16) & 0x7fff
    };

    loop {
        // Read packet header: 4-byte length + 4-byte final range
        let mut ch = [0u8; 4];
        if fin.read_exact(&mut ch).is_err() {
            break;
        }
        len[toggle] = char_to_int(&ch) as i32;
        if len[toggle] > max_payload_bytes as i32 || len[toggle] < 0 {
            eprintln!("Invalid payload length: {}", len[toggle]);
            break;
        }
        if fin.read_exact(&mut ch).is_err() {
            break;
        }
        enc_final_range[toggle] = char_to_int(&ch);
        let read_len = len[toggle] as usize;
        if fin.read_exact(&mut data[toggle][..read_len]).is_err() {
            eprintln!("Ran out of input, expecting {} bytes", len[toggle]);
            break;
        }

        // Decode
        let use_fec_i = if use_inbandfec { 1usize } else { 0 };
        let lost = len[toggle] == 0 || (packet_loss_perc > 0 && (simple_rand(&mut rng_state) % 100) < packet_loss_perc as u32);

        if count >= use_fec_i as i64 {
            // Pick the packet to decode and whether to request in-band FEC,
            // mirroring the C: when FEC is on and the previous frame was OK we
            // decode the *next* packet normally; otherwise we decode this one
            // (None if lost), requesting FEC only on the lost-prev recovery path.
            let (packet, fec): (Option<&[u8]>, bool) = if use_inbandfec && !lost_prev {
                let other = 1 - toggle;
                (Some(&data[other][..len[other] as usize]), false)
            } else {
                let packet = if lost { None } else { Some(&data[toggle][..len[toggle] as usize]) };
                (packet, use_inbandfec && lost_prev)
            };
            let output_samples = match dec.decode(packet, &mut out_buf, fec) {
                Ok(n) => n as c_int,
                Err(e) => e,
            };

            if output_samples > 0 {
                if output_samples > skip {
                    let write_samples = (output_samples - skip) as usize;
                    let skip_offset = skip as usize * channels as usize;
                    for i in 0..write_samples * channels as usize {
                        let s = sample_to_i16(out_buf[i + skip_offset]);
                        fbytes[2 * i] = (s & 0xFF) as u8;
                        fbytes[2 * i + 1] = ((s >> 8) & 0xFF) as u8;
                    }
                    let write_bytes = write_samples * channels as usize * 2;
                    fout.write_all(&fbytes[..write_bytes]).unwrap_or_else(|e| panic!("error writing output: {e}"));
                }
                if output_samples < skip {
                    skip -= output_samples;
                } else {
                    skip = 0;
                }
            } else {
                let msg = opus_strerror(output_samples);
                eprintln!("error decoding frame: {}", msg);
            }
        }

        // Get decoder final range for consistency check
        let dec_final_range = dec.final_range();

        // Compare encoder/decoder range coder states
        let range_idx = toggle ^ use_fec_i;
        if enc_final_range[range_idx] != 0 && !lost && !lost_prev && dec_final_range != enc_final_range[range_idx] {
            eprintln!(
                "Error: Range coder state mismatch \
                 between encoder and decoder \
                 in frame {}: 0x{:8x} vs 0x{:8x}",
                count, enc_final_range[range_idx], dec_final_range
            );
            process::exit(1);
        }

        lost_prev = lost;
        bits += len[toggle] as f64 * 8.0;
        let frame_bits = len[toggle] as f64 * 8.0;
        if frame_bits > bits_max {
            bits_max = frame_bits;
        }
        bits2 += len[toggle] as f64 * len[toggle] as f64 * 64.0;
        count += 1;
        if use_inbandfec {
            toggle ^= 1;
        }
    }

    eprintln!(
        "average bitrate:             {:>7.3} kb/s",
        1e-3 * bits * sampling_rate as f64 / (frame_size as f64 * count as f64)
    );
    eprintln!("maximum bitrate:             {:>7.3} bkp/s", 1e-3 * bits_max * sampling_rate as f64 / frame_size as f64);
    eprintln!(
        "bitrate standard deviation:  {:>7.3} kb/s",
        1e-3 * (bits2 / count as f64 - bits * bits / (count as f64 * count as f64)).sqrt() * sampling_rate as f64
            / frame_size as f64
    );
    // `dec` drops here — no explicit destroy needed.
}
