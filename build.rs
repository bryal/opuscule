// Build script: compiles the RFC 6716 reference C sources into a static
// library that Cargo links into our binaries. The `fixed-point` feature
// toggles between floating-point and fixed-point SILK paths (CELT sources
// are shared, compiled conditionally via #ifdef FIXED_POINT).
//
// Only decoder-side sources are compiled. Encoder code has been removed
// from celt.c and encoder-only source files are excluded.

const CELT_SOURCES: &[&str] = &[
    "celt/bands.c",
    "celt/celt.c",
    "celt/kiss_fft.c",
    "celt/mdct.c",
    "celt/modes.c",
    "celt/pitch.c",
];

const SILK_SOURCES: &[&str] = &[
    // Decoder
    "silk/CNG.c",
    "silk/code_signs.c",
    "silk/init_decoder.c",
    "silk/decode_core.c",
    "silk/decode_frame.c",
    "silk/decode_parameters.c",
    "silk/decode_indices.c",
    "silk/decode_pulses.c",
    "silk/decoder_set_fs.c",
    "silk/dec_API.c",
    "silk/PLC.c",
    "silk/decode_pitch.c",
    // Shared (used by decoder path)
    "silk/LPC_analysis_filter.c",
    "silk/pitch_est_tables.c",
    "silk/gain_quant.c",
    "silk/interpolate.c",
    "silk/NLSF_decode.c",
    "silk/NLSF_unpack.c",
    "silk/NLSF2A.c",
    "silk/NLSF_stabilize.c",
    "silk/NLSF_VQ_weights_laroia.c",
    "silk/process_NLSFs.c",
    "silk/shell_coder.c",
    "silk/tables_gain.c",
    "silk/tables_LTP.c",
    "silk/tables_NLSF_CB_NB_MB.c",
    "silk/tables_NLSF_CB_WB.c",
    "silk/tables_other.c",
    "silk/tables_pitch_lag.c",
    "silk/tables_pulses_per_block.c",
    "silk/biquad_alt.c",
    "silk/bwexpander_32.c",
    "silk/bwexpander.c",
    "silk/debug.c",
    "silk/inner_prod_aligned.c",
    "silk/lin2log.c",
    "silk/log2lin.c",
    "silk/LPC_inv_pred_gain.c",
    "silk/table_LSF_cos.c",
    "silk/resampler.c",
    "silk/resampler_down2_3.c",
    "silk/resampler_down2.c",
    "silk/resampler_private_AR2.c",
    "silk/resampler_private_down_FIR.c",
    "silk/resampler_private_IIR_FIR.c",
    "silk/resampler_private_up2_HQ.c",
    "silk/resampler_rom.c",
    "silk/sigm_Q15.c",
    "silk/sort.c",
    "silk/sum_sqr_shift.c",
    "silk/stereo_MS_to_LR.c",
    "silk/stereo_decode_pred.c",
];

const OPUS_SOURCES: &[&str] =
    &["src/opus.c", "src/opus_decoder.c", "src/opus_multistream.c", "src/repacketizer.c"];

fn main() {
    let fixed = std::env::var("CARGO_FEATURE_FIXED_POINT").is_ok();

    let mut build = cc::Build::new();
    build
        .include("c/include")
        .include("c/celt")
        .include("c/silk")
        .include("c/silk/float")
        .include("c/silk/fixed")
        .include("c/src")
        .define("OPUS_BUILD", None)
        .define("USE_ALLOCA", None)
        .define("restrict", Some(""))
        .define("OPUS_VERSION", Some("\"1.0.0\""));

    if fixed {
        build.define("FIXED_POINT", Some("1"));
        build.define("DISABLE_FLOAT_API", None);
    }

    for f in CELT_SOURCES {
        build.file(format!("c/{f}"));
    }
    for f in SILK_SOURCES {
        build.file(format!("c/{f}"));
    }
    for f in OPUS_SOURCES {
        build.file(format!("c/{f}"));
    }

    build.compile("opus");

    println!("cargo:rerun-if-changed=c/");
}
