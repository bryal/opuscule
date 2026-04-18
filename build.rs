// Build script: compiles the RFC 6716 reference C sources into a static
// library that Cargo links into our binaries. The `fixed-point` feature
// toggles between floating-point and fixed-point SILK paths (CELT sources
// are shared, compiled conditionally via #ifdef FIXED_POINT).
//
// Only decoder-side sources are compiled. Encoder code has been removed
// from celt.c and encoder-only source files are excluded.

const CELT_SOURCES: &[&str] = &[];

const SILK_SOURCES: &[&str] = &[
    // Decoder
    "silk/CNG.c",
    "silk/init_decoder.c",
    "silk/decode_core.c",
    "silk/decode_frame.c",
    "silk/decode_parameters.c",
    "silk/decode_indices.c",
    "silk/decode_pulses.c",
    "silk/decoder_set_fs.c",
    "silk/dec_API.c",
    "silk/PLC.c",
    // Shared (used by decoder path)
    "silk/resampler.c",
];

const OPUS_SOURCES: &[&str] = &[];

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
