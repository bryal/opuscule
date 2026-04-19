// Build script — formerly compiled the RFC 6716 reference C sources into
// a static library that Cargo linked into our binaries. Now that the
// entire decoder has been translated to Rust, no C sources remain and
// this script exists only to keep the cargo-build pipeline stable while
// the cc build-dependency is removed in a follow-up.

fn main() {
    println!("cargo:rerun-if-changed=build.rs");
}
