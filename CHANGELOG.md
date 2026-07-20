# Changelog


v0.1.0 (2026-06-22)
------------------------------------------------------------

The first release: a complete, working, pure-Rust Opus **decoder**.

Opuscule is a function-by-function translation of the [libopus](https://gitlab.xiph.org/xiph/opus)
reference decoder (RFC 6716). The approach was to compile the reference C and
then replace it a function at a time, checking the output stayed bit-exact
against the reference test vectors at every step — so correctness rests on that
verification rather than on line-by-line human review of the signal processing.
By this release no C remained, and the post-translation cleanup had stripped the
`extern "C"` / `#[no_mangle]` shims and turned the raw-pointer struct fields into
`&'static` slices.

The result decodes the whole of Opus: SILK, CELT, and hybrid modes, every
bandwidth from narrowband to fullband, and mono / stereo / multistream-surround
(via `MsDecoder`), plus packet loss concealment and in-band forward error
correction. You feed it raw Opus packets — container demuxing is your job (the
`play` example does it with the `ogg` crate).

Three properties were priorities from the start:

- **Safe.** `#![forbid(unsafe_code)]` — there is no `unsafe` anywhere in the crate.
- **No heap allocation** on the decode path; you supply the output buffer.
- **`no_std`-capable**, in three flavors: float + `std` (the default, and the
  bit-exact baseline), `fixed-point` (integer kernels, no math dependency), and
  `libm` (pure-Rust float math). Both the `std`-float and fixed-point builds are
  bit-exact against the reference vectors.

Ships with the `play` example (Ogg demux + `rodio` playback) and the
`opus_compare` / `opus_demo` binaries ported from the reference. Released under
MPL-2.0, with the upstream Xiph BSD license and Opus patent grants retained in
`LICENSE-XIPH`.

There is **no encoder**, no container parsing, and none of the non-RFC libopus
extensions (QEXT/24-bit, DRED/Deep PLC/OSCE) — decoding only.
