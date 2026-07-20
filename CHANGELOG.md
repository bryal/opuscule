# Changelog


v0.2.0 (2026-07-20)
------------------------------------------------------------

Conformance and robustness. v0.1.0 implemented RFC 6716; this release applies
the full set of **RFC 8251** corrections on top of it, ports several later
libopus hardening fixes, and now **builds on stable Rust**. All 12 reference
vectors still pass in all four configs, now measured against the RFC 8251
reference set.

### Significant changes

- *All of RFC 8251*

  The most audible correction is **section 9's hybrid folding**: at low hybrid
  bitrates a lone coded CELT band was followed by one too wide to fold, so it
  fell back to white noise — audible pre-echo on transients. We now seed that
  second band from the first so it folds properly. This is what motivated
  switching the test suite to the RFC 8251 vectors.

  Most of the rest hardens the decoder against crafted bitstreams, where the C
  reference had overflows that were UB (one an out-of-bounds table read,
  CVE-2017-0381). `forbid(unsafe)` plus bounds checks already made us
  memory-safe, but wrapping arithmetic could still corrupt output or panic a
  debug build, so this restores the normative saturating semantics: capping band
  energy before `celt_exp2` (§8), saturating the SILK inverse-gain and NLSF
  paths (§6, §7), and a padding scan that no longer overflows its length counter
  on a >16 MB packet (§4). Section 3 clears SILK stereo state on a mode switch,
  so a CELT→SILK transition no longer carries stale memory into a single-sample
  impulse.

  **Section 10** disables intensity-stereo phase inversion for mono output,
  where the two out-of-phase channels would otherwise cancel on downmix; mono is
  now verified against the phase-inversion-disabled `m.dec` references.

- *Ported later libopus hardening fixes*

  Fixes from upstream that matter for lossy transport and adversarial input. The
  PLC/FEC path was restructured to conceal the full requested duration and
  return exactly `frame_size` for every mode, with proper in-band-FEC dispatch
  (7fcd66c4); a CELT→SILK/hybrid transition conceals at the previous packet's
  bandwidth, not the new one (92ffce62). Packet parameters are now committed to
  decoder state only once we decide to decode, so a rejected packet can't
  desync the following frames (a5bd4409, 5e50f1f9); the multistream path
  validates every sub-stream before decoding any, so a malformed later stream
  can't advance the earlier sub-decoders (ed463234). These paths aren't in the
  reference vectors, so they come with dedicated contract tests.

- *Builds on stable Rust*

  Two static FFT twiddle tables built themselves with a const `array::map`,
  which isn't const-stable and was the sole reason for needing nightly. An
  explicit const-fn fill loop produces bit-identical tables and drops the
  `#![feature(...)]` line (verified on 1.96.0 across all three math backends).

### Miscellaneous improvements

- Centralize the libopus-style status codes (`OPUS_OK`, `BAD_ARG`, …) and the
  TOC mode/bandwidth constants, each of which had been defined identically in
  several modules.
- Give `MsDecoder` typed arguments: bounded `usize` counts instead of C-style
  `i32` with `< 0` guards, and `stream_channels()` returns `Channels`. A
  too-short mapping returns `BadArg` rather than panicking.
- Prune comments and doc links left from the C-translation era, drop dead code,
  tighten over-broad visibility, and de-C-ify some control flow (bit-identical),
  with a regression test that finally reaches the noise/CNG PLC branch.
- Document the project's scope in the README and crate docs, with an up-front
  note that the translation is heavily AI-assisted.
- Run doctests in both builds in the pre-commit hook (clippy `--all-targets`
  doesn't compile them).

### Fixes

- `opus_demo` printed "bkp/s" instead of "kb/s" (a typo from the C reference).
- The usage doctest didn't typecheck under `fixed-point`, where the sample type
  is `i16`.


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
