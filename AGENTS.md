# AGENTS.md

This file provides guidance to coding agents when working in this repo.

See the [README](README.md) for an overview of this project -
goals, limitations, build instructions, development status, etc.


## Formatting

- Never write em-dashes (—). Just use a normal dash (-). Keep things ASCII when possible.


## Goals

The C→Rust translation of the reference Opus decoder is **done**: this is now a
pure-Rust crate (no C left). The current phase is making it idiomatic, safe Rust
while keeping decode output **bit-exact** - prefer iterators/slices over index
loops, and don't anchor on the old C structure anymore.

We also want to *build knowledge*. Whenever applicable, reference RFC
sections/paragraphs in doc comments, a little like Literate Programming, so
onboarding future maintainers is smooth.


## Process

- Pure idiomatic-Rust crate: no C objects, no `build.rs`, no C ABI (`extern "C"`
  / `#[no_mangle]`), no heap allocation in the decode path. The public API is
  `OpusDecoder::new`/`decode`; `opus_compare` and `opus_demo` are Rust bins.
- Small, atomic, reviewable commits with detailed messages - one logical change
  at a time.
- Every change must stay bit-exact: run `bash tests/run_vectors.sh` (serially -
  it shares a scratch file), and tests must pass on both float and fixed-point.
  Investigate any quality delta; never edit `tests/quality_baseline.txt` to mask one.

### Build configurations

Cargo features are additive, so `no_std` = turning off the default `std`:

- `cargo build` — default: floating-point, `std` (system math). The run_vectors
  baseline.
- `--no-default-features --features fixed-point` — `no_std`, integer kernels, no
  math dependency.
- `--no-default-features --features libm` — `no_std` floating-point, math via the
  pure-Rust `libm` crate.
- `--no-default-features` (float, no math lib) — a `compile_error!` explains the
  choices.

The float-via-`libm` path is not vector-checked and may differ by an ULP from
the system libm (different implementation); fixed-point and std-float are
bit-exact.


## Code quality

- Run `cargo fmt` before every commit.
- Always also `cargo check --features fixed-point` (the float/fixed dual build).
  When touching `no_std`/feature-gated or math code, also check
  `--no-default-features --features fixed-point` and `--features libm`.
- The crate is `#![forbid(unsafe_code)]` - there is no `unsafe` anywhere, and
  any you add is a hard compile error. Find a safe construction instead (e.g.
  `Default`/`array::from_fn` rather than `mem::zeroed`, explicit copies rather
  than slice transmutes). Relax the lint to `deny` only if a vetted unsafe
  block ever genuinely earns its place.

### Panicking (no `.unwrap()` / `.expect()`)

Both are clippy-denied. Recover or propagate where it makes sense; otherwise:

- `Option`: use `util::OrPanic` - `or_panic(v)`, `or_panic_dbg(v)`, `or_panic_with(|| msg)`.
- `Result`: `unwrap_or_else(|e| panic!("...: {e:?}"))`.

The `or_panic` argument is the *failure message*, printed only when it fires -
NOT a satisfied precondition. (Don't `s/expect/or_panic/`: `expect`'s string is
the condition you expect to hold, which is backwards as a panic message.)
Pass a string only if it tells you something the source line (where the
backtrace points) doesn't; otherwise pass the non-obvious runtime values, no
`let` temps. Tuples are fine: `or_panic_dbg(("bad x", x))`.

### Indexing (no panicking `x[i]` / `x[a..b]`)

`indexing_slicing` is clippy-denied (both element and range forms). Default to
*rewriting* the loop: `chunks`/`chunks_mut`, `split_at(_mut)_checked`,
`util::zip`/`zip3`/`zip4` (preferred over `a.iter().zip(b)`), `windows`,
`copy_from_slice`/`fill`/`first(_mut)`. Sub-ranges: `xs.get(a..b)`, never
`iter().take(b).skip(a)` (silently swallows OOB). Reserve `#[allow]` for genuine
math kernels (FFT/MDCT butterflies, Levinson, Givens, cwrs); dense SILK kernels
may take a module-wide `#![allow]` (voice path deprioritized vs CELT).
