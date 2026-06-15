# AGENTS.md

This file provides guidance to coding agents when working in this repo.

See the [README](README.md) for an overview of this project -
goals, limitations, build instructions, development status, etc.


## Formatting

- Never write em-dashes (—). Just use a normal dash (-). Keep things ASCII when possible.


## Goals

Faithfully translate the reference Opus decoder from C to Rust.
This gives us a reference point for later optimization and idiomatic Rust rewrites.

We also want to *build knowledge*. Whenever applicable, we should reference
RFC sections/paragraphs in doc comments. A little bit like if we were doing Literate Programming.
We want to onboarding for future maintainers to be as smooth as possible.


## Process

At first, there will only be the C reference decoder.
The resulting build artifacts will naturally pass all tests.
Then, we'll translate this program from C to Rust, function by function,
all while preserving correctness and passing all preexisting tests along every step of the way.

To clarify, the entrypoints will be the `opus_compare` and `opus_demo` binaries.
These, along with everything else, will come from the C at first,
but they will probably be the first targets to translate to Rust.

The Opus library will be a composite. Originally, it will consist only of object files
produced by compiling the C. However, very early on we'll also get a Rust static lib,
which will be linked together with the C object files to produce the composite Opus library.
Meaning Rust will call C functions and vice versa. There will be lots of FFI.

Generally we'll translate one function per commit, to keep our changes "atomic"
and reviewable.


## Code quality

- Run `cargo fmt` before every commit.
- Always also `cargo check --features fixed-point` (we have a float/fixed dual build).
- All `unsafe` blocks must have `// SAFETY:` comments.

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
*rewriting* the loop: `chunks`/`chunks_mut`, `split_at_mut(_checked)`,
`util::zip`/`zip3`/`zip4` (preferred over `a.iter().zip(b)`), `windows`,
`copy_from_slice`/`fill`/`first(_mut)`. Sub-ranges: `xs.get(a..b)`, never
`iter().take(b).skip(a)` (silently swallows OOB). Reserve `#[allow]` for genuine
math kernels (FFT/MDCT butterflies, Levinson, Givens, cwrs); dense SILK kernels
may take a module-wide `#![allow]` (voice path deprioritized vs CELT).
