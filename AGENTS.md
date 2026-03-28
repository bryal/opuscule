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
- `.unwrap()` is banned. Unless it makes more sense to recover from the error
  or propagate it to the caller, you should panic descriptively with e.g. `.expect(_)`.
- All `unsafe` blocks must have `// SAFETY:` comments.
