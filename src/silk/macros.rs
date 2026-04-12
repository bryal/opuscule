//! SILK fixed-point arithmetic primitives.
//!
//! Translated from the macro/inline infrastructure in
//! `c/silk/{macros.h, Inlines.h, SigProc_FIX.h}`. Names mirror the C side
//! (snake_case Rust equivalents of the silk_* C macros) so leaf functions
//! can be ported by sight. Each helper is added on demand as the SILK port
//! progresses — this is not an upfront port of the entire macro library.
//!
//! Signed-overflow semantics: the C macros assume two's-complement wrap
//! (relying on the de-facto behaviour of every supported compiler), so the
//! Rust translations use `wrapping_*` to match.

/// `silk_CLZ32` — count leading zero bits of a 32-bit integer (32 for 0).
///
/// The C version is a hand-rolled nibble search; Rust's `leading_zeros`
/// intrinsic produces an identical value.
#[inline]
pub fn silk_clz32(in32: i32) -> i32 {
    (in32 as u32).leading_zeros() as i32
}

/// `silk_ROR32` — rotate right.
///
/// The C macro relies on shift behaviour for `rot` in `(0, 32)`. The only
/// caller in the decoder so far (`silk_CLZ_FRAC`) passes `24 - lzeros`, which
/// can be negative; we use `rem_euclid` to fold any signed amount into a
/// valid rotate count, matching the modular semantics of a true rotate.
#[inline]
pub fn silk_ror32(a32: i32, rot: i32) -> i32 {
    (a32 as u32).rotate_right(rot.rem_euclid(32) as u32) as i32
}

/// `silk_CLZ_FRAC` — split `in_` into a leading-zero count and a 7-bit
/// fractional part (the bits immediately below the most significant `1`).
#[inline]
pub fn silk_clz_frac(in_: i32, lz: &mut i32, frac_q7: &mut i32) {
    let lzeros = silk_clz32(in_);
    *lz = lzeros;
    *frac_q7 = silk_ror32(in_, 24 - lzeros) & 0x7f;
}

/// `silk_LSHIFT` — left shift (32-bit signed).
///
/// Uses `wrapping_shl` so that callers like [`super::log2lin::silk_log2lin`],
/// which evaluate `silk_LSHIFT(1, 31)` at the high end of their input range,
/// reproduce the C wrap to `i32::MIN` instead of panicking in debug builds.
#[inline]
pub fn silk_lshift(a: i32, shift: i32) -> i32 {
    a.wrapping_shl(shift as u32)
}

/// `silk_RSHIFT` — arithmetic right shift (32-bit signed).
#[inline]
pub fn silk_rshift(a: i32, shift: i32) -> i32 {
    a >> shift
}

/// `silk_MUL` — 32 × 32 → 32 multiply (wrapping).
#[inline]
pub fn silk_mul(a32: i32, b32: i32) -> i32 {
    a32.wrapping_mul(b32)
}

/// `silk_MLA` — `a + b * c`, 32-bit wrapping.
#[inline]
pub fn silk_mla(a32: i32, b32: i32, c32: i32) -> i32 {
    a32.wrapping_add(b32.wrapping_mul(c32))
}

/// `silk_ADD_RSHIFT` — `a + (b >> shift)` (32-bit signed).
#[inline]
pub fn silk_add_rshift(a: i32, b: i32, shift: i32) -> i32 {
    a.wrapping_add(b >> shift)
}

/// `silk_ADD_RSHIFT32` — alias of [`silk_add_rshift`] for the 32-bit form.
///
/// In C the two macros are textually different but compile to the same
/// expression; preserved here under both names so call sites read like
/// the original.
#[inline]
pub fn silk_add_rshift32(a32: i32, b32: i32, shift: i32) -> i32 {
    silk_add_rshift(a32, b32, shift)
}

/// `silk_RSHIFT_ROUND` — round-to-nearest right shift (signed, ties up).
///
/// Mirrors the C macro's two-branch form. The `shift == 1` branch avoids
/// the `(a >> 0) + 1` overflow on `i32::MAX` that the general branch would
/// hit; the general branch implements `(a + (1 << (shift-1))) >> shift`
/// using only one shift of the input.
#[inline]
pub fn silk_rshift_round(a: i32, shift: i32) -> i32 {
    if shift == 1 { (a >> 1) + (a & 1) } else { ((a >> (shift - 1)) + 1) >> 1 }
}

/// `silk_SMULBB` — `(int16)a * (int16)b`, returning the full 32-bit product.
#[inline]
pub fn silk_smulbb(a32: i32, b32: i32) -> i32 {
    (a32 as i16 as i32).wrapping_mul(b32 as i16 as i32)
}

/// `silk_SMULWB` — `(a32 * (int16)b32) >> 16`.
///
/// Split-half formulation matching `silk/macros.h` — the same structure as
/// [`silk_smlawb`] but without the accumulator add.
#[inline]
pub fn silk_smulwb(a32: i32, b32: i32) -> i32 {
    let b16 = b32 as i16 as i32;
    let hi = (a32 >> 16).wrapping_mul(b16);
    let lo = ((a32 & 0xFFFF).wrapping_mul(b16)) >> 16;
    hi.wrapping_add(lo)
}

/// `silk_SMULWW` — full 32×32 → 32 multiply (top 32 bits of the 64-bit product,
/// with rounding).
///
/// Equivalent to `silk_MLA(silk_SMULWB(a, b), a, silk_RSHIFT_ROUND(b, 16))`.
#[inline]
pub fn silk_smulww(a32: i32, b32: i32) -> i32 {
    silk_mla(silk_smulwb(a32, b32), a32, silk_rshift_round(b32, 16))
}

/// `silk_SMLAWB` — `a32 + ((b32 * (int16)c32) >> 16)`.
///
/// Mirrors the split-half formulation in `silk/macros.h` (avoiding 64-bit
/// arithmetic) so the bit pattern matches the C exactly:
///
/// ```text
/// a32
///   + ( (b32 >> 16) * (int16)c32 )
///   + ( ((b32 & 0xFFFF) * (int16)c32) >> 16 )
/// ```
#[inline]
pub fn silk_smlawb(a32: i32, b32: i32, c32: i32) -> i32 {
    let c16 = c32 as i16 as i32;
    let hi = (b32 >> 16).wrapping_mul(c16);
    let lo = ((b32 & 0xFFFF).wrapping_mul(c16)) >> 16;
    a32.wrapping_add(hi.wrapping_add(lo))
}
