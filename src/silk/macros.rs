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

/// `silk_LIMIT_int` — clamp `a` to `[limit1, limit2]` (or `[limit2, limit1]`
/// if `limit1 > limit2`).
#[inline]
pub fn silk_limit_int(a: i32, limit1: i32, limit2: i32) -> i32 {
    if limit1 > limit2 { a.max(limit2).min(limit1) } else { a.max(limit1).min(limit2) }
}

/// `silk_min_32` — min of two i32.
#[inline]
pub fn silk_min_32(a: i32, b: i32) -> i32 {
    a.min(b)
}

/// `silk_max_int` — max of two i32.
#[inline]
pub fn silk_max_int(a: i32, b: i32) -> i32 {
    a.max(b)
}

/// `silk_ADD_LSHIFT32` — `a + (b << shift)`, 32-bit.
#[inline]
pub fn silk_add_lshift32(a: i32, b: i32, shift: i32) -> i32 {
    a.wrapping_add(silk_lshift(b, shift))
}

/// `silk_SAT16` — saturate an i32 to the i16 range.
#[inline]
pub fn silk_sat16(a: i32) -> i32 {
    a.max(i16::MIN as i32).min(i16::MAX as i32)
}

/// `silk_ADD_LSHIFT` — `a + (b << shift)`, signed.
///
/// Same as [`silk_add_lshift32`] but the C macro is used with mixed-width
/// arguments in some call sites, so we keep the name.
#[inline]
pub fn silk_add_lshift(a: i32, b: i32, shift: i32) -> i32 {
    silk_add_lshift32(a, b, shift)
}

/// `silk_ADD32_ovflw` — wrapping 32-bit add (`(u32)a + (u32)b` cast back to `i32`).
#[inline]
pub fn silk_add32_ovflw(a: i32, b: i32) -> i32 {
    a.wrapping_add(b)
}

/// `silk_SMLABB_ovflw` — `a + (int16)b * (int16)c`, with wrapping add.
#[inline]
pub fn silk_smlabb_ovflw(a32: i32, b32: i32, c32: i32) -> i32 {
    silk_add32_ovflw(a32, (b32 as i16 as i32).wrapping_mul(c32 as i16 as i32))
}

/// `silk_RSHIFT_uint` — unsigned right shift.
#[inline]
pub fn silk_rshift_uint(a: u32, shift: i32) -> u32 {
    a >> shift as u32
}

/// `silk_ADD_RSHIFT_uint` — `a + (b >> shift)`, unsigned.
#[inline]
pub fn silk_add_rshift_uint(a: u32, b: u32, shift: i32) -> u32 {
    a + (b >> shift as u32)
}

// -- 64-bit helpers --

/// `silk_SMULL` — 32 × 32 → 64 signed multiply.
#[inline]
pub fn silk_smull(a32: i32, b32: i32) -> i64 {
    (a32 as i64) * (b32 as i64)
}

/// `silk_SMMUL` — signed top-word multiply: top 32 bits of `a32 * b32`.
#[inline]
pub fn silk_smmul(a32: i32, b32: i32) -> i32 {
    (silk_smull(a32, b32) >> 32) as i32
}

/// `silk_RSHIFT_ROUND64` — round-to-nearest right shift (i64).
#[inline]
pub fn silk_rshift_round64(a: i64, shift: i32) -> i64 {
    if shift == 1 { (a >> 1) + (a & 1) } else { ((a >> (shift - 1)) + 1) >> 1 }
}

// -- Overflow-tolerant operations --

/// `silk_SUB32_ovflw` — wrapping 32-bit subtract.
#[inline]
pub fn silk_sub32_ovflw(a: i32, b: i32) -> i32 {
    a.wrapping_sub(b)
}

/// `silk_LSHIFT_ovflw` — left shift, wrapping (same as [`silk_lshift`]).
#[inline]
pub fn silk_lshift_ovflw(a: i32, shift: i32) -> i32 {
    a.wrapping_shl(shift as u32)
}

// -- Miscellaneous --

/// `silk_abs` — absolute value. WARNING: returns wrong for `i32::MIN`.
#[inline]
pub fn silk_abs_int32(a: i32) -> i32 {
    a.wrapping_abs()
}

/// `silk_DIV32_16` — divide i32 by i16-range value, returning i32.
#[inline]
pub fn silk_div32_16(a32: i32, b16: i32) -> i32 {
    a32 / b16
}

/// `silk_LSHIFT_SAT32` — left shift with saturation.
///
/// Clamps `a` to the range that won't overflow when shifted, then shifts.
#[inline]
pub fn silk_lshift_sat32(a: i32, shift: i32) -> i32 {
    let lo = i32::MIN >> shift;
    let hi = i32::MAX >> shift;
    silk_lshift(a.max(lo).min(hi), shift)
}

/// `silk_SMLABB` — `a + (int16)b * (int16)c`, non-wrapping add.
#[inline]
pub fn silk_smlabb(a32: i32, b32: i32, c32: i32) -> i32 {
    a32 + (b32 as i16 as i32) * (c32 as i16 as i32)
}

/// `silk_SMLAWW` — `a32 + ((b32 * c32) >> 16)`, rounded.
///
/// `silk_MLA(silk_SMLAWB(a, b, c), b, silk_RSHIFT_ROUND(c, 16))`
#[inline]
pub fn silk_smlaww(a32: i32, b32: i32, c32: i32) -> i32 {
    silk_mla(silk_smlawb(a32, b32, c32), b32, silk_rshift_round(c32, 16))
}

/// `silk_INVERSE32_varQ` — approximate `(1 << Qres) / b32` via Newton iteration.
///
/// Translated from `c/silk/Inlines.h`.
#[inline]
pub fn silk_inverse32_varq(b32: i32, qres: i32) -> i32 {
    /* Compute number of bits head room and normalize input */
    let b_headrm = silk_clz32(silk_abs_int32(b32)) - 1;
    let b32_nrm = silk_lshift(b32, b_headrm); /* Q: b_headrm */

    /* Inverse of b32, with 14 bits of precision */
    let b32_inv = silk_div32_16(i32::MAX >> 2, silk_rshift(b32_nrm, 16)); /* Q: 29 + 16 - b_headrm */

    /* First approximation */
    let mut result = silk_lshift(b32_inv, 16); /* Q: 61 - b_headrm */

    /* Compute residual by subtracting product of denominator and first approximation from one */
    let err_q32 = silk_lshift((1 << 29) - silk_smulwb(b32_nrm, b32_inv), 3); /* Q32 */

    /* Refinement */
    result = silk_smlaww(result, err_q32, b32_inv); /* Q: 61 - b_headrm */

    /* Convert to Qres domain */
    let lshift = 61 - b_headrm - qres;
    if lshift <= 0 { silk_lshift_sat32(result, -lshift) } else { if lshift < 32 { silk_rshift(result, lshift) } else { 0 } }
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
