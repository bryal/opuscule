// Translated from c/celt/mathops.c and c/celt/mathops.h (RFC 6716).
//
// Various math utility functions used by the CELT layer.
// Only the mode-independent functions are translated here; the
// fixed-point-only polynomial approximations (celt_sqrt, celt_rsqrt_norm,
// celt_cos_norm, celt_rcp, frac_div32) will be translated once the
// fixed-point arithmetic macros are available in Rust.

/// Multiplies two 16-bit fractional values. Bit-exactness is important.
/// Translated from FRAC_MUL16 macro in c/celt/mathops.h.
#[inline]
pub fn frac_mul16(a: i16, b: i16) -> i32 {
    (16384 + (a as i32) * (b as i32)) >> 15
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frac_mul16() {
        // 0.5 * 0.5 in Q15 = 16384 * 16384 => should give ~8192 (0.25 in Q15)
        assert_eq!(frac_mul16(16384, 16384), 8192);
        // 1.0 * 1.0 doesn't fit in i16 Q15 (32767 is max)
        // 32767 * 32767 => (16384 + 1073676289) >> 15 = 1073692673 >> 15 = 32766
        assert_eq!(frac_mul16(32767, 32767), 32766);
        assert_eq!(frac_mul16(0, 32767), 0);
    }
}
