// Translated from c/celt/mathops.c and c/celt/mathops.h (RFC 6716).
//
// Various math utility functions used by the CELT layer.
// Only the mode-independent functions are translated here; the
// fixed-point-only polynomial approximations (celt_sqrt, celt_rsqrt_norm,
// celt_cos_norm, celt_rcp, frac_div32) will be translated once the
// fixed-point arithmetic macros are available in Rust.

use crate::entcode::ec_ilog;

/// Compute floor(sqrt(val)) with exact arithmetic.
///
/// Uses a binary search approach: finds the largest binary digit b such
/// that (g+b)*(g+b) <= val, and adds it to the running solution g.
/// Translated from c/celt/mathops.c isqrt32().
///
/// This has been tested on all possible 32-bit inputs (in the C version).
#[unsafe(no_mangle)]
pub extern "C" fn isqrt32(val: u32) -> u32 {
    if val == 0 {
        return 0;
    }
    let mut val = val;
    let mut g: u32 = 0;
    let mut bshift = (ec_ilog(val) - 1) >> 1;
    let mut b: u32 = 1 << bshift;
    loop {
        let t = ((g << 1) + b) << bshift as u32;
        if t <= val {
            g += b;
            val -= t;
        }
        b >>= 1;
        bshift -= 1;
        if bshift < 0 {
            break;
        }
    }
    g
}

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
    fn test_isqrt32() {
        assert_eq!(isqrt32(0), 0);
        assert_eq!(isqrt32(1), 1);
        assert_eq!(isqrt32(2), 1);
        assert_eq!(isqrt32(3), 1);
        assert_eq!(isqrt32(4), 2);
        assert_eq!(isqrt32(8), 2);
        assert_eq!(isqrt32(9), 3);
        assert_eq!(isqrt32(15), 3);
        assert_eq!(isqrt32(16), 4);
        assert_eq!(isqrt32(100), 10);
        assert_eq!(isqrt32(255), 15);
        assert_eq!(isqrt32(256), 16);
        assert_eq!(isqrt32(65535), 255);
        assert_eq!(isqrt32(65536), 256);
        assert_eq!(isqrt32(0xFFFFFFFF), 65535);
    }

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
