// Translated from c/celt/arch.h and c/celt/fixed_generic.h (RFC 6716).
//
// Type aliases and arithmetic helpers for the dual float/fixed-point build.
// In float mode, opus_val16/opus_val32/celt_norm are all f32 and the
// arithmetic macros are identity operations. In fixed-point mode they
// are i16/i32 and the macros become real shifts and multiplies.
//
// Only the subset needed by the decoder is translated here.

// In a no_std float build, `f32`/`f64` have no inherent `sqrt`/`cos`/`exp`/
// `floor` (those live in std), so provide them via the pure-Rust `libm` crate.
// With `std` the inherent methods are used and this trait isn't compiled; the
// fixed-point build doesn't touch float math at all.
#[cfg(all(not(feature = "std"), not(feature = "fixed-point")))]
pub(crate) trait FloatMath {
    fn sqrt(self) -> Self;
    fn cos(self) -> Self;
    fn exp(self) -> Self;
    fn floor(self) -> Self;
}

#[cfg(all(not(feature = "std"), not(feature = "fixed-point")))]
impl FloatMath for f32 {
    fn sqrt(self) -> f32 {
        libm::sqrtf(self)
    }
    fn cos(self) -> f32 {
        libm::cosf(self)
    }
    fn exp(self) -> f32 {
        libm::expf(self)
    }
    fn floor(self) -> f32 {
        libm::floorf(self)
    }
}

#[cfg(all(not(feature = "std"), not(feature = "fixed-point")))]
impl FloatMath for f64 {
    fn sqrt(self) -> f64 {
        libm::sqrt(self)
    }
    fn cos(self) -> f64 {
        libm::cos(self)
    }
    fn exp(self) -> f64 {
        libm::exp(self)
    }
    fn floor(self) -> f64 {
        libm::floor(self)
    }
}

// -- Type aliases --
//
// The decoder's scalar audio values come in two precision tiers. `Val` is the
// narrow tier (bounded coefficients, gains, samples, operands) and `Wal` is the
// wider tier ("Val" but Wide - products, sums, energies, anything that needs
// more headroom). In the fixed-point build the tiers are distinct integer
// widths (i16 / i32, Q-format); in the float build both collapse to f32, since
// f32 spans the whole range and the split is purely a fixed-point concern.
//
// `CeltNorm` / `CeltSig` / `CeltEner` are the role-named members of those tiers
// (a normalized MDCT coefficient, a time-domain signal, a band energy); `Val` /
// `Wal` are the generic, role-less ones.
#[cfg(not(feature = "fixed-point"))]
pub type Val = f32;
#[cfg(not(feature = "fixed-point"))]
pub type Wal = f32;
#[cfg(not(feature = "fixed-point"))]
pub type CeltNorm = f32;
#[cfg(not(feature = "fixed-point"))]
pub type CeltSig = f32;
#[cfg(not(feature = "fixed-point"))]
pub type CeltEner = f32;

#[cfg(feature = "fixed-point")]
pub type Val = i16;
#[cfg(feature = "fixed-point")]
pub type Wal = i32;
#[cfg(feature = "fixed-point")]
pub type CeltNorm = i16;
#[cfg(feature = "fixed-point")]
pub type CeltSig = i32;
#[cfg(feature = "fixed-point")]
pub type CeltEner = i32;

// -- Constants --

#[cfg(not(feature = "fixed-point"))]
pub const Q15ONE: f32 = 1.0;
#[cfg(feature = "fixed-point")]
pub const Q15ONE: i16 = 32767;

#[cfg(not(feature = "fixed-point"))]
pub const EPSILON: f32 = 1e-15;
#[cfg(feature = "fixed-point")]
pub const EPSILON: i32 = 1;

#[cfg(not(feature = "fixed-point"))]
pub const NORM_SCALING: f32 = 1.0;
#[cfg(feature = "fixed-point")]
pub const NORM_SCALING: CeltNorm = 16384;

pub const SPREAD_NONE: i32 = 0;

// -- Compile-time fixed-point quantisation helpers --

#[cfg(not(feature = "fixed-point"))]
#[inline(always)]
pub fn qconst16(x: f32, _bits: i32) -> f32 {
    x
}
#[cfg(feature = "fixed-point")]
#[inline(always)]
pub fn qconst16(x: f32, bits: i32) -> i16 {
    (x * ((1 << bits) as f32) + 0.5) as i16
}

#[cfg(not(feature = "fixed-point"))]
#[inline(always)]
pub fn qconst32(x: f32, _bits: i32) -> f32 {
    x
}
#[cfg(feature = "fixed-point")]
#[inline(always)]
pub fn qconst32(x: f32, bits: i32) -> i32 {
    (x * ((1i64 << bits) as f32) + 0.5) as i32
}

// -- Float-mode arithmetic (identity operations) --

#[cfg(not(feature = "fixed-point"))]
mod float_ops {
    #[cfg(not(feature = "std"))]
    use super::FloatMath;

    #[inline(always)]
    pub fn mult16_16(a: f32, b: f32) -> f32 {
        a * b
    }
    #[inline(always)]
    pub fn mac16_16(c: f32, a: f32, b: f32) -> f32 {
        c + a * b
    }
    #[inline(always)]
    pub fn mult16_16_q15(a: f32, b: f32) -> f32 {
        a * b
    }
    #[inline(always)]
    pub fn mult16_16_p15(a: f32, b: f32) -> f32 {
        a * b
    }
    #[inline(always)]
    pub fn shr16(a: f32, _shift: i32) -> f32 {
        a
    }
    #[inline(always)]
    pub fn shl32(a: f32, _shift: i32) -> f32 {
        a
    }
    #[inline(always)]
    pub fn shr32(a: f32, _shift: i32) -> f32 {
        a
    }
    #[inline(always)]
    pub fn pshr32(a: f32, _shift: i32) -> f32 {
        a
    }
    #[inline(always)]
    pub fn vshr32(a: f32, _shift: i32) -> f32 {
        a
    }
    #[inline(always)]
    pub fn round16(a: f32, _shift: i32) -> f32 {
        a
    }
    #[inline(always)]
    pub fn mult32_32_q31(a: f32, b: f32) -> f32 {
        a * b
    }
    #[inline(always)]
    pub fn frac_div32(a: f32, b: f32) -> f32 {
        a / b
    }
    #[inline(always)]
    pub fn add16(a: f32, b: f32) -> f32 {
        a + b
    }
    #[inline(always)]
    pub fn sub16(a: f32, b: f32) -> f32 {
        a - b
    }
    #[inline(always)]
    pub fn add32(a: f32, b: f32) -> f32 {
        a + b
    }
    #[inline(always)]
    pub fn sub32(a: f32, b: f32) -> f32 {
        a - b
    }
    #[inline(always)]
    pub fn neg16(x: f32) -> f32 {
        -x
    }
    #[inline(always)]
    pub fn min16(a: f32, b: f32) -> f32 {
        a.min(b)
    }
    #[inline(always)]
    pub fn max16(a: f32, b: f32) -> f32 {
        a.max(b)
    }
    #[inline(always)]
    pub fn max32(a: f32, b: f32) -> f32 {
        a.max(b)
    }
    #[inline(always)]
    pub fn celt_rsqrt(x: f32) -> f32 {
        1.0 / x.sqrt()
    }
    #[inline(always)]
    pub fn shl16(a: f32, _shift: i32) -> f32 {
        a
    }
    #[inline(always)]
    pub fn extract16(x: f32) -> f32 {
        x
    }
    #[inline(always)]
    pub fn extend32(x: f32) -> f32 {
        x
    }
    #[inline(always)]
    pub fn half16(x: f32) -> f32 {
        0.5 * x
    }
    #[inline(always)]
    pub fn half32(x: f32) -> f32 {
        0.5 * x
    }
    #[inline(always)]
    pub fn mult16_32_q15(a: f32, b: f32) -> f32 {
        a * b
    }
    #[inline(always)]
    pub fn celt_div(a: f32, b: f32) -> f32 {
        a / b
    }
    #[inline(always)]
    pub fn celt_sqrt(x: f32) -> f32 {
        x.sqrt()
    }
    #[inline(always)]
    pub fn celt_rsqrt_norm(x: f32) -> f32 {
        1.0 / x.sqrt()
    }
    #[inline(always)]
    pub fn celt_cos_norm(x: f32) -> f32 {
        (0.5 * core::f32::consts::PI * x).cos()
    }
    /// Base-2 exponential. Matches the C macro:
    /// #define celt_exp2(x) ((float)exp(0.6931471805599453094*(x)))
    #[inline(always)]
    pub fn celt_exp2(x: f32) -> f32 {
        (core::f64::consts::LN_2 * (x as f64)).exp() as f32
    }
}

// -- Fixed-point arithmetic --

#[cfg(feature = "fixed-point")]
mod fixed_ops {
    use crate::entcode::ec_ilog;

    // In the C, ADD16/SUB16 cast both operands to opus_val16 (i16).
    // MULT16_16 takes (opus_val16, opus_val16) and returns opus_val32 (i32).
    // MULT16_16_Q15 returns i32. Callers must cast to i16 when passing
    // the result to add16/sub16, matching the C's implicit casts.

    #[inline(always)]
    pub fn mult16_16(a: i16, b: i16) -> i32 {
        (a as i32) * (b as i32)
    }
    #[inline(always)]
    pub fn mac16_16(c: i32, a: i16, b: i16) -> i32 {
        c + (a as i32) * (b as i32)
    }
    #[inline(always)]
    pub fn mult16_16_q15(a: i16, b: i16) -> i32 {
        ((a as i32) * (b as i32)) >> 15
    }
    #[inline(always)]
    pub fn mult16_16_p15(a: i16, b: i16) -> i32 {
        (16384 + (a as i32) * (b as i32)) >> 15
    }
    #[inline(always)]
    fn mult16_16su(a: i16, b: u16) -> i32 {
        (a as i32) * (b as i32)
    }
    #[inline(always)]
    pub fn mult16_32_q15(a: i16, b: i32) -> i32 {
        (mult16_16(a, (b >> 16) as i16) << 1) + (mult16_16su(a, (b & 0xffff) as u16) >> 15)
    }
    #[inline(always)]
    pub fn mult32_32_q31(a: i32, b: i32) -> i32 {
        (mult16_16((a >> 16) as i16, (b >> 16) as i16) << 1)
            + (mult16_16su((a >> 16) as i16, (b & 0xffff) as u16) >> 15)
            + (mult16_16su((b >> 16) as i16, (a & 0xffff) as u16) >> 15)
    }
    #[inline(always)]
    pub fn round16(a: i32, shift: i32) -> i16 {
        pshr32(a, shift) as i16
    }
    #[inline(always)]
    pub fn shr16(a: i16, shift: i32) -> i16 {
        a >> shift
    }
    #[inline(always)]
    pub fn shl16(a: i16, shift: i32) -> i16 {
        ((a as u16) << shift) as i16
    }
    #[inline(always)]
    pub fn shr32(a: i32, shift: i32) -> i32 {
        a >> shift
    }
    #[inline(always)]
    pub fn shl32(a: i32, shift: i32) -> i32 {
        ((a as u32) << shift) as i32
    }
    #[inline(always)]
    pub fn pshr32(a: i32, shift: i32) -> i32 {
        (a + (1i32 << (shift - 1))) >> shift
    }
    #[inline(always)]
    pub fn vshr32(a: i32, shift: i32) -> i32 {
        if shift > 0 { a >> shift } else { shl32(a, -shift) }
    }
    #[inline(always)]
    pub fn add16(a: i16, b: i16) -> i16 {
        a.wrapping_add(b)
    }
    #[inline(always)]
    pub fn sub16(a: i16, b: i16) -> i16 {
        a.wrapping_sub(b)
    }
    #[inline(always)]
    pub fn add32(a: i32, b: i32) -> i32 {
        a.wrapping_add(b)
    }
    #[inline(always)]
    pub fn sub32(a: i32, b: i32) -> i32 {
        a.wrapping_sub(b)
    }
    #[inline(always)]
    pub fn neg16(x: i16) -> i16 {
        -x
    }
    #[inline(always)]
    pub fn min16(a: i16, b: i16) -> i16 {
        a.min(b)
    }
    #[inline(always)]
    pub fn max16(a: i16, b: i16) -> i16 {
        a.max(b)
    }
    #[inline(always)]
    pub fn min32(a: i32, b: i32) -> i32 {
        a.min(b)
    }
    #[inline(always)]
    pub fn max32(a: i32, b: i32) -> i32 {
        a.max(b)
    }
    #[inline(always)]
    pub fn mult16_16_q14(a: i16, b: i16) -> i32 {
        ((a as i32) * (b as i32)) >> 14
    }
    #[inline(always)]
    pub fn extract16(x: i32) -> i16 {
        x as i16
    }
    #[inline(always)]
    pub fn extend32(x: i16) -> i32 {
        x as i32
    }
    #[inline(always)]
    pub fn half16(x: i16) -> i16 {
        x >> 1
    }
    #[inline(always)]
    pub fn half32(x: i32) -> i32 {
        x >> 1
    }

    // Fixed-point math functions from mathops.c/h

    #[inline(always)]
    pub fn celt_ilog2(x: i32) -> i16 {
        debug_assert!(x > 0);
        (ec_ilog(x as u32) - 1) as i16
    }

    /// Reciprocal sqrt approximation (Q16 in, Q14 out).
    /// C: celt_rsqrt_norm() in mathops.c
    pub fn celt_rsqrt_norm(x: i32) -> i16 {
        let n = (x - 32768) as i16;
        // r = 1.4378 + n*(-0.8234 + n*0.4096)  (Q14)
        let t0 = mult16_16_q15(n, 6713) as i16;
        let t1 = add16(-13490, t0);
        let t2 = mult16_16_q15(n, t1) as i16;
        let r = add16(23557, t2);
        // y = 2*(r^2*n + r^2 - 16384)
        let r2 = mult16_16_q15(r, r) as i16;
        let y = shl16(sub16(add16(mult16_16_q15(r2, n) as i16, r2), 16384), 1);
        // Householder: r += r*y*(y*0.375 - 0.5)
        let yc = sub16(mult16_16_q15(y, 12288) as i16, 16384);
        let yr = mult16_16_q15(y, yc) as i16;
        add16(r, mult16_16_q15(r, yr) as i16)
    }

    /// Sqrt approximation (QX input, QX/2 output).
    /// C: celt_sqrt() in mathops.c
    pub fn celt_sqrt(x: i32) -> i32 {
        if x == 0 {
            return 0;
        }
        let c: [i16; 5] = [23175, 11561, -3011, 1699, -664];
        let k = (celt_ilog2(x) >> 1) - 7;
        let x = vshr32(x, 2 * k as i32);
        let n = (x - 32768) as i16;
        // Horner: C[0] + n*(C[1] + n*(C[2] + n*(C[3] + n*C[4])))
        let t = mult16_16_q15(n, c[4]) as i16;
        let t = add16(c[3], t);
        let t = mult16_16_q15(n, t) as i16;
        let t = add16(c[2], t);
        let t = mult16_16_q15(n, t) as i16;
        let t = add16(c[1], t);
        let t = mult16_16_q15(n, t) as i16;
        let rt = add16(c[0], t);
        vshr32(rt as i32, 7 - k as i32)
    }

    /// cos(pi/2 * x) approximation for fixed-point.
    /// C: celt_cos_norm() in mathops.c
    pub fn celt_cos_norm(x: i32) -> i16 {
        let x = x & 0x0001ffff;
        let x = if x > (1 << 16) { (1 << 17) - x } else { x };
        if x & 0x00007fff != 0 {
            if x < (1 << 15) { cos_pi_2(x as i16) } else { -(cos_pi_2((65536 - x) as i16) as i32) as i16 }
        } else if x & 0x0000ffff != 0 {
            0
        } else if x & 0x0001ffff != 0 {
            -32767
        } else {
            32767
        }
    }

    /// Inner cos approximation: _celt_cos_pi_2
    /// C: ADD16(1, MIN16(32766, ADD32(SUB16(L1,x2),
    ///      P15(x2, ADD32(L2, P15(x2, ADD32(L3, P15(L4,x2))))))))
    fn cos_pi_2(x: i16) -> i16 {
        let (l1, l2, l3, l4): (i16, i16, i16, i16) = (32767, -7651, 8277, -626);
        let x2 = mult16_16_p15(x, x) as i16;
        // Horner from inside out, using ADD32 (i32 add) then truncate to i16
        // at each MULT16_16_P15 boundary (matching C macro casts).
        let t0 = mult16_16_p15(l4, x2); // i32
        let t1 = (l3 as i32 + t0) as i16; // ADD32 then truncate
        let t2 = mult16_16_p15(x2, t1); // i32
        let t3 = (l2 as i32 + t2) as i16; // ADD32 then truncate
        let t4 = mult16_16_p15(x2, t3); // i32
        // ADD32(SUB16(L1,x2), t4) -> i32, then MIN16(32766, ...) truncates to i16
        let sum = sub16(l1, x2) as i32 + t4;
        add16(1, min16(32766, sum as i16))
    }

    /// Reciprocal approximation (Q15 input, Q16 output).
    /// C: celt_rcp() in mathops.c
    pub fn celt_rcp(x: i32) -> i32 {
        debug_assert!(x > 0);
        let i = celt_ilog2(x);
        let n = (vshr32(x, i as i32 - 15) - 32768) as i16;
        // Linear approximation: r = 30840 - 15420*n (Q14)
        let mut r = add16(30840, mult16_16_q15(-15420, n) as i16);
        // Two Newton iterations: r -= r * (r*n + r - 32768)
        let corr = add16(mult16_16_q15(r, n) as i16, add16(r, -32768i16));
        r = sub16(r, mult16_16_q15(r, corr) as i16);
        // Second iteration (subtract extra 1 for truncation compensation)
        let corr = add16(mult16_16_q15(r, n) as i16, add16(r, -32768i16));
        r = sub16(r, add16(1, mult16_16_q15(r, corr) as i16));
        vshr32(r as i32, i as i32 - 16)
    }

    /// frac_div32: fractional division (Q31 result).
    /// C: frac_div32() in mathops.c
    pub fn frac_div32(a: i32, b: i32) -> i32 {
        let shift = celt_ilog2(b) as i32 - 29;
        let a = vshr32(a, shift);
        let b = vshr32(b, shift);
        let rcp = round16(celt_rcp(round16(b, 16) as i32), 3);
        let mut result = shl32(mult16_32_q15(rcp, a), 2);
        let rem = a - mult32_32_q31(result, b);
        result += shl32(mult16_32_q15(rcp, rem), 2);
        result
    }

    /// Base-2 exponential approximation (Q10 input, Q16 output).
    /// C: celt_exp2() in mathops.h (fixed-point path).
    pub fn celt_exp2(x: i16) -> i32 {
        const D0: i16 = 16383;
        const D1: i16 = 22804;
        const D2: i16 = 14819;
        const D3: i16 = 10204;
        let integer = shr16(x, 10) as i32;
        if integer > 14 {
            return 0x7f000000;
        } else if integer < -15 {
            return 0;
        }
        let frac = shl16(x - shl16(integer as i16, 10), 4);
        let frac = add16(
            D0,
            mult16_16_q15(frac, add16(D1, mult16_16_q15(frac, add16(D2, mult16_16_q15(D3, frac) as i16)) as i16)) as i16,
        );
        vshr32(extend32(frac), -integer - 2)
    }

    #[inline(always)]
    pub fn celt_div(a: i32, b: i32) -> i32 {
        mult32_32_q31(a, celt_rcp(b))
    }
}

// Re-export whichever implementation is active

#[cfg(not(feature = "fixed-point"))]
pub use float_ops::*;

#[cfg(feature = "fixed-point")]
pub use fixed_ops::*;
