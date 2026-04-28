// Translated from c/celt/vq.c (RFC 6716).
//
// Vector quantisation of the residual signal. The decoder path uses
// alg_unquant() to decode pulse vectors from the bitstream and produce
// normalised MDCT coefficients for each band. renormalise_vector() and
// stereo_itheta() are shared helpers called from bands.c.
//
// exp_rotation applies a spreading transform that distributes spectral
// energy more evenly, improving perceptual quality. It uses Givens
// rotations with cos/sin derived from a gain function of K/N.
//
// See RFC 6716 Section 4.3.4.

use std::os::raw::c_int;

use crate::arch::*;
use crate::cwrs::decode_pulses;
use crate::entcode::ec_dec;

// -- Static helpers --

/// Apply Givens rotations along a single stride.
/// Used by exp_rotation to spread energy across coefficients.
fn exp_rotation1(x: &mut [CeltNorm], len: usize, stride: usize, c: OpusVal16, s: OpusVal16) {
    // Forward pass
    for i in 0..len - stride {
        let x1 = x[i];
        let x2 = x[i + stride];
        x[i + stride] = extract16(shr32(add32(mult16_16(c, x2), mult16_16(s, x1)), 15));
        x[i] = extract16(shr32(sub32(mult16_16(c, x1), mult16_16(s, x2)), 15));
    }
    // Backward pass (skip if len < 2*stride + 1, matching C's signed i>=0 guard)
    if len < 2 * stride + 1 {
        return;
    }
    for i in (0..=len - 2 * stride - 1).rev() {
        let x1 = x[i];
        let x2 = x[i + stride];
        x[i + stride] = extract16(shr32(add32(mult16_16(c, x2), mult16_16(s, x1)), 15));
        x[i] = extract16(shr32(sub32(mult16_16(c, x1), mult16_16(s, x2)), 15));
    }
}

/// Apply the spreading rotation to the coefficient vector.
/// dir=1 for forward (encode), dir=-1 for inverse (decode).
fn exp_rotation(x: &mut [CeltNorm], len: usize, dir: i32, stride: usize, k: i32, spread: i32) {
    const SPREAD_FACTOR: [i32; 3] = [15, 10, 5];

    if 2 * k >= len as i32 || spread == SPREAD_NONE {
        return;
    }
    let factor = SPREAD_FACTOR[(spread - 1) as usize];

    // C: gain = celt_div((opus_val32)MULT16_16(Q15_ONE,len),
    //                     (opus_val32)(len+factor*K));
    // In float: celt_div(1.0*len, len+factor*K), identity.
    // In fixed: MULT16_16(32767, (i16)len) -> i32, celt_div(i32, i32) -> i32.
    #[cfg(not(feature = "fixed-point"))]
    let (c, s) = {
        let gain = celt_div(Q15ONE * len as f32, len as f32 + factor as f32 * k as f32);
        let theta = half16(mult16_16_q15(gain, gain));
        let c = celt_cos_norm(theta);
        let s = celt_cos_norm(sub16(Q15ONE, theta));
        (c, s)
    };
    #[cfg(feature = "fixed-point")]
    let (c, s) = {
        let gain = celt_div(mult16_16(Q15ONE, len as i16), (len as i32) + factor * k);
        let theta = half16(mult16_16_q15(gain as i16, gain as i16) as i16);
        let c = celt_cos_norm(extend32(theta));
        let s = celt_cos_norm(extend32(sub16(Q15ONE, theta)));
        (c, s)
    };

    let mut stride2 = 0usize;
    if len >= 8 * stride {
        stride2 = 1;
        while (stride2 * stride2 + stride2) * stride + (stride >> 2) < len {
            stride2 += 1;
        }
    }

    let block_len = len / stride;
    for i in 0..stride {
        let slice = &mut x[i * block_len..(i + 1) * block_len];
        if dir < 0 {
            if stride2 > 0 {
                exp_rotation1(slice, block_len, stride2, s, c);
            }
            exp_rotation1(slice, block_len, 1, c, s);
        } else {
            let neg_s = neg16(s);
            exp_rotation1(slice, block_len, 1, c, neg_s);
            if stride2 > 0 {
                let neg_c = neg16(c);
                exp_rotation1(slice, block_len, stride2, s, neg_c);
            }
        }
    }
}

/// Normalise the decoded pulse vector: compute gain so ||g*y|| = 1,
/// then scale each element.
///
/// In float mode, k is unused (the compiler will optimise it away).
/// In fixed mode, k = celt_ilog2(Ryy) >> 1 controls the shift.
#[allow(unused_variables)]
fn normalise_residual(iy: &[c_int], x: &mut [CeltNorm], n: usize, ryy: OpusVal32, gain: OpusVal16) {
    // C: k = celt_ilog2(Ryy)>>1;   (fixed only)
    //    t = VSHR32(Ryy, 2*(k-7));
    //    g = MULT16_16_P15(celt_rsqrt_norm(t), gain);
    //    X[i] = EXTRACT16(PSHR32(MULT16_16(g, iy[i]), k+1));
    #[cfg(feature = "fixed-point")]
    {
        let k = celt_ilog2(ryy) >> 1;
        let t = vshr32(ryy, 2 * (k as i32 - 7));
        let g = mult16_16_p15(celt_rsqrt_norm(t), gain) as i16;
        for i in 0..n {
            x[i] = extract16(pshr32(mult16_16(g, iy[i] as i16), k as i32 + 1));
        }
    }
    #[cfg(not(feature = "fixed-point"))]
    {
        let t = ryy;
        let g = celt_rsqrt_norm(t) * gain;
        for i in 0..n {
            x[i] = g * iy[i] as f32;
        }
    }
}

/// Compute per-block collapse mask from the decoded pulse vector.
/// Each bit i indicates whether block i received any non-zero pulses.
fn extract_collapse_mask(iy: &[c_int], n: usize, b: usize) -> u32 {
    if b <= 1 {
        return 1;
    }
    let n0 = n / b;
    let mut collapse_mask = 0u32;
    for i in 0..b {
        for j in 0..n0 {
            if iy[i * n0 + j] != 0 {
                collapse_mask |= 1 << i;
            }
        }
    }
    collapse_mask
}

// -- Public decoder functions --

/// Decode pulse vector and combine with normalisation to produce the
/// final normalised signal coefficients for one band.
///
/// # Safety
/// `x` must point to N writable celt_norm elements.
/// `dec` must be a valid entropy decoder context.
pub unsafe fn alg_unquant(
    x: *mut CeltNorm,
    n: c_int,
    k: c_int,
    spread: c_int,
    b: c_int,
    dec: *mut ec_dec,
    gain: OpusVal16,
) -> u32 {
    debug_assert!(k > 0, "alg_unquant() needs at least one pulse");
    debug_assert!(n > 1, "alg_unquant() needs at least two dimensions");

    let n = n as usize;
    let mut iy = vec![0i32; n];

    // Decode pulse vector from bitstream
    unsafe { decode_pulses(iy.as_mut_ptr(), n as c_int, k, dec) };

    // Compute sum of squares: Ryy = sum(iy[i]^2)
    let mut ryy: OpusVal32 = 0 as OpusVal32;
    for i in 0..n {
        ryy = mac16_16(ryy, iy[i] as OpusVal16, iy[i] as OpusVal16);
    }

    // SAFETY: caller guarantees x points to n elements
    let x_slice = unsafe { std::slice::from_raw_parts_mut(x, n) };

    normalise_residual(&iy, x_slice, n, ryy, gain);
    exp_rotation(x_slice, n, -1, b as usize, k, spread);
    extract_collapse_mask(&iy, n, b as usize)
}

/// Renormalise a coefficient vector to have the given gain (norm).
///
/// # Safety
/// `x` must point to N writable celt_norm elements.
pub unsafe fn renormalise_vector(x: *mut CeltNorm, n: c_int, gain: OpusVal16) {
    let n = n as usize;
    let x = unsafe { std::slice::from_raw_parts_mut(x, n) };

    let mut e: OpusVal32 = EPSILON as OpusVal32;
    for i in 0..n {
        e = mac16_16(e, x[i], x[i]);
    }

    #[cfg(feature = "fixed-point")]
    {
        let k = celt_ilog2(e) >> 1;
        let t = vshr32(e, 2 * (k as i32 - 7));
        let g = mult16_16_p15(celt_rsqrt_norm(t), gain) as i16;
        for i in 0..n {
            x[i] = extract16(pshr32(mult16_16(g, x[i]), k as i32 + 1));
        }
    }
    #[cfg(not(feature = "fixed-point"))]
    {
        let g = celt_rsqrt_norm(e) * gain;
        for i in 0..n {
            x[i] = g * x[i];
        }
    }
}

/// Compute the stereo angle parameter itheta from two channel vectors.
/// Returns a value in [0, 16384] representing the angle in the
/// mid/side plane (0 = pure mid, 16384 = pure side).
///
/// # Safety
/// `x` and `y` must each point to N readable celt_norm elements.
pub unsafe fn stereo_itheta(x: *const CeltNorm, y: *const CeltNorm, stereo: c_int, n: c_int) -> c_int {
    let n = n as usize;
    let x = unsafe { std::slice::from_raw_parts(x, n) };
    let y = unsafe { std::slice::from_raw_parts(y, n) };

    let mut emid: OpusVal32 = EPSILON as OpusVal32;
    let mut eside: OpusVal32 = EPSILON as OpusVal32;

    if stereo != 0 {
        for i in 0..n {
            let m = add16(shr16(x[i], 1), shr16(y[i], 1));
            let s = sub16(shr16(x[i], 1), shr16(y[i], 1));
            emid = mac16_16(emid, m, m);
            eside = mac16_16(eside, s, s);
        }
    } else {
        for i in 0..n {
            emid = mac16_16(emid, x[i], x[i]);
            eside = mac16_16(eside, y[i], y[i]);
        }
    }

    #[cfg(not(feature = "fixed-point"))]
    {
        let mid = celt_sqrt(emid);
        let side = celt_sqrt(eside);
        (0.5 + 16384.0 * 0.63662 * (side as f64).atan2(mid as f64)) as c_int
    }
    #[cfg(feature = "fixed-point")]
    {
        let mid = celt_sqrt(emid) as i16;
        let side = celt_sqrt(eside) as i16;
        // 0.63662 = 2/pi, QCONST16(0.63662, 15) = 20861
        mult16_16_q15(20861, celt_atan2p(side, mid)) as c_int
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extract_collapse_mask_single_block() {
        assert_eq!(extract_collapse_mask(&[0, 0, 1], 3, 1), 1);
    }

    #[test]
    fn test_extract_collapse_mask_two_blocks() {
        // 4 elements, 2 blocks of 2
        // block 0: [0, 0] -> bit 0 = 0
        // block 1: [1, 0] -> bit 1 = 1
        assert_eq!(extract_collapse_mask(&[0, 0, 1, 0], 4, 2), 0b10);
    }

    #[test]
    fn test_extract_collapse_mask_all_nonzero() {
        assert_eq!(extract_collapse_mask(&[1, 2, 3, 4], 4, 2), 0b11);
    }
}
