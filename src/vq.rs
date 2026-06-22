// Translated from c/celt/vq.c (RFC 6716).
//
// Vector quantisation of the residual signal. The decoder path uses
// alg_unquant() to decode pulse vectors from the bitstream and produce
// normalised MDCT coefficients for each band. renormalise_vector() is a
// shared helper called from bands.c.
//
// exp_rotation applies a spreading transform that distributes spectral
// energy more evenly, improving perceptual quality. It uses Givens
// rotations with cos/sin derived from a gain function of K/N.
//
// See RFC 6716 Section 4.3.4.

use crate::arch::*;
use crate::cwrs::decode_pulses;
use crate::entcode::ec_dec;
use crate::util::OrPanic;

// -- Static helpers --

/// Apply Givens rotations along a single stride.
/// Used by exp_rotation to spread energy across coefficients.
///
/// In-place rotation of the pair `(x[i], x[i+stride])`: a sequential
/// recurrence (consecutive `i` overlap), so it has no iterator form. The
/// loop ranges keep `i+stride < len` in both passes.
#[allow(clippy::indexing_slicing)]
fn exp_rotation1(x: &mut [CeltNorm], stride: usize, c: Val, s: Val) {
    let len = x.len();
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
fn exp_rotation(x: &mut [CeltNorm], dir: i32, stride: usize, k: i32, spread: i32) {
    const SPREAD_FACTOR: [i32; 3] = [15, 10, 5];

    let len = x.len();
    if 2 * k >= len as i32 || spread == SPREAD_NONE {
        return;
    }
    let factor = SPREAD_FACTOR.get((spread - 1) as usize).copied().or_panic(spread - 1);

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

    // CELT guarantees len is a multiple of stride, so this yields exactly
    // `stride` blocks of `block_len` (>= 1) samples.
    let block_len = len / stride;
    for slice in x.chunks_exact_mut(block_len).take(stride) {
        if dir < 0 {
            if stride2 > 0 {
                exp_rotation1(slice, stride2, s, c);
            }
            exp_rotation1(slice, 1, c, s);
        } else {
            let neg_s = neg16(s);
            exp_rotation1(slice, 1, c, neg_s);
            if stride2 > 0 {
                let neg_c = neg16(c);
                exp_rotation1(slice, stride2, s, neg_c);
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
fn normalise_residual(iy: &[i32], x: &mut [CeltNorm], ryy: Wal, gain: Val) {
    // C: k = celt_ilog2(Ryy)>>1;   (fixed only)
    //    t = VSHR32(Ryy, 2*(k-7));
    //    g = MULT16_16_P15(celt_rsqrt_norm(t), gain);
    //    X[i] = EXTRACT16(PSHR32(MULT16_16(g, iy[i]), k+1));
    #[cfg(feature = "fixed-point")]
    {
        let k = celt_ilog2(ryy) >> 1;
        let t = vshr32(ryy, 2 * (k as i32 - 7));
        let g = mult16_16_p15(celt_rsqrt_norm(t), gain) as i16;
        for (xi, &iyi) in x.iter_mut().zip(iy) {
            *xi = extract16(pshr32(mult16_16(g, iyi as i16), k as i32 + 1));
        }
    }
    #[cfg(not(feature = "fixed-point"))]
    {
        let t = ryy;
        let g = celt_rsqrt_norm(t) * gain;
        for (xi, &iyi) in x.iter_mut().zip(iy) {
            *xi = g * iyi as f32;
        }
    }
}

/// Compute per-block collapse mask from the decoded pulse vector.
/// Each bit i indicates whether block i received any non-zero pulses.
fn extract_collapse_mask(iy: &[i32], b: usize) -> u32 {
    if b <= 1 {
        return 1;
    }
    // CELT guarantees the band size iy.len() is a multiple of the block count
    // b, so iy splits into exactly b blocks of n0 (>= 1) samples.
    let n0 = iy.len() / b;
    let mut collapse_mask = 0u32;
    for (i, block) in iy.chunks_exact(n0).take(b).enumerate() {
        if block.iter().any(|&v| v != 0) {
            collapse_mask |= 1 << i;
        }
    }
    collapse_mask
}

// -- Public decoder functions --

/// Decode pulse vector and combine with normalisation to produce the
/// final normalised signal coefficients for one band.
pub fn alg_unquant(x: &mut [CeltNorm], k: i32, spread: i32, b: i32, dec: &mut ec_dec, gain: Val) -> u32 {
    debug_assert!(k > 0, "alg_unquant() needs at least one pulse");
    let n = x.len();
    debug_assert!(n > 1, "alg_unquant() needs at least two dimensions");

    // Largest CELT band: (1<<max_lm)=8 * largest eBand delta (100-78=22) = 176.
    const MAX_BAND_SIZE: usize = 176;
    let mut iy = [0i32; MAX_BAND_SIZE];
    let iy = iy.get_mut(..n).or_panic(n);

    // Decode pulse vector from bitstream
    decode_pulses(iy, k, dec);

    // Compute sum of squares: Ryy = sum(iy[i]^2)
    let mut ryy: Wal = 0 as Wal;
    for &v in &*iy {
        ryy = mac16_16(ryy, v as Val, v as Val);
    }

    normalise_residual(iy, x, ryy, gain);
    exp_rotation(x, -1, b as usize, k, spread);
    extract_collapse_mask(iy, b as usize)
}

/// Renormalise a coefficient vector to have the given gain (norm).
///
pub fn renormalise_vector(x: &mut [CeltNorm], gain: Val) {
    let mut e: Wal = EPSILON as Wal;
    for &xi in x.iter() {
        e = mac16_16(e, xi, xi);
    }

    #[cfg(feature = "fixed-point")]
    {
        let k = celt_ilog2(e) >> 1;
        let t = vshr32(e, 2 * (k as i32 - 7));
        let g = mult16_16_p15(celt_rsqrt_norm(t), gain) as i16;
        for xi in x.iter_mut() {
            *xi = extract16(pshr32(mult16_16(g, *xi), k as i32 + 1));
        }
    }
    #[cfg(not(feature = "fixed-point"))]
    {
        let g = celt_rsqrt_norm(e) * gain;
        for xi in x.iter_mut() {
            *xi = g * *xi;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_extract_collapse_mask_single_block() {
        assert_eq!(extract_collapse_mask(&[0, 0, 1], 1), 1);
    }

    #[test]
    fn test_extract_collapse_mask_two_blocks() {
        // 4 elements, 2 blocks of 2
        // block 0: [0, 0] -> bit 0 = 0
        // block 1: [1, 0] -> bit 1 = 1
        assert_eq!(extract_collapse_mask(&[0, 0, 1, 0], 2), 0b10);
    }

    #[test]
    fn test_extract_collapse_mask_all_nonzero() {
        assert_eq!(extract_collapse_mask(&[1, 2, 3, 4], 2), 0b11);
    }
}
