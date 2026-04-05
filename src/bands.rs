// Translated from c/celt/bands.c (RFC 6716).
//
// CELT band processing functions: energy computation, normalisation,
// stereo handling, and spectral folding/unfolding.
//
// Functions are translated incrementally, innermost helpers first.

use std::os::raw::c_int;

use crate::arch::{CeltEner, CeltNorm, CeltSig, EPSILON, OpusVal16, OpusVal32, Q15ONE};
use crate::arch::{
    add16, celt_rsqrt_norm, celt_sqrt, div32_16, extend32, extract16, mac16_16, min16, mult16_16, mult16_16_q14, mult16_16_q15,
    mult16_32_q15, pshr32, shl32, shr32, sub16, vshr32,
};
#[cfg(feature = "fixed-point")]
use crate::arch::{celt_exp2, celt_ilog2, celt_zlog2, shl16, shr16};
#[cfg(not(feature = "fixed-point"))]
use crate::arch::{celt_exp2, celt_rsqrt};
use crate::entcode::BITRES;
use crate::modes::CELTMode;
use crate::vq::renormalise_vector;

/// Linear congruential generator used for pseudo-random noise injection
/// in the CELT decoder (PLC comfort noise, anti-collapse, spectral folding).
/// Constants match Numerical Recipes / Knuth MMIX.
#[unsafe(no_mangle)]
pub extern "C" fn celt_lcg_rand(seed: u32) -> u32 {
    seed.wrapping_mul(1664525).wrapping_add(1013904223)
}

/// De-normalise band energies to produce synthesis output.
///
/// Scales each unit-energy band by its decoded energy envelope, producing
/// the frequency-domain signal that feeds into the inverse MDCT.
/// Bands beyond `end` are zeroed (above the Nyquist for the coded bandwidth).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn denormalise_bands(
    m: *const CELTMode,
    x: *const CeltNorm,
    freq: *mut CeltSig,
    band_e: *const CeltEner,
    end: c_int,
    c_channels: c_int,
    m_factor: c_int,
) {
    unsafe {
        let mode = &*m;
        let ebands = mode.ebands;
        let n = m_factor * mode.short_mdct_size;

        let mut c = 0;
        loop {
            let f_base = freq.add((c * n) as usize);
            let x_base = x.add((c * n) as usize);
            for i in 0..end as usize {
                let g = shr32(*band_e.add(i + (c as usize) * mode.nb_ebands as usize), 1);
                let j_start = (m_factor * (*ebands.add(i) as c_int)) as usize;
                let band_end = (m_factor * (*ebands.add(i + 1) as c_int)) as usize;
                for j in j_start..band_end {
                    *f_base.add(j) = shl32(mult16_32_q15(*x_base.add(j), g), 2);
                }
            }

            // Zero above the coded bandwidth
            let zero_start = (m_factor * (*ebands.add(end as usize) as c_int)) as usize;
            for i in zero_start..n as usize {
                *f_base.add(i) = 0 as CeltSig;
            }

            c += 1;
            if c >= c_channels {
                break;
            }
        }
    }
}

/// Single-level Haar wavelet transform on interleaved sub-vectors.
///
/// Used inside quant_band to (de)interleave time-frequency coefficients
/// for the Hadamard rearrangement that maps between time-domain short
/// blocks and the band's frequency layout. Applies the unnormalised
/// Haar butterfly (scaled by 1/sqrt(2)) in-place.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn haar1(x: *mut CeltNorm, n0: c_int, stride: c_int) {
    unsafe {
        let n0 = n0 >> 1;
        for i in 0..stride {
            for j in 0..n0 {
                let idx0 = (stride * 2 * j + i) as usize;
                let idx1 = (stride * (2 * j + 1) + i) as usize;
                let tmp1 = mult16_16_q15(qconst16(0.70710678, 15), *x.add(idx0)) as CeltNorm;
                let tmp2 = mult16_16_q15(qconst16(0.70710678, 15), *x.add(idx1)) as CeltNorm;
                *x.add(idx0) = tmp1 + tmp2;
                *x.add(idx1) = tmp1 - tmp2;
            }
        }
    }
}

/// Intensity stereo: rotate (X, Y) onto X using the energy ratio.
///
/// Computes the left/right energy ratio for the band, derives mixing
/// coefficients a1 and a2, and replaces X with the intensity-coded
/// mono signal. Y is not updated (side is discarded at this point).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn intensity_stereo(
    m: *const CELTMode,
    x: *mut CeltNorm,
    y: *const CeltNorm,
    band_e: *const CeltEner,
    band_id: c_int,
    n: c_int,
) {
    unsafe {
        let mode = &*m;
        let i = band_id as usize;
        let nb = mode.nb_ebands as usize;

        #[cfg(feature = "fixed-point")]
        let shift = (celt_zlog2((*band_e.add(i)).max(*band_e.add(i + nb))) - 13) as i32;
        #[cfg(not(feature = "fixed-point"))]
        let shift: i32 = 0;

        let left = vshr32(*band_e.add(i), shift);
        let right = vshr32(*band_e.add(i + nb), shift);
        let norm = EPSILON
            + celt_sqrt(
                EPSILON + mult16_16(left as OpusVal16, left as OpusVal16) + mult16_16(right as OpusVal16, right as OpusVal16),
            );
        let a1 = div32_16(shl32(extend32(left as OpusVal16), 14), norm as OpusVal16);
        let a2 = div32_16(shl32(extend32(right as OpusVal16), 14), norm as OpusVal16);
        for j in 0..n as usize {
            let l = *x.add(j);
            let r = *y.add(j);
            *x.add(j) = (mult16_16_q14(a1 as OpusVal16, l) + mult16_16_q14(a2 as OpusVal16, r)) as CeltNorm;
        }
    }
}

/// Stereo split: convert (L, R) to (mid, side) using Haar-like transform.
///
/// X becomes (L+R)/sqrt(2), Y becomes (R-L)/sqrt(2).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn stereo_split(x: *mut CeltNorm, y: *mut CeltNorm, n: c_int) {
    unsafe {
        for j in 0..n as usize {
            let l = mult16_16_q15(qconst16(0.70710678, 15), *x.add(j)) as CeltNorm;
            let r = mult16_16_q15(qconst16(0.70710678, 15), *y.add(j)) as CeltNorm;
            *x.add(j) = l + r;
            *y.add(j) = r - l;
        }
    }
}

#[cfg(not(feature = "fixed-point"))]
fn qconst32(x: f32, _bits: i32) -> f32 {
    x
}
#[cfg(feature = "fixed-point")]
fn qconst32(x: f32, bits: i32) -> i32 {
    (x * ((1i64 << bits) as f32) + 0.5) as i32
}

/// Stereo merge: reconstruct (L, R) from (mid, side) after decoding.
///
/// Uses the energy invariance property to compute proper L/R gains
/// from the decoded mid and side signals. Falls back to copying mid
/// to both channels if the energy is near zero.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn stereo_merge(x: *mut CeltNorm, y: *mut CeltNorm, mid: OpusVal16, n: c_int) {
    unsafe {
        let mut xp: OpusVal32 = 0 as OpusVal32;
        let mut side: OpusVal32 = 0 as OpusVal32;

        for j in 0..n as usize {
            xp = mac16_16(xp, *x.add(j), *y.add(j));
            side = mac16_16(side, *y.add(j), *y.add(j));
        }
        xp = mult16_32_q15(mid, xp);
        let mid2 = shr32(mid as OpusVal32, 1) as OpusVal16;
        let el = mult16_16(mid2, mid2) + side - (2 as OpusVal32) * xp;
        let er = mult16_16(mid2, mid2) + side + (2 as OpusVal32) * xp;
        if er < qconst32(6e-4, 28) || el < qconst32(6e-4, 28) {
            for j in 0..n as usize {
                *y.add(j) = *x.add(j);
            }
            return;
        }

        #[cfg(feature = "fixed-point")]
        let (kl, kr): (i32, i32);
        #[cfg(not(feature = "fixed-point"))]
        let (kl, kr): (i32, i32) = (0, 0);

        #[cfg(feature = "fixed-point")]
        {
            kl = (celt_ilog2(el) >> 1) as i32;
            kr = (celt_ilog2(er) >> 1) as i32;
        }

        let t = vshr32(el, (kl - 7) << 1);
        let lgain = celt_rsqrt_norm(t);
        let t = vshr32(er, (kr - 7) << 1);
        let rgain = celt_rsqrt_norm(t);

        #[cfg(feature = "fixed-point")]
        let (kl, kr) = (kl.max(7), kr.max(7));

        for j in 0..n as usize {
            let l = mult16_16_q15(mid, *x.add(j));
            let r = *y.add(j);
            *x.add(j) = extract16(pshr32(mult16_16(lgain as OpusVal16, sub16(l as OpusVal16, r)), kl + 1));
            *y.add(j) = extract16(pshr32(mult16_16(rgain as OpusVal16, add16(l as OpusVal16, r)), kr + 1));
        }
    }
}

/// Bit-reversed Gray code table for Hadamard ordering.
/// Lines are for N=2, 4, 8, 16; DC is placed at the end.
const ORDERY_TABLE: [c_int; 30] =
    [1, 0, 3, 0, 2, 1, 7, 0, 4, 3, 6, 1, 5, 2, 15, 0, 8, 7, 12, 3, 11, 4, 14, 1, 9, 6, 13, 2, 10, 5];

/// Deinterleave sub-vectors with optional Hadamard reordering.
///
/// Rearranges `X` from interleaved layout (stride-interleaved short blocks)
/// into contiguous sub-vectors, optionally applying the ordery Hadamard
/// permutation. Used before recursive band splitting in quant_band.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn deinterleave_hadamard(x: *mut CeltNorm, n0: c_int, stride: c_int, hadamard: c_int) {
    unsafe {
        let n = n0 * stride;
        let mut tmp = vec![0 as CeltNorm; n as usize];
        if hadamard != 0 {
            let ordery = &ORDERY_TABLE[(stride - 2) as usize..];
            for i in 0..stride as usize {
                for j in 0..n0 as usize {
                    tmp[ordery[i] as usize * n0 as usize + j] = *x.add(j * stride as usize + i);
                }
            }
        } else {
            for i in 0..stride as usize {
                for j in 0..n0 as usize {
                    tmp[i * n0 as usize + j] = *x.add(j * stride as usize + i);
                }
            }
        }
        for j in 0..n as usize {
            *x.add(j) = tmp[j];
        }
    }
}

/// Interleave sub-vectors with optional Hadamard reordering.
///
/// Inverse of deinterleave_hadamard: rearranges contiguous sub-vectors
/// back into stride-interleaved layout, with optional ordery Hadamard
/// permutation. Used after recursive band reconstruction in quant_band.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn interleave_hadamard(x: *mut CeltNorm, n0: c_int, stride: c_int, hadamard: c_int) {
    unsafe {
        let n = n0 * stride;
        let mut tmp = vec![0 as CeltNorm; n as usize];
        if hadamard != 0 {
            let ordery = &ORDERY_TABLE[(stride - 2) as usize..];
            for i in 0..stride as usize {
                for j in 0..n0 as usize {
                    tmp[j * stride as usize + i] = *x.add(ordery[i] as usize * n0 as usize + j);
                }
            }
        } else {
            for i in 0..stride as usize {
                for j in 0..n0 as usize {
                    tmp[j * stride as usize + i] = *x.add(i * n0 as usize + j);
                }
            }
        }
        for j in 0..n as usize {
            *x.add(j) = tmp[j];
        }
    }
}

/// Spread mode constants (from bands.h).
pub const SPREAD_NONE: c_int = 0;
pub const SPREAD_LIGHT: c_int = 1;
pub const SPREAD_NORMAL: c_int = 2;
pub const SPREAD_AGGRESSIVE: c_int = 3;

/// Compute the number of quantisation levels for a band given the
/// available bits, band size, and pulse cap. Used by quant_band to
/// decide how finely to quantise the angular parameter theta.
#[unsafe(no_mangle)]
pub extern "C" fn compute_qn(n: c_int, b: c_int, offset: c_int, pulse_cap: c_int, stereo: c_int) -> c_int {
    const EXP2_TABLE8: [i16; 8] = [16384, 17866, 19483, 21247, 23170, 25267, 27554, 30048];
    let mut n2 = 2 * n - 1;
    if stereo != 0 && n == 2 {
        n2 -= 1;
    }
    let qb = (b - pulse_cap - (4 << BITRES)).min((b + n2 * offset) / n2);
    let qb = qb.min(8 << BITRES as c_int);

    if qb < (1 << BITRES >> 1) {
        1
    } else {
        let qn = (EXP2_TABLE8[(qb as usize) & 0x7] >> (14 - (qb >> BITRES as c_int))) as c_int;
        (qn + 1) >> 1 << 1
    }
}

#[cfg(not(feature = "fixed-point"))]
fn qconst16(x: f32, _bits: i32) -> f32 {
    x
}
#[cfg(feature = "fixed-point")]
fn qconst16(x: f32, bits: i32) -> i16 {
    (x * ((1 << bits) as f32) + 0.5) as i16
}

/// Prevent energy collapse for transients with multiple short MDCTs.
///
/// When a band has no pulses allocated (collapse_masks bit clear), fills
/// it with shaped pseudo-random noise at a level derived from the energy
/// difference between the current and previous frames, then renormalises.
/// This avoids audible "holes" in transient signals decoded at low bitrate.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn anti_collapse(
    m: *const CELTMode,
    x_: *mut CeltNorm,
    collapse_masks: *const u8,
    lm: c_int,
    c_channels: c_int,
    size: c_int,
    start: c_int,
    end: c_int,
    log_e: *const OpusVal16,
    prev1log_e: *const OpusVal16,
    prev2log_e: *const OpusVal16,
    pulses: *const c_int,
    seed: u32,
) {
    unsafe {
        let mode = &*m;
        let mut seed = seed;

        for i in start..end {
            let i = i as usize;
            let n0 = (*mode.ebands.add(i + 1) - *mode.ebands.add(i)) as c_int;
            // depth in 1/8 bits
            let depth = (1 + *pulses.add(i)) / (((*mode.ebands.add(i + 1) - *mode.ebands.add(i)) as c_int) << lm);

            #[cfg(feature = "fixed-point")]
            let (thresh, sqrt_1, shift): (OpusVal16, OpusVal16, i32);
            #[cfg(not(feature = "fixed-point"))]
            let (thresh, sqrt_1): (OpusVal16, OpusVal16);

            #[cfg(feature = "fixed-point")]
            {
                thresh = mult16_32_q15(
                    qconst16(0.5, 15),
                    (shr32(celt_exp2(-shl16(depth as i16, 10 - BITRES as i32)), 1)).min(32767),
                ) as OpusVal16;
                let t: OpusVal32 = n0 << lm;
                let sh = (celt_ilog2(t) >> 1) as i32;
                let t2 = shl32(t, (7 - sh) << 1);
                sqrt_1 = celt_rsqrt_norm(t2);
                shift = sh;
            }
            #[cfg(not(feature = "fixed-point"))]
            {
                thresh = 0.5 * celt_exp2(-0.125 * depth as f32);
                sqrt_1 = celt_rsqrt((n0 << lm) as f32);
            }

            let mut c = 0;
            loop {
                let nb_ebands = mode.nb_ebands as usize;
                let mut prev1 = *prev1log_e.add(c as usize * nb_ebands + i);
                let mut prev2 = *prev2log_e.add(c as usize * nb_ebands + i);
                if c_channels == 1 {
                    prev1 = prev1.max(*prev1log_e.add(nb_ebands + i));
                    prev2 = prev2.max(*prev2log_e.add(nb_ebands + i));
                }
                let ediff = extend32(*log_e.add(c as usize * nb_ebands + i)) - extend32(min16(prev1, prev2));
                let ediff = ediff.max(0 as OpusVal32);

                #[cfg(feature = "fixed-point")]
                let r: OpusVal16;
                #[cfg(not(feature = "fixed-point"))]
                let r: OpusVal16;

                #[cfg(feature = "fixed-point")]
                {
                    let mut rv: i16;
                    if ediff < 16384 {
                        rv = 2 * (shr32(celt_exp2(-extract16(ediff)), 1) as i16).min(16383);
                    } else {
                        rv = 0;
                    }
                    if lm == 3 {
                        rv = mult16_16_q14(23170, (rv as i32).min(23169) as i16) as i16;
                    }
                    rv = shr16(min16(thresh, rv), 1);
                    r = shr32(mult16_16_q15(sqrt_1, rv) as i32, shift) as i16;
                }
                #[cfg(not(feature = "fixed-point"))]
                {
                    let mut rv = 2.0 * celt_exp2(-ediff);
                    if lm == 3 {
                        rv *= 1.41421356;
                    }
                    rv = rv.min(thresh);
                    r = rv * sqrt_1;
                }

                let x_ptr = x_.add(c as usize * size as usize + ((*mode.ebands.add(i) as c_int) << lm) as usize);
                let mut renormalize = 0;
                for k in 0..1 << lm {
                    // Detect collapse
                    if (*collapse_masks.add(i * c_channels as usize + c as usize) & (1 << k)) == 0 {
                        // Fill with noise
                        for j in 0..n0 {
                            seed = celt_lcg_rand(seed);
                            *x_ptr.add(((j << lm) + k) as usize) = if seed & 0x8000 != 0 { r } else { -r };
                        }
                        renormalize = 1;
                    }
                }
                // We just added some energy, so we need to renormalise
                if renormalize != 0 {
                    renormalise_vector(x_ptr, n0 << lm, Q15ONE);
                }

                c += 1;
                if c >= c_channels {
                    break;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lcg_rand_known_values() {
        // seed=0 → 1013904223
        assert_eq!(celt_lcg_rand(0), 1013904223);
        // seed=1 → 1664525 + 1013904223 = 1015568748
        assert_eq!(celt_lcg_rand(1), 1015568748);
        // Two iterations from 0
        let s1 = celt_lcg_rand(0);
        let s2 = celt_lcg_rand(s1);
        assert_eq!(s2, 1196435762);
    }

    #[test]
    fn test_lcg_rand_wrapping() {
        // Large seed should wrap without panic
        let _ = celt_lcg_rand(u32::MAX);
    }
}
