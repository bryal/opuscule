// Translated from c/celt/bands.c (RFC 6716).
//
// CELT band processing functions: energy computation, normalisation,
// stereo handling, and spectral folding/unfolding.
//
// Functions are translated incrementally, innermost helpers first.

use std::os::raw::c_int;

use crate::arch::{CeltEner, CeltNorm, CeltSig, EPSILON, NORM_SCALING, OpusVal16, OpusVal32, Q15ONE};
use crate::arch::{
    add16, celt_rsqrt_norm, celt_sqrt, div32_16, extend32, extract16, half32, mac16_16, min16, mult16_16, mult16_16_p15,
    mult16_16_q14, mult16_16_q15, mult16_32_q15, pshr32, shl32, shr16, shr32, sub16, vshr32,
};
#[cfg(feature = "fixed-point")]
use crate::arch::{celt_exp2, celt_ilog2, celt_zlog2, shl16};
#[cfg(not(feature = "fixed-point"))]
use crate::arch::{celt_exp2, celt_rsqrt};
use crate::entcode::{BITRES, ec_ctx, ec_ilog, ec_tell_frac};
use crate::entdec::{ec_dec_bit_logp, ec_dec_bits, ec_dec_uint, ec_dec_update, ec_decode};
use crate::mathops::{frac_mul16, isqrt32};
use crate::modes::CELTMode;
use crate::rate::{bits2pulses, get_pulses, pulses2bits};
use crate::vq::{alg_unquant, renormalise_vector};

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

const QTHETA_OFFSET: c_int = 4;
const QTHETA_OFFSET_TWOPHASE: c_int = 16;

#[cfg(not(feature = "fixed-point"))]
fn qconst16(x: f32, _bits: i32) -> f32 {
    x
}
#[cfg(feature = "fixed-point")]
fn qconst16(x: f32, bits: i32) -> i16 {
    (x * ((1 << bits) as f32) + 0.5) as i16
}

/// Bit-exact cosine approximation. Important for deterministic bit allocation.
fn bitexact_cos(x: i16) -> i16 {
    let tmp = (4096 + (x as i32) * (x as i32)) >> 13;
    let x2 = tmp as i16;
    let x2 =
        (32767 - x2 as i32 + frac_mul16(x2, -7651 + frac_mul16(x2, 8277 + frac_mul16(-626, x2) as i16) as i16) as i32) as i16;
    1 + x2
}

/// Bit-exact log2(tan) approximation for mid/side bit allocation.
fn bitexact_log2tan(isin: c_int, icos: c_int) -> c_int {
    let ls = ec_ilog(isin as u32);
    let lc = ec_ilog(icos as u32);
    let isin = isin << (15 - ls);
    let icos = icos << (15 - lc);
    (ls - lc) * (1 << 11) + frac_mul16(isin as i16, frac_mul16(isin as i16, -2597) as i16 + 7932) as c_int
        - frac_mul16(icos as i16, frac_mul16(icos as i16, -2597) as i16 + 7932) as c_int
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

/// Decode (or encode) a single band, possibly with stereo and/or recursive splitting.
///
/// This is the core of the CELT spectral quantisation. It handles:
/// - Single-sample bands (N=1): just a sign bit
/// - Band recombining and time-frequency resolution changes
/// - Stereo: intensity coding, mid/side splitting, N=2 special case
/// - Recursive binary splitting when bits exceed the cache limit
/// - Leaf decoding via alg_unquant (PVQ) or noise/fold fill when q=0
/// - Resynthesis: stereo merge, Hadamard interleaving, lowband output
///
/// Returns the collapse mask (which sub-blocks are non-zero).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn quant_band(
    encode: c_int,
    m: *const CELTMode,
    i: c_int,
    x: *mut CeltNorm,
    y_in: *mut CeltNorm,
    n_in: c_int,
    b_in: c_int,
    spread: c_int,
    b_blocks_in: c_int,
    intensity: c_int,
    tf_change_in: c_int,
    lowband_in: *mut CeltNorm,
    ec: *mut ec_ctx,
    remaining_bits: *mut i32,
    lm_in: c_int,
    lowband_out: *mut CeltNorm,
    band_e: *const CeltEner,
    level: c_int,
    seed: *mut u32,
    gain: OpusVal16,
    lowband_scratch: *mut CeltNorm,
    fill_in: c_int,
) -> u32 {
    unsafe {
        let mode = &*m;
        let resynth = encode == 0;

        let mut n = n_in;
        let mut b = b_in;
        let mut y = y_in;
        let mut b_blocks = b_blocks_in;
        let mut tf_change = tf_change_in;
        let mut lowband = lowband_in;
        let mut lm = lm_in;
        let mut fill = fill_in;

        let long_blocks = b_blocks == 1;
        let n0 = n;
        let mut n_b = n / b_blocks;
        let n_b0;
        let mut b0 = b_blocks;
        let mut time_divide = 0;
        let mut recombine = 0;
        let mut inv = 0;
        let mut mid: OpusVal16 = 0 as OpusVal16;
        let mut side: OpusVal16 = 0 as OpusVal16;
        let mut cm: u32 = 0;
        let imid;
        let iside;

        let stereo = (!y.is_null()) as c_int;
        let mut split = stereo;

        // Special case for one sample
        if n == 1 {
            let mut x_ptr = x;
            let mut c = 0;
            loop {
                let mut sign = 0u32;
                if *remaining_bits >= 1 << BITRES {
                    sign = ec_dec_bits(ec, 1);
                    *remaining_bits -= 1 << BITRES as c_int;
                    b -= 1 << BITRES as c_int;
                }
                if resynth {
                    *x_ptr = if sign != 0 { -NORM_SCALING } else { NORM_SCALING };
                }
                x_ptr = y;
                c += 1;
                if c >= 1 + stereo {
                    break;
                }
            }
            if !lowband_out.is_null() {
                *lowband_out = shr16(*x, 4);
            }
            return 1;
        }

        if stereo == 0 && level == 0 {
            if tf_change > 0 {
                recombine = tf_change;
            }

            if !lowband.is_null() && (recombine != 0 || ((n_b & 1) == 0 && tf_change < 0) || b0 > 1) {
                for j in 0..n as usize {
                    *lowband_scratch.add(j) = *lowband.add(j);
                }
                lowband = lowband_scratch;
            }

            for k in 0..recombine {
                const BIT_INTERLEAVE_TABLE: [u8; 16] = [0, 1, 1, 1, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3];
                if !lowband.is_null() {
                    haar1(lowband, n >> k, 1 << k);
                }
                fill = BIT_INTERLEAVE_TABLE[(fill & 0xF) as usize] as c_int
                    | (BIT_INTERLEAVE_TABLE[(fill >> 4) as usize] as c_int) << 2;
            }
            b_blocks >>= recombine;
            n_b <<= recombine;

            // Increasing the time resolution
            while (n_b & 1) == 0 && tf_change < 0 {
                if !lowband.is_null() {
                    haar1(lowband, n_b, b_blocks);
                }
                fill |= fill << b_blocks;
                b_blocks <<= 1;
                n_b >>= 1;
                time_divide += 1;
                tf_change += 1;
            }
            b0 = b_blocks;

            // Reorganize the samples in time order instead of frequency order
            if b0 > 1 {
                if !lowband.is_null() {
                    deinterleave_hadamard(lowband, n_b >> recombine, b0 << recombine, long_blocks as c_int);
                }
            }
        }
        n_b0 = n_b;

        // If we need 1.5 more bit than we can produce, split the band in two.
        let cache = mode.cache.bits.add((*mode.cache.index.add(((lm + 1) * mode.nb_ebands + i) as usize)) as usize);
        if stereo == 0 && lm != -1 && b > (*cache.add(*cache as usize)) as c_int + 12 && n > 2 {
            n >>= 1;
            y = x.add(n as usize);
            split = 1;
            lm -= 1;
            if b_blocks == 1 {
                fill = (fill & 1) | (fill << 1);
            }
            b_blocks = (b_blocks + 1) >> 1;
        }

        if split != 0 {
            let mut itheta: c_int = 0;
            let qalloc;
            let mut mbits;
            let mut sbits;
            let mut delta;
            let pulse_cap;
            let offset;
            let orig_fill;

            // Decide on the resolution to give to the split parameter theta
            pulse_cap = (*mode.log_n.add(i as usize)) as c_int + lm * (1 << BITRES as c_int);
            offset = (pulse_cap >> 1) - if stereo != 0 && n == 2 { QTHETA_OFFSET_TWOPHASE } else { QTHETA_OFFSET };
            let qn = compute_qn(n, b, offset, pulse_cap, stereo);
            let qn_val = if stereo != 0 && i >= intensity { 1 } else { qn };

            let tell = ec_tell_frac(ec);
            if qn_val != 1 {
                // Entropy coding of the angle
                if stereo != 0 && n > 2 {
                    let p0 = 3i32;
                    let x0 = qn_val / 2;
                    let ft = (p0 * (x0 + 1) + x0) as u32;
                    let fs = ec_decode(ec, ft) as i32;
                    let x_val;
                    if fs < (x0 + 1) * p0 {
                        x_val = fs / p0;
                    } else {
                        x_val = x0 + 1 + (fs - (x0 + 1) * p0);
                    }
                    let fl = if x_val <= x0 { p0 * x_val } else { (x_val - 1 - x0) + (x0 + 1) * p0 };
                    let fh = if x_val <= x0 { p0 * (x_val + 1) } else { (x_val - x0) + (x0 + 1) * p0 };
                    ec_dec_update(ec, fl as u32, fh as u32, ft);
                    itheta = x_val;
                } else if b0 > 1 || stereo != 0 {
                    // Uniform pdf
                    itheta = ec_dec_uint(ec, qn_val as u32 + 1) as c_int;
                } else {
                    // Triangular pdf
                    let ft = (((qn_val >> 1) + 1) * ((qn_val >> 1) + 1)) as u32;
                    let fm = ec_decode(ec, ft);

                    if (fm as i32) < ((qn_val >> 1) * ((qn_val >> 1) + 1) >> 1) {
                        itheta = ((isqrt32(8 * fm + 1) as i32 - 1) >> 1) as c_int;
                        let fs = itheta + 1;
                        let fl = itheta * (itheta + 1) >> 1;
                        ec_dec_update(ec, fl as u32, (fl + fs) as u32, ft);
                    } else {
                        itheta = ((2 * (qn_val + 1) - isqrt32(8 * (ft - fm - 1) + 1) as i32) >> 1) as c_int;
                        let fs = qn_val + 1 - itheta;
                        let fl = ft as i32 - ((qn_val + 1 - itheta) * (qn_val + 2 - itheta) >> 1);
                        ec_dec_update(ec, fl as u32, (fl + fs) as u32, ft);
                    }
                }
                itheta = (itheta as i64 * 16384 / qn_val as i64) as c_int;
            } else if stereo != 0 {
                // qn==1, stereo
                if b > 2 << BITRES as c_int && *remaining_bits > 2 << BITRES as c_int {
                    inv = ec_dec_bit_logp(ec, 2);
                } else {
                    inv = 0;
                }
                itheta = 0;
            }
            qalloc = ec_tell_frac(ec) as c_int - tell as c_int;
            b -= qalloc;

            orig_fill = fill;
            if itheta == 0 {
                imid = 32767;
                iside = 0;
                fill &= (1 << b_blocks) - 1;
                delta = -16384;
            } else if itheta == 16384 {
                imid = 0;
                iside = 32767;
                fill &= ((1 << b_blocks) - 1) << b_blocks;
                delta = 16384;
            } else {
                imid = bitexact_cos(itheta as i16) as c_int;
                iside = bitexact_cos((16384 - itheta) as i16) as c_int;
                delta = frac_mul16(((n - 1) << 7) as i16, bitexact_log2tan(iside, imid) as i16) as c_int;
            }

            #[cfg(feature = "fixed-point")]
            {
                mid = imid as OpusVal16;
                side = iside as OpusVal16;
            }
            #[cfg(not(feature = "fixed-point"))]
            {
                mid = (1.0 / 32768.0) * imid as f32;
                side = (1.0 / 32768.0) * iside as f32;
            }

            // Special case for N=2 stereo
            if n == 2 && stereo != 0 {
                let mut sign = 0i32;
                mbits = b;
                sbits = if itheta != 0 && itheta != 16384 { 1 << BITRES as c_int } else { 0 };
                let mbits = mbits - sbits;
                let c_side = (itheta > 8192) as c_int;
                *remaining_bits -= qalloc + sbits;

                let x2 = if c_side != 0 { y } else { x };
                let y2 = if c_side != 0 { x } else { y };
                if sbits != 0 {
                    sign = ec_dec_bits(ec, 1) as i32;
                }
                sign = 1 - 2 * sign;
                cm = quant_band(
                    encode,
                    m,
                    i,
                    x2,
                    std::ptr::null_mut(),
                    n,
                    mbits,
                    spread,
                    b_blocks,
                    intensity,
                    tf_change,
                    lowband,
                    ec,
                    remaining_bits,
                    lm,
                    lowband_out,
                    std::ptr::null(),
                    level,
                    seed,
                    gain,
                    lowband_scratch,
                    orig_fill,
                );
                *y2.add(0) = -(sign as CeltNorm) * *x2.add(1);
                *y2.add(1) = (sign as CeltNorm) * *x2.add(0);
                if resynth {
                    *x.add(0) = mult16_16_q15(mid, *x.add(0)) as CeltNorm;
                    *x.add(1) = mult16_16_q15(mid, *x.add(1)) as CeltNorm;
                    *y.add(0) = mult16_16_q15(side, *y.add(0)) as CeltNorm;
                    *y.add(1) = mult16_16_q15(side, *y.add(1)) as CeltNorm;
                    let tmp = *x.add(0);
                    *x.add(0) = sub16(tmp, *y.add(0));
                    *y.add(0) = add16(tmp, *y.add(0));
                    let tmp = *x.add(1);
                    *x.add(1) = sub16(tmp, *y.add(1));
                    *y.add(1) = add16(tmp, *y.add(1));
                }
            } else {
                // "Normal" split code
                let mut next_lowband2: *mut CeltNorm = std::ptr::null_mut();
                let mut next_lowband_out1: *mut CeltNorm = std::ptr::null_mut();
                let mut next_level = 0;

                // Give more bits to low-energy MDCTs than they would otherwise deserve
                if b0 > 1 && stereo == 0 && (itheta & 0x3fff) != 0 {
                    if itheta > 8192 {
                        delta -= delta >> (4 - lm);
                    } else {
                        delta = 0.min(delta + (n << BITRES as c_int >> (5 - lm)));
                    }
                }
                mbits = 0.max(b.min((b - delta) / 2));
                sbits = b - mbits;
                *remaining_bits -= qalloc;

                if !lowband.is_null() && stereo == 0 {
                    next_lowband2 = lowband.add(n as usize);
                }

                if stereo != 0 {
                    next_lowband_out1 = lowband_out;
                } else {
                    next_level = level + 1;
                }

                let mut rebalance = *remaining_bits;
                if mbits >= sbits {
                    cm = quant_band(
                        encode,
                        m,
                        i,
                        x,
                        std::ptr::null_mut(),
                        n,
                        mbits,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        lowband,
                        ec,
                        remaining_bits,
                        lm,
                        next_lowband_out1,
                        std::ptr::null(),
                        next_level,
                        seed,
                        if stereo != 0 { Q15ONE } else { mult16_16_p15(gain, mid) as OpusVal16 },
                        lowband_scratch,
                        fill,
                    );
                    rebalance = mbits - (rebalance - *remaining_bits);
                    if rebalance > 3 << BITRES as c_int && itheta != 0 {
                        sbits += rebalance - (3 << BITRES as c_int);
                    }
                    cm |= quant_band(
                        encode,
                        m,
                        i,
                        y,
                        std::ptr::null_mut(),
                        n,
                        sbits,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        next_lowband2,
                        ec,
                        remaining_bits,
                        lm,
                        std::ptr::null_mut(),
                        std::ptr::null(),
                        next_level,
                        seed,
                        mult16_16_p15(gain, side) as OpusVal16,
                        std::ptr::null_mut(),
                        fill >> b_blocks,
                    ) << ((b0 >> 1) & (stereo - 1));
                } else {
                    cm = quant_band(
                        encode,
                        m,
                        i,
                        y,
                        std::ptr::null_mut(),
                        n,
                        sbits,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        next_lowband2,
                        ec,
                        remaining_bits,
                        lm,
                        std::ptr::null_mut(),
                        std::ptr::null(),
                        next_level,
                        seed,
                        mult16_16_p15(gain, side) as OpusVal16,
                        std::ptr::null_mut(),
                        fill >> b_blocks,
                    ) << ((b0 >> 1) & (stereo - 1));
                    rebalance = sbits - (rebalance - *remaining_bits);
                    if rebalance > 3 << BITRES as c_int && itheta != 16384 {
                        mbits += rebalance - (3 << BITRES as c_int);
                    }
                    cm |= quant_band(
                        encode,
                        m,
                        i,
                        x,
                        std::ptr::null_mut(),
                        n,
                        mbits,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        lowband,
                        ec,
                        remaining_bits,
                        lm,
                        next_lowband_out1,
                        std::ptr::null(),
                        next_level,
                        seed,
                        if stereo != 0 { Q15ONE } else { mult16_16_p15(gain, mid) as OpusVal16 },
                        lowband_scratch,
                        fill,
                    );
                }
            }
        } else {
            // This is the basic no-split case
            let q = bits2pulses(m, i, lm, b);
            let curr_bits = pulses2bits(m, i, lm, q);
            *remaining_bits -= curr_bits;
            let mut q = q;
            let mut curr_bits = curr_bits;

            // Ensures we can never bust the budget
            while *remaining_bits < 0 && q > 0 {
                *remaining_bits += curr_bits;
                q -= 1;
                curr_bits = pulses2bits(m, i, lm, q);
                *remaining_bits -= curr_bits;
            }

            if q != 0 {
                let k = get_pulses(q);
                cm = alg_unquant(x, n, k, spread, b_blocks, ec, gain);
            } else {
                // If there's no pulse, fill the band anyway
                if resynth {
                    let cm_mask: u32 = (1u32 << b_blocks as u32) - 1;
                    fill &= cm_mask as c_int;
                    if fill == 0 {
                        for j in 0..n as usize {
                            *x.add(j) = 0 as CeltNorm;
                        }
                    } else {
                        if lowband.is_null() {
                            // Noise
                            for j in 0..n as usize {
                                *seed = celt_lcg_rand(*seed);
                                *x.add(j) = ((*seed as i32) >> 20) as CeltNorm;
                            }
                            cm = cm_mask;
                        } else {
                            // Folded spectrum
                            for j in 0..n as usize {
                                *seed = celt_lcg_rand(*seed);
                                let tmp = qconst16(1.0 / 256.0, 10);
                                let tmp = if (*seed) & 0x8000 != 0 { tmp } else { -tmp };
                                *x.add(j) = *lowband.add(j) + tmp;
                            }
                            cm = fill as u32;
                        }
                        renormalise_vector(x, n, gain);
                    }
                }
            }
        }

        // Resynthesis
        if resynth {
            if stereo != 0 {
                if n != 2 {
                    stereo_merge(x, y, mid, n);
                }
                if inv != 0 {
                    for j in 0..n as usize {
                        *y.add(j) = -*y.add(j);
                    }
                }
            } else if level == 0 {
                // Undo the sample reorganization going from time order to frequency order
                if b0 > 1 {
                    interleave_hadamard(x, n_b >> recombine, b0 << recombine, long_blocks as c_int);
                }

                // Undo time-freq changes that we did earlier
                n_b = n_b0;
                b_blocks = b0;
                for _k in 0..time_divide {
                    b_blocks >>= 1;
                    n_b <<= 1;
                    cm |= cm >> b_blocks as u32;
                    haar1(x, n_b, b_blocks);
                }

                for k in 0..recombine {
                    const BIT_DEINTERLEAVE_TABLE: [u8; 16] =
                        [0x00, 0x03, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F, 0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF];
                    cm = BIT_DEINTERLEAVE_TABLE[cm as usize] as u32;
                    haar1(x, n0 >> k, 1 << k);
                }
                b_blocks <<= recombine;

                // Scale output for later folding
                if !lowband_out.is_null() {
                    let norm_val = celt_sqrt(shl32(extend32(n0 as OpusVal16), 22));
                    for j in 0..n0 as usize {
                        *lowband_out.add(j) = mult16_16_q15(norm_val as OpusVal16, *x.add(j)) as CeltNorm;
                    }
                }
                cm &= (1u32 << b_blocks as u32) - 1;
            }
        }
        cm
    }
}

/// Decode all CELT bands by iterating over the band structure and calling
/// quant_band for each.
///
/// Handles dual-stereo (separate L/R decoding) vs joint stereo (mid/side),
/// bit budget management with rebalancing, lowband folding, and
/// collapse mask tracking.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn quant_all_bands(
    encode: c_int,
    m: *const CELTMode,
    start: c_int,
    end: c_int,
    x_: *mut CeltNorm,
    y_: *mut CeltNorm,
    collapse_masks: *mut u8,
    band_e: *const CeltEner,
    pulses: *mut c_int,
    short_blocks: c_int,
    spread: c_int,
    dual_stereo: c_int,
    intensity: c_int,
    tf_res: *mut c_int,
    total_bits: i32,
    balance: i32,
    ec: *mut ec_ctx,
    lm: c_int,
    coded_bands: c_int,
    seed: *mut u32,
) {
    unsafe {
        let mode = &*m;
        let resynth = encode == 0;
        let _ = resynth;
        let ebands = mode.ebands;

        let big_m = 1 << lm;
        let b_blocks = if short_blocks != 0 { big_m } else { 1 };
        let c = if !y_.is_null() { 2 } else { 1 };

        let norm_size = (c * big_m * (*ebands.add(mode.nb_ebands as usize)) as c_int) as usize;
        let scratch_size = (big_m
            * ((*ebands.add(mode.nb_ebands as usize)) as c_int - (*ebands.add((mode.nb_ebands - 1) as usize)) as c_int))
            as usize;
        let mut norm_buf = vec![0 as CeltNorm; norm_size];
        let mut scratch_buf = vec![0 as CeltNorm; scratch_size];
        let norm = norm_buf.as_mut_ptr();
        let norm2 = norm.add((big_m * (*ebands.add(mode.nb_ebands as usize)) as c_int) as usize);
        let lowband_scratch = scratch_buf.as_mut_ptr();

        let mut remaining_bits: i32;
        let mut balance = balance;
        let mut lowband_offset: c_int = 0;
        let mut update_lowband = 1;
        let mut dual_stereo = dual_stereo;

        for i in start..end {
            let i_u = i as usize;
            let x = x_.add((big_m * (*ebands.add(i_u)) as c_int) as usize);
            let y_ptr =
                if !y_.is_null() { y_.add((big_m * (*ebands.add(i_u)) as c_int) as usize) } else { std::ptr::null_mut() };
            let n = big_m * (*ebands.add(i_u + 1)) as c_int - big_m * (*ebands.add(i_u)) as c_int;
            let tell = ec_tell_frac(ec) as i32;

            if i != start {
                balance -= tell;
            }
            remaining_bits = total_bits - tell - 1;
            let b_val;
            if i <= coded_bands - 1 {
                let curr_balance = balance / 3.min(coded_bands - i);
                b_val = 0.max(16383.min(remaining_bits + 1).min(*pulses.add(i_u) + curr_balance));
            } else {
                b_val = 0;
            }

            if resynth
                && big_m * (*ebands.add(i_u)) as c_int - n >= big_m * (*ebands.add(start as usize)) as c_int
                && (update_lowband != 0 || lowband_offset == 0)
            {
                lowband_offset = i;
            }

            let tf_change = *tf_res.add(i_u);
            let (x_eff, y_eff) =
                if i >= mode.eff_ebands { (norm, if !y_.is_null() { norm } else { std::ptr::null_mut() }) } else { (x, y_ptr) };

            // Get a conservative estimate of the collapse_mask's for the bands we're going to fold from
            let mut x_cm: u32;
            let mut y_cm: u32;
            if lowband_offset != 0 && (spread != SPREAD_AGGRESSIVE || b_blocks > 1 || tf_change < 0) {
                let effective_lowband = (big_m * (*ebands.add(start as usize)) as c_int)
                    .max(big_m * (*ebands.add(lowband_offset as usize)) as c_int - n);
                let mut fold_start = lowband_offset;
                loop {
                    fold_start -= 1;
                    if big_m * (*ebands.add(fold_start as usize)) as c_int <= effective_lowband {
                        break;
                    }
                }
                let mut fold_end = lowband_offset - 1;
                loop {
                    fold_end += 1;
                    if big_m * (*ebands.add(fold_end as usize)) as c_int >= effective_lowband + n {
                        break;
                    }
                }
                x_cm = 0;
                y_cm = 0;
                let mut fold_i = fold_start;
                loop {
                    x_cm |= *collapse_masks.add((fold_i as usize) * c as usize) as u32;
                    y_cm |= *collapse_masks.add((fold_i as usize) * c as usize + c as usize - 1) as u32;
                    fold_i += 1;
                    if fold_i >= fold_end {
                        break;
                    }
                }

                // Now actually do the folding
                if dual_stereo != 0 && i == intensity {
                    dual_stereo = 0;
                    for j in (big_m * (*ebands.add(start as usize)) as c_int) as usize
                        ..(big_m * (*ebands.add(i_u)) as c_int) as usize
                    {
                        *norm.add(j) = half32(*norm.add(j) as OpusVal32 + *norm2.add(j) as OpusVal32) as CeltNorm;
                    }
                }

                if dual_stereo != 0 {
                    x_cm = quant_band(
                        encode,
                        m,
                        i,
                        x_eff,
                        std::ptr::null_mut(),
                        n,
                        b_val / 2,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        if effective_lowband != -1 { norm.add(effective_lowband as usize) } else { std::ptr::null_mut() },
                        ec,
                        &mut remaining_bits,
                        lm,
                        norm.add((big_m * (*ebands.add(i_u)) as c_int) as usize),
                        band_e,
                        0,
                        seed,
                        Q15ONE,
                        lowband_scratch,
                        x_cm as c_int,
                    );
                    y_cm = quant_band(
                        encode,
                        m,
                        i,
                        y_eff,
                        std::ptr::null_mut(),
                        n,
                        b_val / 2,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        if effective_lowband != -1 { norm2.add(effective_lowband as usize) } else { std::ptr::null_mut() },
                        ec,
                        &mut remaining_bits,
                        lm,
                        norm2.add((big_m * (*ebands.add(i_u)) as c_int) as usize),
                        band_e,
                        0,
                        seed,
                        Q15ONE,
                        lowband_scratch,
                        y_cm as c_int,
                    );
                } else {
                    x_cm = quant_band(
                        encode,
                        m,
                        i,
                        x_eff,
                        y_eff,
                        n,
                        b_val,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        if effective_lowband != -1 { norm.add(effective_lowband as usize) } else { std::ptr::null_mut() },
                        ec,
                        &mut remaining_bits,
                        lm,
                        norm.add((big_m * (*ebands.add(i_u)) as c_int) as usize),
                        band_e,
                        0,
                        seed,
                        Q15ONE,
                        lowband_scratch,
                        (x_cm | y_cm) as c_int,
                    );
                    y_cm = x_cm;
                }
            } else {
                x_cm = (1u32 << b_blocks as u32) - 1;
                y_cm = x_cm;

                if dual_stereo != 0 && i == intensity {
                    dual_stereo = 0;
                    for j in (big_m * (*ebands.add(start as usize)) as c_int) as usize
                        ..(big_m * (*ebands.add(i_u)) as c_int) as usize
                    {
                        *norm.add(j) = half32(*norm.add(j) as OpusVal32 + *norm2.add(j) as OpusVal32) as CeltNorm;
                    }
                }

                if dual_stereo != 0 {
                    x_cm = quant_band(
                        encode,
                        m,
                        i,
                        x_eff,
                        std::ptr::null_mut(),
                        n,
                        b_val / 2,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        std::ptr::null_mut(),
                        ec,
                        &mut remaining_bits,
                        lm,
                        norm.add((big_m * (*ebands.add(i_u)) as c_int) as usize),
                        band_e,
                        0,
                        seed,
                        Q15ONE,
                        lowband_scratch,
                        x_cm as c_int,
                    );
                    y_cm = quant_band(
                        encode,
                        m,
                        i,
                        y_eff,
                        std::ptr::null_mut(),
                        n,
                        b_val / 2,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        std::ptr::null_mut(),
                        ec,
                        &mut remaining_bits,
                        lm,
                        norm2.add((big_m * (*ebands.add(i_u)) as c_int) as usize),
                        band_e,
                        0,
                        seed,
                        Q15ONE,
                        lowband_scratch,
                        y_cm as c_int,
                    );
                } else {
                    x_cm = quant_band(
                        encode,
                        m,
                        i,
                        x_eff,
                        y_eff,
                        n,
                        b_val,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        std::ptr::null_mut(),
                        ec,
                        &mut remaining_bits,
                        lm,
                        norm.add((big_m * (*ebands.add(i_u)) as c_int) as usize),
                        band_e,
                        0,
                        seed,
                        Q15ONE,
                        lowband_scratch,
                        (x_cm | y_cm) as c_int,
                    );
                    y_cm = x_cm;
                }
            }
            *collapse_masks.add(i_u * c as usize) = x_cm as u8;
            *collapse_masks.add(i_u * c as usize + c as usize - 1) = y_cm as u8;
            balance += *pulses.add(i_u) + tell;

            update_lowband = (b_val > (n << BITRES as c_int)) as c_int;
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
