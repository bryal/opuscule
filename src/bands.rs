// Translated from c/celt/bands.c (RFC 6716).
//
// CELT band processing functions: energy computation, normalisation,
// stereo handling, and spectral folding/unfolding.
//
// Functions are translated incrementally, innermost helpers first.

use std::os::raw::c_int;

use crate::arch::{CeltEner, CeltNorm, CeltSig, EPSILON, NORM_SCALING, OpusVal16, OpusVal32, Q15ONE, qconst16, qconst32};
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
use crate::mathops::frac_mul16;
use crate::modes::CELTMode;
use crate::rate::{bits2pulses, get_pulses, pulses2bits};
use crate::vq::{alg_unquant, renormalise_vector};

/// Linear congruential generator used for pseudo-random noise injection
/// in the CELT decoder (PLC comfort noise, anti-collapse, spectral folding).
/// Constants match Numerical Recipes / Knuth MMIX.
pub fn celt_lcg_rand(seed: u32) -> u32 {
    seed.wrapping_mul(1664525).wrapping_add(1013904223)
}

/// De-normalise band energies to produce synthesis output.
///
/// Scales each unit-energy band by its decoded energy envelope, producing
/// the frequency-domain signal that feeds into the inverse MDCT.
/// Bands beyond `end` are zeroed (above the Nyquist for the coded bandwidth).
pub fn denormalise_bands(
    m: &CELTMode,
    x: &[CeltNorm],
    freq: &mut [CeltSig],
    band_e: &[CeltEner],
    end: c_int,
    c_channels: c_int,
    m_factor: c_int,
) {
    let ebands = m.ebands;
    let n = m_factor * m.short_mdct_size;

    let mut c = 0;
    loop {
        let ch_off = (c * n) as usize;
        for i in 0..end as usize {
            let g = shr32(band_e[i + (c as usize) * m.nb_ebands as usize], 1);
            let j_start = (m_factor * (ebands[i] as c_int)) as usize;
            let band_end = (m_factor * (ebands[i + 1] as c_int)) as usize;
            for j in j_start..band_end {
                freq[ch_off + j] = shl32(mult16_32_q15(x[ch_off + j], g), 2);
            }
        }

        // Zero above the coded bandwidth
        let zero_start = (m_factor * (ebands[end as usize] as c_int)) as usize;
        for i in zero_start..n as usize {
            freq[ch_off + i] = 0 as CeltSig;
        }

        c += 1;
        if c >= c_channels {
            break;
        }
    }
}

/// Single-level Haar wavelet transform on interleaved sub-vectors.
///
/// Used inside quant_band to (de)interleave time-frequency coefficients
/// for the Hadamard rearrangement that maps between time-domain short
/// blocks and the band's frequency layout. Applies the unnormalised
/// Haar butterfly (scaled by 1/sqrt(2)) in-place.
pub fn haar1(x: &mut [CeltNorm], n0: c_int, stride: c_int) {
    let n0 = n0 >> 1;
    for i in 0..stride {
        for j in 0..n0 {
            let idx0 = (stride * 2 * j + i) as usize;
            let idx1 = (stride * (2 * j + 1) + i) as usize;
            let tmp1 = mult16_16_q15(qconst16(0.70710678, 15), x[idx0]) as CeltNorm;
            let tmp2 = mult16_16_q15(qconst16(0.70710678, 15), x[idx1]) as CeltNorm;
            x[idx0] = tmp1 + tmp2;
            x[idx1] = tmp1 - tmp2;
        }
    }
}

/// Intensity stereo: rotate (X, Y) onto X using the energy ratio.
///
/// Computes the left/right energy ratio for the band, derives mixing
/// coefficients a1 and a2, and replaces X with the intensity-coded
/// mono signal. Y is not updated (side is discarded at this point).
pub fn intensity_stereo(m: &CELTMode, x: &mut [CeltNorm], y: &[CeltNorm], band_e: &[CeltEner], band_id: c_int, n: c_int) {
    let i = band_id as usize;
    let nb = m.nb_ebands as usize;

    #[cfg(feature = "fixed-point")]
    let shift = (celt_zlog2(band_e[i].max(band_e[i + nb])) - 13) as i32;
    #[cfg(not(feature = "fixed-point"))]
    let shift: i32 = 0;

    let left = vshr32(band_e[i], shift);
    let right = vshr32(band_e[i + nb], shift);
    let norm = EPSILON
        + celt_sqrt(
            EPSILON + mult16_16(left as OpusVal16, left as OpusVal16) + mult16_16(right as OpusVal16, right as OpusVal16),
        );
    let a1 = div32_16(shl32(extend32(left as OpusVal16), 14), norm as OpusVal16);
    let a2 = div32_16(shl32(extend32(right as OpusVal16), 14), norm as OpusVal16);
    for j in 0..n as usize {
        let l = x[j];
        let r = y[j];
        x[j] = (mult16_16_q14(a1 as OpusVal16, l) + mult16_16_q14(a2 as OpusVal16, r)) as CeltNorm;
    }
}

/// Stereo merge: reconstruct (L, R) from (mid, side) after decoding.
///
/// Uses the energy invariance property to compute proper L/R gains
/// from the decoded mid and side signals. Falls back to copying mid
/// to both channels if the energy is near zero.
pub fn stereo_merge(x: &mut [CeltNorm], y: &mut [CeltNorm], mid: OpusVal16) {
    let mut xp: OpusVal32 = 0 as OpusVal32;
    let mut side: OpusVal32 = 0 as OpusVal32;

    for j in 0..x.len() {
        xp = mac16_16(xp, x[j], y[j]);
        side = mac16_16(side, y[j], y[j]);
    }
    xp = mult16_32_q15(mid, xp);
    let mid2 = shr32(mid as OpusVal32, 1) as OpusVal16;
    let el = mult16_16(mid2, mid2) + side - (2 as OpusVal32) * xp;
    let er = mult16_16(mid2, mid2) + side + (2 as OpusVal32) * xp;
    if er < qconst32(6e-4, 28) || el < qconst32(6e-4, 28) {
        y.copy_from_slice(x);
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

    for j in 0..x.len() {
        let l = mult16_16_q15(mid, x[j]);
        let r = y[j];
        x[j] = extract16(pshr32(mult16_16(lgain as OpusVal16, sub16(l as OpusVal16, r)), kl + 1));
        y[j] = extract16(pshr32(mult16_16(rgain as OpusVal16, add16(l as OpusVal16, r)), kr + 1));
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
pub fn deinterleave_hadamard(x: &mut [CeltNorm], n0: c_int, stride: c_int, hadamard: c_int) {
    let n = (n0 * stride) as usize;
    let mut tmp = vec![0 as CeltNorm; n];
    if hadamard != 0 {
        let ordery = &ORDERY_TABLE[(stride - 2) as usize..];
        for i in 0..stride as usize {
            for j in 0..n0 as usize {
                tmp[ordery[i] as usize * n0 as usize + j] = x[j * stride as usize + i];
            }
        }
    } else {
        for i in 0..stride as usize {
            for j in 0..n0 as usize {
                tmp[i * n0 as usize + j] = x[j * stride as usize + i];
            }
        }
    }
    x[..n].copy_from_slice(&tmp);
}

/// Interleave sub-vectors with optional Hadamard reordering.
///
/// Inverse of deinterleave_hadamard: rearranges contiguous sub-vectors
/// back into stride-interleaved layout, with optional ordery Hadamard
/// permutation. Used after recursive band reconstruction in quant_band.
pub fn interleave_hadamard(x: &mut [CeltNorm], n0: c_int, stride: c_int, hadamard: c_int) {
    let n = (n0 * stride) as usize;
    let mut tmp = vec![0 as CeltNorm; n];
    if hadamard != 0 {
        let ordery = &ORDERY_TABLE[(stride - 2) as usize..];
        for i in 0..stride as usize {
            for j in 0..n0 as usize {
                tmp[j * stride as usize + i] = x[ordery[i] as usize * n0 as usize + j];
            }
        }
    } else {
        for i in 0..stride as usize {
            for j in 0..n0 as usize {
                tmp[j * stride as usize + i] = x[i * n0 as usize + j];
            }
        }
    }
    x[..n].copy_from_slice(&tmp);
}

/// Spread mode constants (from bands.h).
pub const SPREAD_NONE: c_int = 0;
pub const SPREAD_LIGHT: c_int = 1;
pub const SPREAD_NORMAL: c_int = 2;
pub const SPREAD_AGGRESSIVE: c_int = 3;

/// Compute the number of quantisation levels for a band given the
/// available bits, band size, and pulse cap. Used by quant_band to
/// decide how finely to quantise the angular parameter theta.
pub fn compute_qn(n: c_int, b: c_int, offset: c_int, pulse_cap: c_int, stereo: c_int) -> c_int {
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
pub fn anti_collapse(
    m: &CELTMode,
    x_: &mut [CeltNorm],
    collapse_masks: &[u8],
    lm: c_int,
    c_channels: c_int,
    size: c_int,
    start: c_int,
    end: c_int,
    log_e: &[OpusVal16],
    prev1log_e: &[OpusVal16],
    prev2log_e: &[OpusVal16],
    pulses: &[c_int],
    seed: u32,
) {
    {
        let mut seed = seed;

        for i in start..end {
            let i = i as usize;
            let n0 = (m.ebands[i + 1] - m.ebands[i]) as c_int;
            // depth in 1/8 bits
            let depth = (1 + pulses[i]) / (((m.ebands[i + 1] - m.ebands[i]) as c_int) << lm);

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
                let nb_ebands = m.nb_ebands as usize;
                let mut prev1 = prev1log_e[c as usize * nb_ebands + i];
                let mut prev2 = prev2log_e[c as usize * nb_ebands + i];
                if c_channels == 1 {
                    prev1 = prev1.max(prev1log_e[nb_ebands + i]);
                    prev2 = prev2.max(prev2log_e[nb_ebands + i]);
                }
                let ediff = extend32(log_e[c as usize * nb_ebands + i]) - extend32(min16(prev1, prev2));
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

                let x_off = c as usize * size as usize + ((m.ebands[i] as c_int) << lm) as usize;
                let mut renormalize = 0;
                for k in 0..1 << lm {
                    // Detect collapse
                    if (collapse_masks[i * c_channels as usize + c as usize] & (1 << k)) == 0 {
                        // Fill with noise
                        for j in 0..n0 {
                            seed = celt_lcg_rand(seed);
                            x_[x_off + ((j << lm) + k) as usize] = if seed & 0x8000 != 0 { r } else { -r };
                        }
                        renormalize = 1;
                    }
                }
                // We just added some energy, so we need to renormalise
                if renormalize != 0 {
                    renormalise_vector(&mut x_[x_off..x_off + ((n0 << lm) as usize)], Q15ONE);
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
///
/// Slice-based replacement for the C pointer interface:
/// - `x`/`y` are the two channel (or band-half) buffers; they are always
///   disjoint (quant_all_bands routes the discarded above-eff_ebands
///   stereo case to norm/norm2 instead of the C's aliased norm/norm).
/// - The mono band-split keeps `x` whole and splits it into temporary
///   halves around the recursive calls only, because the end-of-band
///   resynthesis (Hadamard/Haar undo) runs over the full band.
/// - `lowband` is a snapshot of the fold source (see quant_all_bands);
///   when it needs in-place transforms it is first copied into
///   `lowband_scratch`, which is then *moved* into `lowband` (the C kept
///   both pointers, but never uses the scratch again afterwards).
#[allow(clippy::too_many_arguments)]
pub fn quant_band(
    encode: c_int,
    m: &CELTMode,
    i: c_int,
    x: &mut [CeltNorm],
    y: Option<&mut [CeltNorm]>,
    n_in: c_int,
    b_in: c_int,
    spread: c_int,
    b_blocks_in: c_int,
    intensity: c_int,
    tf_change_in: c_int,
    lowband_in: Option<&mut [CeltNorm]>,
    ec: &mut ec_ctx,
    remaining_bits: &mut i32,
    lm_in: c_int,
    lowband_out: Option<&mut [CeltNorm]>,
    level: c_int,
    seed: &mut u32,
    gain: OpusVal16,
    lowband_scratch: Option<&mut [CeltNorm]>,
    fill_in: c_int,
) -> u32 {
    let resynth = encode == 0;

    let mut n = n_in;
    let mut b = b_in;
    let mut b_blocks = b_blocks_in;
    let mut tf_change = tf_change_in;
    let mut lm = lm_in;
    let mut fill = fill_in;

    let x_s = x;
    let mut y_s: Option<&mut [CeltNorm]> = y;
    let mut lowband: Option<&mut [CeltNorm]> = lowband_in;
    let mut lowband_scratch: Option<&mut [CeltNorm]> = lowband_scratch;
    let mut lowband_out: Option<&mut [CeltNorm]> = lowband_out;

    let long_blocks = b_blocks == 1;
    let n0 = n;
    let mut n_b = n / b_blocks;
    let n_b0;
    let mut b0 = b_blocks;
    let mut time_divide = 0;
    let mut recombine = 0;
    let mut inv = 0;
    let mut mid: OpusVal16 = 0 as OpusVal16;
    let mut cm: u32 = 0;
    let imid;
    let iside;

    let stereo = y_s.is_some() as c_int;
    let mut split = stereo;

    // Special case for one sample
    if n == 1 {
        let mut c = 0;
        loop {
            let mut sign = 0u32;
            if *remaining_bits >= 1 << BITRES {
                sign = ec_dec_bits(ec, 1);
                *remaining_bits -= 1 << BITRES as c_int;
            }
            if resynth {
                let val = if sign != 0 { -NORM_SCALING } else { NORM_SCALING };
                if c == 0 {
                    x_s[0] = val;
                } else {
                    // c reaches 1 only when 1 + stereo > 1, i.e. stereo, i.e. y_s is Some.
                    y_s.as_mut().expect("stereo path implies y_s is Some")[0] = val;
                }
            }
            c += 1;
            if c >= 1 + stereo {
                break;
            }
        }
        if let Some(lb_out) = lowband_out {
            lb_out[0] = shr16(x_s[0], 4);
        }
        return 1;
    }

    if stereo == 0 && level == 0 {
        if tf_change > 0 {
            recombine = tf_change;
        }

        if lowband.is_some() && (recombine != 0 || ((n_b & 1) == 0 && tf_change < 0) || b0 > 1) {
            // Copy the fold source somewhere we can transform it in place;
            // the scratch then *becomes* the lowband for the rest of the band.
            let lb = lowband.take().expect("guarded by lowband.is_some() in the condition");
            let scratch = lowband_scratch.take().expect("quant_all_bands always supplies scratch at level 0");
            scratch[..n as usize].copy_from_slice(&lb[..n as usize]);
            lowband = Some(scratch);
        }

        for k in 0..recombine {
            const BIT_INTERLEAVE_TABLE: [u8; 16] = [0, 1, 1, 1, 2, 3, 3, 3, 2, 3, 3, 3, 2, 3, 3, 3];
            if let Some(lb) = lowband.as_mut() {
                let n0k = n >> k;
                let stride = 1 << k;
                haar1(&mut lb[..(n0k * stride) as usize], n0k, stride);
            }
            fill = BIT_INTERLEAVE_TABLE[(fill & 0xF) as usize] as c_int
                | (BIT_INTERLEAVE_TABLE[(fill >> 4) as usize] as c_int) << 2;
        }
        b_blocks >>= recombine;
        n_b <<= recombine;

        // Increasing the time resolution
        while (n_b & 1) == 0 && tf_change < 0 {
            if let Some(lb) = lowband.as_mut() {
                haar1(&mut lb[..(n_b * b_blocks) as usize], n_b, b_blocks);
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
            if let Some(lb) = lowband.as_mut() {
                let n0d = n_b >> recombine;
                let stride = b0 << recombine;
                deinterleave_hadamard(&mut lb[..(n0d * stride) as usize], n0d, stride, long_blocks as c_int);
            }
        }
    }
    n_b0 = n_b;

    // If we need 1.5 more bit than we can produce, split the band in two.
    let cache = &m.cache.bits[m.cache.index[((lm + 1) * m.nb_ebands + i) as usize] as usize..];
    if stereo == 0 && lm != -1 && b > cache[cache[0] as usize] as c_int + 12 && n > 2 {
        n >>= 1;
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
        pulse_cap = m.log_n[i as usize] as c_int + lm * (1 << BITRES as c_int);
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
                    itheta = (((8 * fm + 1).isqrt() as i32 - 1) >> 1) as c_int;
                    let fs = itheta + 1;
                    let fl = itheta * (itheta + 1) >> 1;
                    ec_dec_update(ec, fl as u32, (fl + fs) as u32, ft);
                } else {
                    itheta = ((2 * (qn_val + 1) - (8 * (ft - fm - 1) + 1).isqrt() as i32) >> 1) as c_int;
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

        let side;
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

            if sbits != 0 {
                sign = ec_dec_bits(ec, 1) as i32;
            }
            sign = 1 - 2 * sign;
            {
                let y_sl = y_s.as_deref_mut().expect("stereo path implies y_s is Some");
                let (x2, y2): (&mut [CeltNorm], &mut [CeltNorm]) =
                    if c_side != 0 { (y_sl, &mut x_s[..]) } else { (&mut x_s[..], y_sl) };
                cm = quant_band(
                    encode,
                    m,
                    i,
                    &mut x2[..],
                    None,
                    n,
                    mbits,
                    spread,
                    b_blocks,
                    intensity,
                    tf_change,
                    lowband.take(),
                    ec,
                    remaining_bits,
                    lm,
                    lowband_out.take(),
                    level,
                    seed,
                    gain,
                    lowband_scratch.take(),
                    orig_fill,
                );
                y2[0] = -(sign as CeltNorm) * x2[1];
                y2[1] = (sign as CeltNorm) * x2[0];
            }
            if resynth {
                let y_sl = y_s.as_deref_mut().expect("stereo path implies y_s is Some");
                x_s[0] = mult16_16_q15(mid, x_s[0]) as CeltNorm;
                x_s[1] = mult16_16_q15(mid, x_s[1]) as CeltNorm;
                y_sl[0] = mult16_16_q15(side, y_sl[0]) as CeltNorm;
                y_sl[1] = mult16_16_q15(side, y_sl[1]) as CeltNorm;
                let tmp = x_s[0];
                x_s[0] = sub16(tmp, y_sl[0]);
                y_sl[0] = add16(tmp, y_sl[0]);
                let tmp = x_s[1];
                x_s[1] = sub16(tmp, y_sl[1]);
                y_sl[1] = add16(tmp, y_sl[1]);
            }
        } else {
            // "Normal" split code
            let mut next_level = level;

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

            // Split the fold source for the two halves (mono band-split only)
            let mut lowband1: Option<&mut [CeltNorm]> = None;
            let mut next_lowband2: Option<&mut [CeltNorm]> = None;
            if let Some(lb) = lowband.take() {
                if stereo == 0 {
                    let (a, b2) = lb.split_at_mut(n as usize);
                    lowband1 = Some(a);
                    next_lowband2 = Some(b2);
                } else {
                    lowband1 = Some(lb);
                }
            }

            let mut next_lowband_out1: Option<&mut [CeltNorm]> = None;
            if stereo != 0 {
                // For a stereo split, the high bits of fill are always zero,
                // so no folding will be done to the side.
                next_lowband_out1 = lowband_out.take();
            } else {
                next_level = level + 1;
            }

            {
                // The two recursion targets: true stereo channels, or the
                // two halves of this band (temporary reborrow so the full
                // band is available again for the resynthesis below).
                let (x_part, y_part): (&mut [CeltNorm], &mut [CeltNorm]) = if stereo != 0 {
                    (&mut x_s[..], y_s.as_deref_mut().expect("stereo path implies y_s is Some"))
                } else {
                    x_s.split_at_mut(n as usize)
                };

                let mut rebalance = *remaining_bits;
                if mbits >= sbits {
                    cm = quant_band(
                        encode,
                        m,
                        i,
                        x_part,
                        None,
                        n,
                        mbits,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        lowband1.take(),
                        ec,
                        remaining_bits,
                        lm,
                        next_lowband_out1.take(),
                        next_level,
                        seed,
                        if stereo != 0 { Q15ONE } else { mult16_16_p15(gain, mid) as OpusVal16 },
                        lowband_scratch.take(),
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
                        y_part,
                        None,
                        n,
                        sbits,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        next_lowband2.take(),
                        ec,
                        remaining_bits,
                        lm,
                        None,
                        next_level,
                        seed,
                        mult16_16_p15(gain, side) as OpusVal16,
                        None,
                        fill >> b_blocks,
                    ) << ((b0 >> 1) & (stereo - 1));
                } else {
                    cm = quant_band(
                        encode,
                        m,
                        i,
                        y_part,
                        None,
                        n,
                        sbits,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        next_lowband2.take(),
                        ec,
                        remaining_bits,
                        lm,
                        None,
                        next_level,
                        seed,
                        mult16_16_p15(gain, side) as OpusVal16,
                        None,
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
                        x_part,
                        None,
                        n,
                        mbits,
                        spread,
                        b_blocks,
                        intensity,
                        tf_change,
                        lowband1.take(),
                        ec,
                        remaining_bits,
                        lm,
                        next_lowband_out1.take(),
                        next_level,
                        seed,
                        if stereo != 0 { Q15ONE } else { mult16_16_p15(gain, mid) as OpusVal16 },
                        lowband_scratch.take(),
                        fill,
                    );
                }
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
            cm = alg_unquant(&mut x_s[..n as usize], n, k, spread, b_blocks, ec, gain);
        } else {
            // If there's no pulse, fill the band anyway
            if resynth {
                let cm_mask: u32 = (1u32 << b_blocks as u32) - 1;
                fill &= cm_mask as c_int;
                if fill == 0 {
                    for j in 0..n as usize {
                        x_s[j] = 0 as CeltNorm;
                    }
                } else {
                    match lowband.as_ref() {
                        None => {
                            // Noise
                            for j in 0..n as usize {
                                *seed = celt_lcg_rand(*seed);
                                x_s[j] = ((*seed as i32) >> 20) as CeltNorm;
                            }
                            cm = cm_mask;
                        }
                        Some(lb) => {
                            // Folded spectrum
                            for j in 0..n as usize {
                                *seed = celt_lcg_rand(*seed);
                                let tmp = qconst16(1.0 / 256.0, 10);
                                let tmp = if (*seed) & 0x8000 != 0 { tmp } else { -tmp };
                                x_s[j] = lb[j] + tmp;
                            }
                            cm = fill as u32;
                        }
                    }
                    renormalise_vector(&mut x_s[..n as usize], gain);
                }
            }
        }
    }

    // Resynthesis
    if resynth {
        if stereo != 0 {
            if n != 2 {
                let y_sl = y_s.as_deref_mut().expect("stereo path implies y_s is Some");
                stereo_merge(&mut x_s[..n as usize], &mut y_sl[..n as usize], mid);
            }
            if inv != 0 {
                let y_sl = y_s.as_deref_mut().expect("stereo path implies y_s is Some");
                for yj in y_sl[..n as usize].iter_mut() {
                    *yj = -*yj;
                }
            }
        } else if level == 0 {
            // Undo the sample reorganization going from time order to frequency order
            if b0 > 1 {
                let n0d = n_b >> recombine;
                let stride = b0 << recombine;
                interleave_hadamard(&mut x_s[..(n0d * stride) as usize], n0d, stride, long_blocks as c_int);
            }

            // Undo time-freq changes that we did earlier
            n_b = n_b0;
            b_blocks = b0;
            for _k in 0..time_divide {
                b_blocks >>= 1;
                n_b <<= 1;
                cm |= cm >> b_blocks as u32;
                haar1(&mut x_s[..(n_b * b_blocks) as usize], n_b, b_blocks);
            }

            for k in 0..recombine {
                const BIT_DEINTERLEAVE_TABLE: [u8; 16] =
                    [0x00, 0x03, 0x0C, 0x0F, 0x30, 0x33, 0x3C, 0x3F, 0xC0, 0xC3, 0xCC, 0xCF, 0xF0, 0xF3, 0xFC, 0xFF];
                cm = BIT_DEINTERLEAVE_TABLE[cm as usize] as u32;
                let n0_param = n0 >> k;
                let stride = 1 << k;
                haar1(&mut x_s[..(n0_param * stride) as usize], n0_param, stride);
            }
            b_blocks <<= recombine;

            // Scale output for later folding
            if let Some(lb_out) = lowband_out {
                let norm_val = celt_sqrt(shl32(extend32(n0 as OpusVal16), 22));
                for j in 0..n0 as usize {
                    lb_out[j] = mult16_16_q15(norm_val as OpusVal16, x_s[j]) as CeltNorm;
                }
            }
            cm &= (1u32 << b_blocks as u32) - 1;
        }
    }
    cm
}

/// Decode all CELT bands by iterating over the band structure and calling
/// quant_band for each.
///
/// Handles dual-stereo (separate L/R decoding) vs joint stereo (mid/side),
/// bit budget management with rebalancing, lowband folding, and
/// collapse mask tracking.
///
/// Two deliberate departures from the C pointer plumbing, both confined
/// to bands at or above m.eff_ebands (whose decoded samples never reach
/// the output — denormalise_bands stops at eff_end — and whose bit
/// consumption and collapse masks depend only on the bitstream):
/// - The C passes the *same* norm buffer as both X and Y for the joint
///   stereo case there; we pass norm2 for Y instead.
/// - The fold source (lowband) is snapshotted into a separate buffer
///   before each quant_band call instead of pointing into norm. In the C
///   the live region can overlap X, but every fold read stays ahead of
///   the corresponding write (read index = write index + lowband offset,
///   which is positive), so a pre-copy is exactly equivalent.
#[allow(clippy::too_many_arguments)]
pub fn quant_all_bands(
    encode: c_int,
    m: &CELTMode,
    start: c_int,
    end: c_int,
    x_: &mut [CeltNorm],
    y_: Option<&mut [CeltNorm]>,
    collapse_masks: &mut [u8],
    pulses: &[c_int],
    short_blocks: c_int,
    spread: c_int,
    dual_stereo: c_int,
    intensity: c_int,
    tf_res: &[c_int],
    total_bits: i32,
    balance: i32,
    ec: &mut ec_ctx,
    lm: c_int,
    coded_bands: c_int,
    seed: &mut u32,
) {
    let resynth = encode == 0;
    let ebands = m.ebands;

    let big_m = 1 << lm;
    let b_blocks = if short_blocks != 0 { big_m } else { 1 };
    let mut y_ = y_;
    let c = if y_.is_some() { 2 } else { 1 };

    let norm_len = (big_m * ebands[m.nb_ebands as usize] as c_int) as usize;
    let norm_size = c as usize * norm_len;
    let scratch_size = (big_m * (ebands[m.nb_ebands as usize] as c_int - ebands[(m.nb_ebands - 1) as usize] as c_int)) as usize;
    let mut norm_buf = vec![0 as CeltNorm; norm_size];
    let mut scratch_buf = vec![0 as CeltNorm; scratch_size];
    let mut fold_buf = vec![0 as CeltNorm; scratch_size];
    let (norm, norm2) = norm_buf.split_at_mut(norm_len);

    let mut remaining_bits: i32;
    let mut balance = balance;
    let mut lowband_offset: c_int = 0;
    let mut update_lowband = 1;
    let mut dual_stereo = dual_stereo;

    for i in start..end {
        let i_u = i as usize;
        let eb_i = (big_m * ebands[i_u] as c_int) as usize;
        let n = big_m * ebands[i_u + 1] as c_int - big_m * ebands[i_u] as c_int;
        let tell = ec_tell_frac(ec) as i32;

        if i != start {
            balance -= tell;
        }
        remaining_bits = total_bits - tell - 1;
        let b_val;
        if i <= coded_bands - 1 {
            let curr_balance = balance / 3.min(coded_bands - i);
            b_val = 0.max(16383.min(remaining_bits + 1).min(pulses[i_u] + curr_balance));
        } else {
            b_val = 0;
        }

        if resynth
            && big_m * ebands[i_u] as c_int - n >= big_m * ebands[start as usize] as c_int
            && (update_lowband != 0 || lowband_offset == 0)
        {
            lowband_offset = i;
        }

        let tf_change = tf_res[i_u];
        let above_eff = i >= m.eff_ebands;

        // Get a conservative estimate of the collapse_mask's for the bands we're going to fold from
        let mut x_cm: u32;
        let mut y_cm: u32;
        let mut effective_lowband: c_int = -1;
        if lowband_offset != 0 && (spread != SPREAD_AGGRESSIVE || b_blocks > 1 || tf_change < 0) {
            effective_lowband =
                (big_m * ebands[start as usize] as c_int).max(big_m * ebands[lowband_offset as usize] as c_int - n);
            let mut fold_start = lowband_offset;
            loop {
                fold_start -= 1;
                if big_m * ebands[fold_start as usize] as c_int <= effective_lowband {
                    break;
                }
            }
            let mut fold_end = lowband_offset - 1;
            loop {
                fold_end += 1;
                if big_m * ebands[fold_end as usize] as c_int >= effective_lowband + n {
                    break;
                }
            }
            x_cm = 0;
            y_cm = 0;
            let mut fold_i = fold_start;
            loop {
                x_cm |= collapse_masks[(fold_i as usize) * c as usize] as u32;
                y_cm |= collapse_masks[(fold_i as usize) * c as usize + c as usize - 1] as u32;
                fold_i += 1;
                if fold_i >= fold_end {
                    break;
                }
            }
        } else {
            x_cm = (1u32 << b_blocks as u32) - 1;
            y_cm = x_cm;
        }

        if dual_stereo != 0 && i == intensity {
            dual_stereo = 0;
            for j in (big_m * ebands[start as usize] as c_int) as usize..eb_i {
                norm[j] = half32(norm[j] as OpusVal32 + norm2[j] as OpusVal32) as CeltNorm;
            }
        }

        if dual_stereo != 0 {
            // Channel 0 from norm, channel 1 from norm2
            let lb = if effective_lowband != -1 {
                fold_buf[..n as usize].copy_from_slice(&norm[effective_lowband as usize..(effective_lowband + n) as usize]);
                Some(&mut fold_buf[..n as usize])
            } else {
                None
            };
            x_cm = {
                let (x_band, lb_out) = if above_eff {
                    let (nlo, nhi) = norm.split_at_mut(eb_i);
                    (&mut nlo[..n as usize], &mut nhi[..n as usize])
                } else {
                    (&mut x_[eb_i..eb_i + n as usize], &mut norm[eb_i..eb_i + n as usize])
                };
                quant_band(
                    encode,
                    m,
                    i,
                    x_band,
                    None,
                    n,
                    b_val / 2,
                    spread,
                    b_blocks,
                    intensity,
                    tf_change,
                    lb,
                    ec,
                    &mut remaining_bits,
                    lm,
                    Some(lb_out),
                    0,
                    seed,
                    Q15ONE,
                    Some(&mut scratch_buf[..]),
                    x_cm as c_int,
                )
            };
            let lb2 = if effective_lowband != -1 {
                fold_buf[..n as usize].copy_from_slice(&norm2[effective_lowband as usize..(effective_lowband + n) as usize]);
                Some(&mut fold_buf[..n as usize])
            } else {
                None
            };
            y_cm = {
                let (y_band, lb_out) = if above_eff {
                    let (nlo, nhi) = norm2.split_at_mut(eb_i);
                    (&mut nlo[..n as usize], &mut nhi[..n as usize])
                } else {
                    (
                        &mut y_.as_deref_mut().expect("dual_stereo implies stereo: y_ is Some")[eb_i..eb_i + n as usize],
                        &mut norm2[eb_i..eb_i + n as usize],
                    )
                };
                quant_band(
                    encode,
                    m,
                    i,
                    y_band,
                    None,
                    n,
                    b_val / 2,
                    spread,
                    b_blocks,
                    intensity,
                    tf_change,
                    lb2,
                    ec,
                    &mut remaining_bits,
                    lm,
                    Some(lb_out),
                    0,
                    seed,
                    Q15ONE,
                    Some(&mut scratch_buf[..]),
                    y_cm as c_int,
                )
            };
        } else {
            // Joint stereo (or mono)
            let lb = if effective_lowband != -1 {
                fold_buf[..n as usize].copy_from_slice(&norm[effective_lowband as usize..(effective_lowband + n) as usize]);
                Some(&mut fold_buf[..n as usize])
            } else {
                None
            };
            x_cm = if above_eff {
                let (nlo, nhi) = norm.split_at_mut(eb_i);
                quant_band(
                    encode,
                    m,
                    i,
                    &mut nlo[..n as usize],
                    if y_.is_some() { Some(&mut norm2[..n as usize]) } else { None },
                    n,
                    b_val,
                    spread,
                    b_blocks,
                    intensity,
                    tf_change,
                    lb,
                    ec,
                    &mut remaining_bits,
                    lm,
                    Some(&mut nhi[..n as usize]),
                    0,
                    seed,
                    Q15ONE,
                    Some(&mut scratch_buf[..]),
                    (x_cm | y_cm) as c_int,
                )
            } else {
                quant_band(
                    encode,
                    m,
                    i,
                    &mut x_[eb_i..eb_i + n as usize],
                    y_.as_deref_mut().map(|y| &mut y[eb_i..eb_i + n as usize]),
                    n,
                    b_val,
                    spread,
                    b_blocks,
                    intensity,
                    tf_change,
                    lb,
                    ec,
                    &mut remaining_bits,
                    lm,
                    Some(&mut norm[eb_i..eb_i + n as usize]),
                    0,
                    seed,
                    Q15ONE,
                    Some(&mut scratch_buf[..]),
                    (x_cm | y_cm) as c_int,
                )
            };
            y_cm = x_cm;
        }
        collapse_masks[i_u * c as usize] = x_cm as u8;
        collapse_masks[i_u * c as usize + c as usize - 1] = y_cm as u8;
        balance += pulses[i_u] + tell;

        update_lowband = (b_val > (n << BITRES as c_int)) as c_int;
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
