// Translated from c/celt/pitch.c (RFC 6716).
//
// Pitch analysis routines used by the CELT PLC (packet loss concealment)
// path. Only decoder-side functions are translated here.

use crate::arch::*;
use crate::celt_lpc::{_celt_autocorr, _celt_lpc, celt_fir};
use crate::util::{OrPanic, zip};

/// SIG_SHIFT: number of fractional bits in celt_sig (fixed-point mode).
/// In float mode the shift is a no-op because shr32 is identity on f32.
const SIG_SHIFT: i32 = 12;

/// Find the two best pitch candidates by comparing normalized cross-correlation
/// against a sliding energy window.
///
/// For each lag i in [0, max_pitch), computes xcorr[i]^2 / Syy (energy of y
/// starting at lag i) and keeps the two best. The comparison is done as
/// cross-multiplied fractions (num * other_den > other_num * den) to avoid
/// division.
///
/// In fixed-point mode, xcorr and Syy are prescaled to avoid overflow in the
/// Q15 multiplies: xcorr is right-shifted by xshift = ilog2(maxcorr)-14,
/// and Syy is accumulated with a yshift right-shift on each MAC.
///
/// C implementation: pitch.c lines 53-106.
///
/// Stateful top-2 tracker with a sliding energy window (`syy` updated as
/// a recurrence by adding `y[i+len]^2` and removing `y[i]^2`); kept as
/// explicit indexed DSP. Indices bounded: i < max_pitch, and the window
/// needs `y` of length len + max_pitch.
#[cfg(not(feature = "fixed-point"))]
#[allow(clippy::indexing_slicing)]
fn find_best_pitch(xcorr: &[OpusVal32], y: &[OpusVal16], len: i32, max_pitch: i32, best_pitch: &mut [i32; 2]) {
    let mut syy: OpusVal32 = 1.0;
    let mut best_num: [OpusVal16; 2] = [-1.0; 2];
    let mut best_den: [OpusVal32; 2] = [0.0; 2];
    best_pitch[0] = 0;
    best_pitch[1] = 1;

    // Initial energy of y[0..len]
    for &sample in &y[..len as usize] {
        syy = mac16_16(syy, sample, sample);
    }

    for i in 0..max_pitch as usize {
        if xcorr[i] > 0.0 {
            let xcorr16 = extract16(vshr32(xcorr[i], 0)); // identity in float
            let num = mult16_16_q15(xcorr16, xcorr16);
            if mult16_32_q15(num, best_den[1]) > mult16_32_q15(best_num[1], syy) {
                if mult16_32_q15(num, best_den[0]) > mult16_32_q15(best_num[0], syy) {
                    best_num[1] = best_num[0];
                    best_den[1] = best_den[0];
                    best_pitch[1] = best_pitch[0];
                    best_num[0] = num;
                    best_den[0] = syy;
                    best_pitch[0] = i as i32;
                } else {
                    best_num[1] = num;
                    best_den[1] = syy;
                    best_pitch[1] = i as i32;
                }
            }
        }
        // Slide energy window: add y[i+len]^2, remove y[i]^2
        let yi_add = y[i + len as usize];
        let yi_rem = y[i];
        syy += shr32(mult16_16(yi_add, yi_add), 0) - shr32(mult16_16(yi_rem, yi_rem), 0);
        syy = max32(1.0, syy);
    }
}

#[cfg(feature = "fixed-point")]
#[allow(clippy::indexing_slicing)]
fn find_best_pitch(
    xcorr: &[OpusVal32],
    y: &[OpusVal16],
    len: i32,
    max_pitch: i32,
    best_pitch: &mut [i32; 2],
    yshift: i32,
    maxcorr: OpusVal32,
) {
    let mut syy: OpusVal32 = 1;
    let mut best_num: [OpusVal16; 2] = [-1; 2];
    let mut best_den: [OpusVal32; 2] = [0; 2];
    best_pitch[0] = 0;
    best_pitch[1] = 1;

    let xshift = celt_ilog2(maxcorr) as i32 - 14;

    // Initial energy of y[0..len]
    for &sample in &y[..len as usize] {
        syy = mac16_16(syy, sample, sample);
    }

    for i in 0..max_pitch as usize {
        if xcorr[i] > 0 {
            let xcorr16 = extract16(vshr32(xcorr[i], xshift));
            let num = mult16_16_q15(xcorr16, xcorr16) as OpusVal16;
            if mult16_32_q15(num, best_den[1]) > mult16_32_q15(best_num[1], syy) {
                if mult16_32_q15(num, best_den[0]) > mult16_32_q15(best_num[0], syy) {
                    best_num[1] = best_num[0];
                    best_den[1] = best_den[0];
                    best_pitch[1] = best_pitch[0];
                    best_num[0] = num;
                    best_den[0] = syy;
                    best_pitch[0] = i as i32;
                } else {
                    best_num[1] = num;
                    best_den[1] = syy;
                    best_pitch[1] = i as i32;
                }
            }
        }
        // Slide energy window: add y[i+len]^2, remove y[i]^2
        let yi_add = y[i + len as usize];
        let yi_rem = y[i];
        syy += shr32(mult16_16(yi_add, yi_add), yshift) - shr32(mult16_16(yi_rem, yi_rem), yshift);
        syy = max32(1, syy);
    }
}

/// Maximum absolute value of a val16 array (fixed-point only).
/// Matches the static inline celt_maxabs16() in mathops.h.
#[cfg(feature = "fixed-point")]
fn celt_maxabs16(x: &[OpusVal16]) -> OpusVal16 {
    x.iter().fold(0, |maxval, &v| max16(maxval, v.abs()))
}

/// Two-pass pitch search: coarse search at 4x decimation, then refined
/// search at 2x decimation, with pseudo-interpolation to choose the
/// final pitch period.
///
/// Called from the CELT PLC path (celt.c:562) after pitch_downsample
/// has produced the half-rate LP-filtered signal.
///
/// C implementation: pitch.c lines 159-265.
pub fn pitch_search(
    x_lp: &[OpusVal16], // LP-filtered signal (len/2 samples from pitch_downsample)
    y: &[OpusVal16],    // decode memory buffer (lag = len + max_pitch samples)
    len: i32,
    max_pitch: i32,
    pitch: &mut i32,
) {
    debug_assert!(len > 0);
    debug_assert!(max_pitch > 0);
    let lag = len + max_pitch;

    // Stack scratch for the 4x-decimated signals and cross-correlation. The
    // CELT PLC is the only caller; its search spans the DECODE_BUFFER_SIZE
    // (2048) decode history, so len, lag and max_pitch all stay within it.
    const DECODE_BUFFER_SIZE: i32 = 2048;
    let mut x_lp4 = [0 as OpusVal16; (DECODE_BUFFER_SIZE >> 2) as usize];
    let x_lp4 = x_lp4.get_mut(..(len >> 2) as usize).or_panic((len >> 2) as usize);
    let mut y_lp4 = [0 as OpusVal16; (DECODE_BUFFER_SIZE >> 2) as usize];
    let y_lp4 = y_lp4.get_mut(..(lag >> 2) as usize).or_panic((lag >> 2) as usize);
    let mut xcorr = [0 as OpusVal32; (DECODE_BUFFER_SIZE >> 2) as usize];
    let xcorr = xcorr.get_mut(..(max_pitch >> 1) as usize).or_panic((max_pitch >> 1) as usize);

    // Downsample by 2 again (from half-rate to quarter-rate)
    for (dst, &src) in zip(x_lp4.iter_mut(), x_lp.iter().step_by(2)) {
        *dst = src;
    }
    for (dst, &src) in zip(y_lp4.iter_mut(), y.iter().step_by(2)) {
        *dst = src;
    }

    // Fixed-point: normalize to prevent overflow in MAC
    #[cfg(feature = "fixed-point")]
    let shift = {
        let max_x = celt_maxabs16(&x_lp4);
        let max_y = celt_maxabs16(&y_lp4);
        let max_val = max16(1, max16(max_x, max_y));
        let s = celt_ilog2(max_val as i32) as i32 - 11;
        if s > 0 {
            for v in x_lp4.iter_mut() {
                *v = shr16(*v, s);
            }
            for v in y_lp4.iter_mut() {
                *v = shr16(*v, s);
            }
            s * 2 // double the shift for MAC accumulation
        } else {
            0
        }
    };
    #[cfg(not(feature = "fixed-point"))]
    let shift: i32 = 0;

    // --- Pass 1: coarse search with 4x decimation ---

    let mut best_pitch = [0i32; 2];

    #[cfg(feature = "fixed-point")]
    let mut maxcorr: OpusVal32 = 1;

    // Cross-correlate at quarter-rate: xcorr[i] = sum_j x_lp4[j] * y_lp4[i+j],
    // accumulated in increasing j (same order as the C, so float is bit-exact).
    let xcorr_len = (max_pitch >> 2) as usize;
    let len4 = (len >> 2) as usize;
    for (i, xc) in xcorr.iter_mut().take(xcorr_len).enumerate() {
        let sum = zip(&*x_lp4, y_lp4.iter().skip(i)).take(len4).fold(0 as OpusVal32, |s, (&a, &b)| mac16_16(s, a, b));
        *xc = max32(-1 as OpusVal32, sum);
        #[cfg(feature = "fixed-point")]
        {
            maxcorr = max32(maxcorr, sum);
        }
    }

    let xcorr1 = xcorr.get(..xcorr_len).or_panic(xcorr_len);
    #[cfg(not(feature = "fixed-point"))]
    find_best_pitch(xcorr1, &y_lp4, len >> 2, max_pitch >> 2, &mut best_pitch);
    #[cfg(feature = "fixed-point")]
    find_best_pitch(xcorr1, &y_lp4, len >> 2, max_pitch >> 2, &mut best_pitch, 0, maxcorr);

    // --- Pass 2: finer search with 2x decimation ---

    #[cfg(feature = "fixed-point")]
    {
        maxcorr = 1;
    }

    // Pass 2 only computes xcorr near the two pass-1 candidates.
    let [bp0, bp1] = best_pitch;
    let half_max_pitch = (max_pitch >> 1) as usize;
    let len2 = (len >> 1) as usize;
    for (i, xc) in xcorr.iter_mut().take(half_max_pitch).enumerate() {
        *xc = 0 as OpusVal32;
        if (i as i32 - 2 * bp0).abs() > 2 && (i as i32 - 2 * bp1).abs() > 2 {
            continue;
        }
        let sum = zip(x_lp, y.iter().skip(i)).take(len2).fold(0 as OpusVal32, |s, (&a, &b)| s + shr32(mult16_16(a, b), shift));
        *xc = max32(-1 as OpusVal32, sum);
        #[cfg(feature = "fixed-point")]
        {
            maxcorr = max32(maxcorr, sum);
        }
    }

    let xcorr2 = xcorr.get(..half_max_pitch).or_panic(half_max_pitch);
    #[cfg(not(feature = "fixed-point"))]
    find_best_pitch(xcorr2, y, len >> 1, max_pitch >> 1, &mut best_pitch);
    #[cfg(feature = "fixed-point")]
    find_best_pitch(xcorr2, y, len >> 1, max_pitch >> 1, &mut best_pitch, shift, maxcorr);

    // --- Refine by pseudo-interpolation ---

    let [winner, _] = best_pitch;
    let mut offset = 0;
    if winner > 0 && winner < (max_pitch >> 1) - 1 {
        // The guard keeps winner-1 ..= winner+1 inside xcorr.
        let w = winner as usize;
        if let (Some(&a), Some(&b), Some(&c)) = (xcorr.get(w - 1), xcorr.get(w), xcorr.get(w + 1)) {
            if (c - a) > mult16_32_q15(qconst16(0.7, 15), b - a) {
                offset = 1;
            } else if (a - c) > mult16_32_q15(qconst16(0.7, 15), b - c) {
                offset = -1;
            }
        }
    }
    *pitch = 2 * winner - offset;
}

/// Downsample and LP-filter a signal for pitch analysis.
///
/// Takes one or two channels of celt_sig samples (`len` samples each) and
/// produces a half-rate, LP-filtered output in `x_lp`.
///
/// The LP filter is a 4th-order all-pole filter (Levinson-Durbin from a
/// short autocorrelation) with exponential coefficient decay (0.9^k),
/// followed by a 1st-order highpass-ish pre-emphasis filter (0.8 at Q12).
///
/// C implementation: pitch.c lines 108-157.
///
/// Strided 3-tap decimation (`x0[2i-1] + 2*x0[2i] + x0[2i+1]`) into the
/// half-length `x_lp`, then autocorrelation / LPC / FIR over `x_lp[..half]`.
/// Kept as explicit indexed DSP.
#[allow(clippy::indexing_slicing)]
pub fn pitch_downsample(
    x: &[&[OpusVal32]],     // channel slices, each `len` samples
    x_lp: &mut [OpusVal16], // output: len/2 samples
    len: i32,
    c_channels: i32,
) {
    let len = len as usize;
    let half = len >> 1;

    // Downsample channel 0 with a 3-tap averaging filter:
    //   x_lp[i] = (x[0][2i-1] + 2*x[0][2i] + x[0][2i+1]) / 4, then >> (SIG_SHIFT+3)
    let x0 = x[0];
    for i in 1..half {
        let val = half32(half32(x0[2 * i - 1] + x0[2 * i + 1]) + x0[2 * i]);
        x_lp[i] = shr32(val, SIG_SHIFT + 3) as OpusVal16;
    }
    // Edge case: i=0 — no x[0][-1], so use (x[0][1]/2 + x[0][0]) / 2
    x_lp[0] = shr32(half32(half32(x0[1]) + x0[0]), SIG_SHIFT + 3) as OpusVal16;

    // If stereo, add channel 1's contribution
    if c_channels == 2 {
        let x1 = x[1];
        for i in 1..half {
            let val = half32(half32(x1[2 * i - 1] + x1[2 * i + 1]) + x1[2 * i]);
            x_lp[i] += shr32(val, SIG_SHIFT + 3) as OpusVal16;
        }
        x_lp[0] += shr32(half32(half32(x1[1]) + x1[0]), SIG_SHIFT + 3) as OpusVal16;
    }

    // Compute 4th-order autocorrelation of the downsampled signal
    let mut ac = [0 as OpusVal32; 5];
    _celt_autocorr(
        &x_lp[..half],
        &mut ac,
        &[], // no window
        0,   // overlap = 0
        4,   // order
    );

    // Noise floor: -40 dB
    #[cfg(not(feature = "fixed-point"))]
    {
        ac[0] *= 1.0001;
    }
    #[cfg(feature = "fixed-point")]
    {
        ac[0] += shr32(ac[0], 13);
    }

    // Lag windowing: ac[i] -= ac[i] * (0.008*i)^2
    // i is the lag number, used in the window weight, not just an index.
    #[allow(clippy::needless_range_loop)]
    for i in 1..=4 {
        #[cfg(not(feature = "fixed-point"))]
        {
            ac[i] -= ac[i] * (0.008 * i as f32) * (0.008 * i as f32);
        }
        #[cfg(feature = "fixed-point")]
        {
            ac[i] -= mult16_32_q15((2 * i * i) as i16, ac[i]);
        }
    }

    // Fit a 4th-order LPC model
    let mut lpc = [0 as OpusVal16; 4];
    _celt_lpc(&mut lpc, &ac, 4);

    // Exponential decay of LPC coefficients: lpc[i] *= 0.9^(i+1)
    let mut tmp: OpusVal16 = Q15ONE;
    for coef in &mut lpc {
        tmp = mult16_16_q15(qconst16(0.9, 15), tmp) as OpusVal16;
        *coef = mult16_16_q15(*coef, tmp) as OpusVal16;
    }

    // Apply the LP filter (all-pole → FIR with these coefficients), in place
    let mut mem = [0 as OpusVal16; 4];
    celt_fir(&mut x_lp[..half], &lpc, 4, &mut mem);

    // Second pass: 1st-order high-pass-ish filter at 0.8 (Q12)
    mem[0] = 0 as OpusVal16;
    let lpc_hp: [OpusVal16; 1] = [qconst16(0.8, 12)];
    celt_fir(&mut x_lp[..half], &lpc_hp, 1, &mut mem);
}
