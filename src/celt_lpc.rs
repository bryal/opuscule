// Translated from c/celt/celt_lpc.c (RFC 6716).
//
// LPC analysis and filtering functions used by the CELT decoder's PLC
// (packet loss concealment) path and pitch analysis. Four functions:
//
// - _celt_lpc: Levinson-Durbin LPC coefficient computation from autocorrelation
// - celt_fir: FIR filter (used for pitch pre-filtering)
// - celt_iir: IIR filter (used for PLC synthesis)
// - _celt_autocorr: Windowed autocorrelation

use std::os::raw::c_int;

use crate::arch::*;

/// Maximum autocorrelation length (MAX_PERIOD from modes.h).
const MAX_PERIOD: usize = 1024;

/// LPC_ORDER from celt_lpc.h.
pub const LPC_ORDER: usize = 24;

/// Levinson-Durbin recursion: compute LPC coefficients from autocorrelation.
///
/// Computes `p` LPC coefficients from `p+1` autocorrelation values.
/// Bails out early if prediction gain exceeds ~30 dB (error < ac[0]/1024
/// in fixed-point, error < 0.001*ac[0] in float).
pub unsafe fn _celt_lpc(_lpc: *mut OpusVal16, ac: *const OpusVal32, p: c_int) {
    unsafe {
        let p = p as usize;

        // In fixed-point mode, work in a local i32 buffer then round to i16.
        // In float mode, _lpc and lpc alias the same memory (matching C).
        #[cfg(feature = "fixed-point")]
        let mut lpc_buf = [0i32; LPC_ORDER];
        #[cfg(feature = "fixed-point")]
        let lpc = lpc_buf.as_mut_ptr();
        #[cfg(not(feature = "fixed-point"))]
        let lpc = _lpc;

        for i in 0..p {
            *lpc.add(i) = 0 as OpusVal32;
        }

        let mut error = *ac;
        if *ac != 0 as OpusVal32 {
            for i in 0..p {
                // Sum up this iteration's reflection coefficient
                let mut rr: OpusVal32 = 0 as OpusVal32;
                for j in 0..i {
                    rr = rr + mult32_32_q31(*lpc.add(j), *ac.add(i - j));
                }
                rr = rr + shr32(*ac.add(i + 1), 3);
                let r = -frac_div32(shl32(rr, 3), error);
                // Update LPC coefficients and total error
                *lpc.add(i) = shr32(r, 3);
                for j in 0..(i + 1) >> 1 {
                    let tmp1 = *lpc.add(j);
                    let tmp2 = *lpc.add(i - 1 - j);
                    *lpc.add(j) = tmp1 + mult32_32_q31(r, tmp2);
                    *lpc.add(i - 1 - j) = tmp2 + mult32_32_q31(r, tmp1);
                }

                error = error - mult32_32_q31(mult32_32_q31(r, r), error);
                // Bail out once we get 30 dB gain
                #[cfg(feature = "fixed-point")]
                {
                    if error < shr32(*ac, 10) {
                        break;
                    }
                }
                #[cfg(not(feature = "fixed-point"))]
                {
                    if error < 0.001f32 * *ac {
                        break;
                    }
                }
            }
        }

        #[cfg(feature = "fixed-point")]
        {
            for i in 0..p {
                *_lpc.add(i) = round16(*lpc.add(i), 16);
            }
        }
    }
}

/// FIR filter with memory.
///
/// Applies an `ord`-tap FIR filter to `N` input samples, writing to `y`.
/// `mem` holds `ord` state samples, updated in-place (shift register).
pub unsafe fn celt_fir(
    x: *const OpusVal16,
    num: *const OpusVal16,
    y: *mut OpusVal16,
    n: c_int,
    ord: c_int,
    mem: *mut OpusVal16,
) {
    unsafe {
        let n = n as usize;
        let ord = ord as usize;
        for i in 0..n {
            let mut sum: OpusVal32 = shl32(extend32(*x.add(i)), SIG_SHIFT);
            for j in 0..ord {
                sum = sum + mult16_16(*num.add(j), *mem.add(j));
            }
            for j in (1..ord).rev() {
                *mem.add(j) = *mem.add(j - 1);
            }
            *mem = *x.add(i);
            *y.add(i) = round16(sum, SIG_SHIFT);
        }
    }
}

/// IIR filter with memory.
///
/// Applies an `ord`-tap IIR filter to `N` input samples, writing to `y`.
/// `mem` holds `ord` state samples, updated in-place (shift register).
/// Used in the PLC path to resynthesize audio from LPC coefficients.
pub unsafe fn celt_iir(
    x: *const OpusVal32,
    den: *const OpusVal16,
    y: *mut OpusVal32,
    n: c_int,
    ord: c_int,
    mem: *mut OpusVal16,
) {
    unsafe {
        let n = n as usize;
        let ord = ord as usize;
        for i in 0..n {
            let mut sum: OpusVal32 = *x.add(i);
            for j in 0..ord {
                sum = sum - mult16_16(*den.add(j), *mem.add(j));
            }
            for j in (1..ord).rev() {
                *mem.add(j) = *mem.add(j - 1);
            }
            *mem = round16(sum, SIG_SHIFT);
            *y.add(i) = sum;
        }
    }
}

/// Windowed autocorrelation.
///
/// Computes autocorrelation of `x[0..n-1]` for lags 0 through `lag`,
/// applying `window[0..overlap-1]` symmetrically to the edges.
/// In fixed-point mode, normalizes to prevent overflow.
/// Adds a small bias (+10) to ac[0] to avoid division by zero.
pub unsafe fn _celt_autocorr(
    x: *const OpusVal16,
    ac: *mut OpusVal32,
    window: *const OpusVal16,
    overlap: c_int,
    lag: c_int,
    n: c_int,
) {
    unsafe {
        let n = n as usize;
        let overlap = overlap as usize;
        let mut lag = lag as usize;

        debug_assert!(n > 0);
        debug_assert!(n <= MAX_PERIOD);

        // Copy x into local buffer, apply window to edges
        let mut xx = [0 as OpusVal16; MAX_PERIOD];
        for i in 0..n {
            xx[i] = *x.add(i);
        }
        for i in 0..overlap {
            xx[i] = mult16_16_q15(*x.add(i), *window.add(i)) as OpusVal16;
            xx[n - i - 1] = mult16_16_q15(*x.add(n - i - 1), *window.add(i)) as OpusVal16;
        }

        // Fixed-point normalization to prevent overflow in the dot products
        #[cfg(feature = "fixed-point")]
        {
            let mut ac0: i32 = 0;
            for i in 0..n {
                ac0 += shr32(mult16_16(xx[i], xx[i]), 9);
            }
            ac0 += 1 + n as i32;

            let shift = (celt_ilog2(ac0) as i32 - 30 + 10 + 1) / 2;
            for i in 0..n {
                xx[i] = vshr32(xx[i] as i32, shift) as i16;
            }
        }

        // Compute autocorrelation for each lag (lag down to 0)
        loop {
            let mut d: OpusVal32 = 0 as OpusVal32;
            for i in lag..n {
                d = d + mult16_16(xx[i], xx[i - lag]);
            }
            *ac.add(lag) = d;
            if lag == 0 {
                break;
            }
            lag -= 1;
        }

        // Bias to avoid division by zero
        *ac = *ac + 10 as OpusVal32;
    }
}

/// SIG_SHIFT constant from arch.h — controls the Q format for signals.
const SIG_SHIFT: i32 = 12;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_celt_fir_identity() {
        // Single-tap FIR with coefficient 1.0 should pass input through
        let x: [OpusVal16; 4] = [1.0, 2.0, 3.0, 4.0];
        let num: [OpusVal16; 1] = [1.0];
        let mut y: [OpusVal16; 4] = [0.0; 4];
        let mut mem: [OpusVal16; 1] = [0.0];

        unsafe {
            celt_fir(x.as_ptr(), num.as_ptr(), y.as_mut_ptr(), 4, 1, mem.as_mut_ptr());
        }
        // FIR output is: y[i] = x[i] + num[0]*mem[0], where mem shifts
        // y[0] = 1.0 + 1.0*0.0 = 1.0  (mem was 0)
        // y[1] = 2.0 + 1.0*1.0 = 3.0  (mem = x[0] = 1.0)
        // y[2] = 3.0 + 1.0*2.0 = 5.0  (mem = x[1] = 2.0)
        // y[3] = 4.0 + 1.0*3.0 = 7.0  (mem = x[2] = 3.0)
        assert_eq!(y, [1.0, 3.0, 5.0, 7.0]);
    }

    #[test]
    fn test_celt_iir_dc() {
        // DC input with zero denominator → output equals input
        let x: [OpusVal32; 4] = [1.0, 1.0, 1.0, 1.0];
        let den: [OpusVal16; 1] = [0.0];
        let mut y: [OpusVal32; 4] = [0.0; 4];
        let mut mem: [OpusVal16; 1] = [0.0];

        unsafe {
            celt_iir(x.as_ptr(), den.as_ptr(), y.as_mut_ptr(), 4, 1, mem.as_mut_ptr());
        }
        assert_eq!(y, [1.0, 1.0, 1.0, 1.0]);
    }

    #[test]
    fn test_autocorr_impulse() {
        // Autocorrelation of an impulse: ac[0] should dominate, ac[1..] ≈ 0
        let mut x = [0.0 as OpusVal16; 64];
        x[0] = 1.0;
        let mut ac = [0.0 as OpusVal32; 5];

        unsafe {
            _celt_autocorr(x.as_ptr(), ac.as_mut_ptr(), std::ptr::null(), 0, 4, 64);
        }
        // ac[0] = 1.0 + 10 (bias), ac[1..4] = 0.0
        assert!((ac[0] - 11.0).abs() < 1e-6);
        for i in 1..5 {
            assert!((ac[i]).abs() < 1e-6, "ac[{}] = {} should be ~0", i, ac[i]);
        }
    }

    #[test]
    fn test_autocorr_constant() {
        // Autocorrelation of a constant signal: all lags should be equal
        // (minus the missing overlap at higher lags)
        let n = 16;
        let x = [1.0 as OpusVal16; 16];
        let mut ac = [0.0 as OpusVal32; 3];

        unsafe {
            _celt_autocorr(x.as_ptr(), ac.as_mut_ptr(), std::ptr::null(), 0, 2, n);
        }
        // ac[0] = 16 + 10 (bias) = 26, ac[1] = 15, ac[2] = 14
        assert!((ac[0] - 26.0).abs() < 1e-6);
        assert!((ac[1] - 15.0).abs() < 1e-6);
        assert!((ac[2] - 14.0).abs() < 1e-6);
    }

    #[test]
    fn test_celt_lpc_from_constant_autocorr() {
        // For a white noise signal, ac = [1, 0, 0, ...] → all LPC coeffs ≈ 0
        let mut ac = [0.0 as OpusVal32; 5];
        ac[0] = 1.0;
        // ac[1..4] = 0 → no correlation
        let mut lpc = [0.0 as OpusVal16; 4];

        unsafe {
            _celt_lpc(lpc.as_mut_ptr(), ac.as_ptr(), 4);
        }
        for i in 0..4 {
            assert!((lpc[i]).abs() < 1e-6, "lpc[{}] = {} should be ~0 for white noise", i, lpc[i]);
        }
    }
}
