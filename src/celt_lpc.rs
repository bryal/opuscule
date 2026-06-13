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
pub fn _celt_lpc(_lpc: &mut [OpusVal16], ac: &[OpusVal32], p: c_int) {
    let p = p as usize;

    // Work in an OpusVal32 buffer through the recursion (matches C, which
    // either reuses the caller's array directly in float mode or stages
    // into a local i32 buffer in fixed-point mode). At the end we narrow
    // back to OpusVal16 for the caller — a no-op for the float build
    // (where OpusVal16 == OpusVal32 == f32) and a round16 in fixed.
    let mut lpc = [0 as OpusVal32; LPC_ORDER];

    let mut error = ac[0];
    if ac[0] != 0 as OpusVal32 {
        for i in 0..p {
            // Sum up this iteration's reflection coefficient
            let mut rr: OpusVal32 = 0 as OpusVal32;
            for j in 0..i {
                rr = rr + mult32_32_q31(lpc[j], ac[i - j]);
            }
            rr = rr + shr32(ac[i + 1], 3);
            let r = -frac_div32(shl32(rr, 3), error);
            // Update LPC coefficients and total error
            lpc[i] = shr32(r, 3);
            for j in 0..(i + 1) >> 1 {
                let tmp1 = lpc[j];
                let tmp2 = lpc[i - 1 - j];
                lpc[j] = tmp1 + mult32_32_q31(r, tmp2);
                lpc[i - 1 - j] = tmp2 + mult32_32_q31(r, tmp1);
            }

            error = error - mult32_32_q31(mult32_32_q31(r, r), error);
            // Bail out once we get 30 dB gain
            #[cfg(feature = "fixed-point")]
            {
                if error < shr32(ac[0], 10) {
                    break;
                }
            }
            #[cfg(not(feature = "fixed-point"))]
            {
                if error < 0.001f32 * ac[0] {
                    break;
                }
            }
        }
    }

    #[cfg(feature = "fixed-point")]
    for i in 0..p {
        _lpc[i] = round16(lpc[i], 16);
    }
    #[cfg(not(feature = "fixed-point"))]
    for i in 0..p {
        _lpc[i] = lpc[i];
    }
}

/// FIR filter with memory, in place.
///
/// Applies an `ord`-tap FIR filter over the whole of `x`, writing the
/// result back into `x`. `mem` holds `ord` state samples, updated
/// in-place (shift register).
///
/// The C version takes separate input/output pointers plus a length, but
/// every caller passes the same buffer for both and the length is just
/// its length; each iteration reads its sample before writing it and the
/// history comes from `mem`, so in-place over the slice is exactly
/// equivalent and the Rust version makes the aliasing explicit.
pub fn celt_fir(x: &mut [OpusVal16], num: &[OpusVal16], ord: c_int, mem: &mut [OpusVal16]) {
    let ord = ord as usize;
    for xi in x {
        let input = *xi;
        let mut sum: OpusVal32 = shl32(extend32(input), SIG_SHIFT);
        for j in 0..ord {
            sum = sum + mult16_16(num[j], mem[j]);
        }
        for j in (1..ord).rev() {
            mem[j] = mem[j - 1];
        }
        mem[0] = input;
        *xi = round16(sum, SIG_SHIFT);
    }
}

/// IIR filter with memory, in place.
///
/// Applies an `ord`-tap IIR filter over the whole of `x`, writing the
/// result back into `x`. `mem` holds `ord` state samples, updated
/// in-place (shift register). Used in the PLC path to resynthesize audio
/// from LPC coefficients. Callers that work on a prefix of a larger
/// buffer slice it at the call.
///
/// In-place for the same reason as [`celt_fir`].
pub fn celt_iir(x: &mut [OpusVal32], den: &[OpusVal16], ord: c_int, mem: &mut [OpusVal16]) {
    let ord = ord as usize;
    for xi in x {
        let mut sum: OpusVal32 = *xi;
        for j in 0..ord {
            sum = sum - mult16_16(den[j], mem[j]);
        }
        for j in (1..ord).rev() {
            mem[j] = mem[j - 1];
        }
        mem[0] = round16(sum, SIG_SHIFT);
        *xi = sum;
    }
}

/// Windowed autocorrelation.
///
/// Computes autocorrelation of `x[0..n-1]` for lags 0 through `lag`,
/// applying `window[0..overlap-1]` symmetrically to the edges
/// (`window` may be empty when `overlap` is 0).
/// In fixed-point mode, normalizes to prevent overflow.
/// Adds a small bias (+10) to ac[0] to avoid division by zero.
pub fn _celt_autocorr(x: &[OpusVal16], ac: &mut [OpusVal32], window: &[OpusVal16], overlap: c_int, lag: c_int, n: c_int) {
    let n = n as usize;
    let overlap = overlap as usize;
    let mut lag = lag as usize;

    debug_assert!(n > 0);
    debug_assert!(n <= MAX_PERIOD);

    // Copy x into local buffer, apply window to edges
    let mut xx = [0 as OpusVal16; MAX_PERIOD];
    xx[..n].copy_from_slice(&x[..n]);
    for i in 0..overlap {
        xx[i] = mult16_16_q15(x[i], window[i]) as OpusVal16;
        xx[n - i - 1] = mult16_16_q15(x[n - i - 1], window[i]) as OpusVal16;
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
        ac[lag] = d;
        if lag == 0 {
            break;
        }
        lag -= 1;
    }

    // Bias to avoid division by zero
    ac[0] = ac[0] + 10 as OpusVal32;
}

/// SIG_SHIFT constant from arch.h — controls the Q format for signals.
const SIG_SHIFT: i32 = 12;

#[cfg(test)]
mod tests {
    use super::*;

    // --- Float-mode tests ---

    #[cfg(not(feature = "fixed-point"))]
    #[test]
    fn test_celt_fir_identity() {
        // Single-tap FIR with coefficient 1.0 should pass input through
        let mut x: [OpusVal16; 4] = [1.0, 2.0, 3.0, 4.0];
        let num: [OpusVal16; 1] = [1.0];
        let mut mem: [OpusVal16; 1] = [0.0];

        celt_fir(&mut x, &num, 1, &mut mem);
        // FIR output is: x[i] = x[i] + num[0]*mem[0], where mem shifts
        // x[0] = 1.0 + 1.0*0.0 = 1.0  (mem was 0)
        // x[1] = 2.0 + 1.0*1.0 = 3.0  (mem = old x[0] = 1.0)
        // x[2] = 3.0 + 1.0*2.0 = 5.0  (mem = old x[1] = 2.0)
        // x[3] = 4.0 + 1.0*3.0 = 7.0  (mem = old x[2] = 3.0)
        assert_eq!(x, [1.0, 3.0, 5.0, 7.0]);
    }

    #[cfg(not(feature = "fixed-point"))]
    #[test]
    fn test_celt_iir_dc() {
        // DC input with zero denominator → output equals input
        let mut x: [OpusVal32; 4] = [1.0, 1.0, 1.0, 1.0];
        let den: [OpusVal16; 1] = [0.0];
        let mut mem: [OpusVal16; 1] = [0.0];

        celt_iir(&mut x, &den, 1, &mut mem);
        assert_eq!(x, [1.0, 1.0, 1.0, 1.0]);
    }

    #[cfg(not(feature = "fixed-point"))]
    #[test]
    fn test_autocorr_impulse() {
        // Autocorrelation of an impulse: ac[0] should dominate, ac[1..] ≈ 0
        let mut x = [0.0 as OpusVal16; 64];
        x[0] = 1.0;
        let mut ac = [0.0 as OpusVal32; 5];

        _celt_autocorr(&x, &mut ac, &[], 0, 4, 64);
        // ac[0] = 1.0 + 10 (bias), ac[1..4] = 0.0
        assert!((ac[0] - 11.0).abs() < 1e-6);
        for i in 1..5 {
            assert!((ac[i]).abs() < 1e-6, "ac[{}] = {} should be ~0", i, ac[i]);
        }
    }

    #[cfg(not(feature = "fixed-point"))]
    #[test]
    fn test_autocorr_constant() {
        // Autocorrelation of a constant signal: all lags should be equal
        // (minus the missing overlap at higher lags)
        let n = 16;
        let x = [1.0 as OpusVal16; 16];
        let mut ac = [0.0 as OpusVal32; 3];

        _celt_autocorr(&x, &mut ac, &[], 0, 2, n);
        // ac[0] = 16 + 10 (bias) = 26, ac[1] = 15, ac[2] = 14
        assert!((ac[0] - 26.0).abs() < 1e-6);
        assert!((ac[1] - 15.0).abs() < 1e-6);
        assert!((ac[2] - 14.0).abs() < 1e-6);
    }

    #[cfg(not(feature = "fixed-point"))]
    #[test]
    fn test_celt_lpc_from_constant_autocorr() {
        // For a white noise signal, ac = [1, 0, 0, ...] → all LPC coeffs ≈ 0
        let mut ac = [0.0 as OpusVal32; 5];
        ac[0] = 1.0;
        let mut lpc = [0.0 as OpusVal16; 4];

        _celt_lpc(&mut lpc, &ac, 4);
        for i in 0..4 {
            assert!((lpc[i]).abs() < 1e-6, "lpc[{}] = {} should be ~0 for white noise", i, lpc[i]);
        }
    }

    // --- Fixed-point tests ---
    // In fixed-point mode: OpusVal16 = i16, OpusVal32 = i32.
    // celt_fir: sum = (x[i] as i32) << SIG_SHIFT + num[j]*mem[j]; y[i] = (sum+2048)>>12 as i16
    // celt_iir: sum = x[i] - den[j]*mem[j]; mem = round16(sum, 12); y[i] = sum
    // _celt_autocorr: normalizes, then computes dot products, adds +10 bias

    #[cfg(feature = "fixed-point")]
    #[test]
    fn test_celt_fir_zero_coeff() {
        // FIR with zero coefficient: x[i] = round16(x[i] << 12, 12) = x[i]
        let mut x: [OpusVal16; 4] = [10, 20, 30, 40];
        let num: [OpusVal16; 1] = [0];
        let mut mem: [OpusVal16; 1] = [0];

        celt_fir(&mut x, &num, 1, &mut mem);
        assert_eq!(x, [10, 20, 30, 40]);
    }

    #[cfg(feature = "fixed-point")]
    #[test]
    fn test_celt_iir_zero_denom() {
        // IIR with zero denominator: sum = x[i], output = x[i]
        let mut x: [OpusVal32; 4] = [1000, 2000, 3000, 4000];
        let den: [OpusVal16; 1] = [0];
        let mut mem: [OpusVal16; 1] = [0];

        celt_iir(&mut x, &den, 1, &mut mem);
        assert_eq!(x, [1000, 2000, 3000, 4000]);
    }

    #[cfg(feature = "fixed-point")]
    #[test]
    fn test_autocorr_zero_input() {
        // Autocorrelation of all-zero input: only the +10 bias survives
        let x = [0i16; 64];
        let mut ac = [0i32; 5];

        _celt_autocorr(&x, &mut ac, &[], 0, 4, 64);
        assert_eq!(ac[0], 10);
        for i in 1..5 {
            assert_eq!(ac[i], 0, "ac[{}] = {} should be 0", i, ac[i]);
        }
    }

    #[cfg(feature = "fixed-point")]
    #[test]
    fn test_autocorr_ordering() {
        // For a constant input, ac[0] >= ac[1] >= ac[2] (decreasing with lag)
        let x = [100i16; 16];
        let mut ac = [0i32; 3];

        _celt_autocorr(&x, &mut ac, &[], 0, 2, 16);
        assert!(ac[0] > ac[1], "ac[0]={} should exceed ac[1]={}", ac[0], ac[1]);
        assert!(ac[1] > ac[2], "ac[1]={} should exceed ac[2]={}", ac[1], ac[2]);
        assert!(ac[2] > 0, "ac[2]={} should be positive", ac[2]);
    }

    #[cfg(feature = "fixed-point")]
    #[test]
    fn test_celt_lpc_white_noise() {
        // White noise: ac = [large, 0, 0, 0] → all LPC coefficients should be 0
        let mut ac = [0i32; 5];
        ac[0] = 32767;
        let mut lpc = [0i16; 4];

        _celt_lpc(&mut lpc, &ac, 4);
        for i in 0..4 {
            assert_eq!(lpc[i], 0, "lpc[{}] = {} should be 0 for white noise", i, lpc[i]);
        }
    }
}
