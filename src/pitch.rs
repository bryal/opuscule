// Translated from c/celt/pitch.c (RFC 6716).
//
// Pitch analysis routines used by the CELT PLC (packet loss concealment)
// path. Only decoder-side functions are translated here.

use crate::arch::*;
use crate::celt_lpc::{_celt_autocorr, _celt_lpc, celt_fir};

/// SIG_SHIFT: number of fractional bits in celt_sig (fixed-point mode).
/// In float mode the shift is a no-op because shr32 is identity on f32.
const SIG_SHIFT: i32 = 12;

#[cfg(not(feature = "fixed-point"))]
#[inline(always)]
fn qconst16(x: f32, _bits: i32) -> f32 {
    x
}

#[cfg(feature = "fixed-point")]
#[inline(always)]
fn qconst16(x: f32, bits: i32) -> i16 {
    (x * ((1 << bits) as f32) + 0.5) as i16
}

/// Downsample and LP-filter a signal for pitch analysis.
///
/// Takes one or two channels of celt_sig samples (C channels, `len` samples
/// each) and produces a half-rate, LP-filtered output in `x_lp`.
///
/// The LP filter is a 4th-order all-pole filter (Levinson-Durbin from a
/// short autocorrelation) with exponential coefficient decay (0.9^k),
/// followed by a 1st-order highpass-ish pre-emphasis filter (0.8 at Q12).
///
/// C implementation: pitch.c lines 108-157.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn pitch_downsample(
    x: *const *const OpusVal32,   // C channel pointers, each `len` samples
    x_lp: *mut OpusVal16,         // output: len/2 samples
    len: i32,
    c_channels: i32,
) {
    let len = len as usize;
    let half = len >> 1;

    // Downsample channel 0 with a 3-tap averaging filter:
    //   x_lp[i] = (x[0][2i-1] + 2*x[0][2i] + x[0][2i+1]) / 4, then >> (SIG_SHIFT+3)
    let x0 = *x;
    for i in 1..half {
        let val = half32(half32(*x0.add(2 * i - 1) + *x0.add(2 * i + 1)) + *x0.add(2 * i));
        *x_lp.add(i) = shr32(val, SIG_SHIFT + 3) as OpusVal16;
    }
    // Edge case: i=0 — no x[0][-1], so use (x[0][1]/2 + x[0][0]) / 2
    *x_lp = shr32(half32(half32(*x0.add(1)) + *x0), SIG_SHIFT + 3) as OpusVal16;

    // If stereo, add channel 1's contribution
    if c_channels == 2 {
        let x1 = *x.add(1);
        for i in 1..half {
            let val = half32(half32(*x1.add(2 * i - 1) + *x1.add(2 * i + 1)) + *x1.add(2 * i));
            let prev = *x_lp.add(i);
            *x_lp.add(i) = prev + shr32(val, SIG_SHIFT + 3) as OpusVal16;
        }
        let prev = *x_lp;
        *x_lp = prev + shr32(half32(half32(*x1.add(1)) + *x1), SIG_SHIFT + 3) as OpusVal16;
    }

    // Compute 4th-order autocorrelation of the downsampled signal
    let mut ac = [0 as OpusVal32; 5];
    _celt_autocorr(
        x_lp,
        ac.as_mut_ptr(),
        core::ptr::null(),   // no window
        0,                   // overlap = 0
        4,                   // order
        half as i32,         // n
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
    _celt_lpc(lpc.as_mut_ptr(), ac.as_mut_ptr(), 4);

    // Exponential decay of LPC coefficients: lpc[i] *= 0.9^(i+1)
    let mut tmp: OpusVal16 = Q15ONE;
    for i in 0..4 {
        tmp = mult16_16_q15(qconst16(0.9, 15), tmp) as OpusVal16;
        lpc[i] = mult16_16_q15(lpc[i], tmp) as OpusVal16;
    }

    // Apply the LP filter (all-pole → FIR with these coefficients)
    let mut mem = [0 as OpusVal16; 4];
    celt_fir(x_lp, lpc.as_ptr(), x_lp, half as i32, 4, mem.as_mut_ptr());

    // Second pass: 1st-order high-pass-ish filter at 0.8 (Q12)
    mem[0] = 0 as OpusVal16;
    let lpc_hp: [OpusVal16; 1] = [qconst16(0.8, 12)];
    celt_fir(x_lp, lpc_hp.as_ptr(), x_lp, half as i32, 1, mem.as_mut_ptr());
}
