// Translated from c/celt/mdct.c (RFC 6716).
//
// MDCT (Modified Discrete Cosine Transform) implementation using an N/4
// complex FFT. Only the inverse (backward) transform is needed for decoding.
//
// The algorithm: pre-rotate → inverse FFT → post-rotate → de-shuffle →
// window and overlap-add (TDAC). See Fabrice Bellard's FFMPEG MDCT for
// the original inspiration, though signs, ordering and scaling differ.

use crate::arch::*;
use crate::kiss_fft::{KissFftCpx, KissFftState, opus_ifft, s_mul};
use core::ffi::c_int;

/// mdct_lookup: the MDCT state struct. Matches C's mdct_lookup in mdct.h.
pub struct MdctLookup {
    pub n: c_int,
    pub maxshift: c_int,
    pub kfft: [&'static KissFftState; 4],
    pub trig: &'static [Val],
}

/// Inverse MDCT (decoder path).
///
/// Transforms N frequency-domain coefficients back to N time-domain samples,
/// applying the synthesis window and overlap-add (TDAC) with the previous
/// frame's output.
///
/// The C version takes an `out` pointer which it adjusts *backwards* by
/// `(N/2 - overlap)/2` before writing. Since slices can't index before
/// their start, the Rust version takes the whole output buffer plus
/// `out_off`, the offset the C caller would have passed as its pointer;
/// the back-adjustment is folded into a base index.
///
/// C implementation: mdct.c clt_mdct_backward(), lines 217-237.
///
/// This is a strided FFT/MDCT butterfly with bidirectional cursors
/// (pre/post-rotate, de-shuffle, and the TDAC overlap-add that scatters
/// into `out` counting both up and down from `base`). Every index is
/// governed by the transform sizes — `i < n4`, the `f`/`f2` scratch
/// buffers are `n2 = 2*n4` long, `trig` is sized for the mode, and the
/// `out` writes stay within the current frame — so it stays as indexed
/// butterfly math rather than a contrived iterator form.
#[allow(clippy::indexing_slicing)]
pub fn clt_mdct_backward(
    l: &MdctLookup,
    inp: &[Wal],     // frequency-domain input (N/2 strided values)
    out: &mut [Wal], // time-domain output buffer (overlap-add target)
    out_off: usize,  // logical start within `out` (pre-adjustment)
    window: &[Val],  // synthesis window (overlap samples)
    overlap: c_int,
    shift: c_int,
    stride: c_int,
) {
    let mut n = l.n;
    n >>= shift;
    let n2 = n >> 1;
    let n4 = n >> 2;

    // Stack scratch (N/2 scalars each = N/4 complex), sliced to the current
    // half-size. MAX_N2 = l.n (1920 for the standard mode) >> 1; `shift` only
    // shrinks it.
    const MAX_N2: usize = 960;
    let mut f = [0 as Wal; MAX_N2];
    let f = &mut f[..n2 as usize];
    let mut f2 = [0 as Wal; MAX_N2];
    let f2 = &mut f2[..n2 as usize];

    // sin(pi/4N) approximation: sin(x) ≈ x for small x
    #[cfg(not(feature = "fixed-point"))]
    let sine: Val = 2.0 * core::f32::consts::PI * 0.125 / n as f32;
    #[cfg(feature = "fixed-point")]
    let sine: Val = {
        // TRIG_UPSCALE * (QCONST16(0.7853981, 15) + N2) / N
        // TRIG_UPSCALE = 1 in the default fixed-point build
        // The 0.7853981 literal mirrors the C QCONST16 argument verbatim.
        #[allow(clippy::approx_constant)]
        let q = (0.7853981 * 32768.0 + 0.5) as i32; // QCONST16(0.7853981, 15)
        ((q + n2) / n) as i16
    };

    // --- Pre-rotate ---
    // Combine pairs of real input coefficients into complex values,
    // applying twiddle factors from the trig table.
    {
        let mut xp1 = 0usize; // scalar index into inp
        let mut xp2 = (stride * (n2 - 1)) as usize;
        let trig = l.trig;

        for i in 0..n4 as usize {
            let x1 = inp[xp1];
            let x2 = inp[xp2];
            let t_i = trig[i << shift as usize];
            let t_ni = trig[(n4 as usize - i) << shift as usize];

            let yr = -s_mul(x2, t_i) + s_mul(x1, t_ni);
            let yi = -s_mul(x2, t_ni) - s_mul(x1, t_i);
            // Rotation by sine ≈ pi/(4N): compensates for not having
            // exact twiddles at half-sample offsets.
            f2[2 * i] = yr - s_mul(yi, sine);
            f2[2 * i + 1] = yi + s_mul(yr, sine);

            xp1 += 2 * stride as usize;
            xp2 = xp2.wrapping_sub(2 * stride as usize);
        }
    }

    // --- Inverse N/4 complex FFT ---
    // This should NOT downscale even in fixed-point. opus_ifft works on
    // complex slices, while f/f2 are scalar buffers holding interleaved
    // (re, im) pairs, so bridge through dedicated complex scratch: each
    // adjacent scalar pair maps to one complex value. MAX_N4 = MAX_N2 / 2.
    const MAX_N4: usize = MAX_N2 / 2;
    let nfft = (n2 / 2) as usize;
    let mut fft_in = [KissFftCpx { r: 0 as Wal, i: 0 as Wal }; MAX_N4];
    let mut fft_out = [KissFftCpx { r: 0 as Wal, i: 0 as Wal }; MAX_N4];
    let fft_in = &mut fft_in[..nfft];
    let fft_out = &mut fft_out[..nfft];
    for (c, pair) in fft_in.iter_mut().zip(f2.chunks_exact(2)) {
        *c = KissFftCpx { r: pair[0], i: pair[1] };
    }
    opus_ifft(l.kfft[shift as usize], fft_in, fft_out);
    for (c, pair) in fft_out.iter().zip(f.chunks_exact_mut(2)) {
        pair[0] = c.r;
        pair[1] = c.i;
    }

    // --- Post-rotate ---
    // Apply twiddle factors to convert back from frequency to time domain.
    {
        let trig = l.trig;
        for i in 0..n4 as usize {
            let re = f[2 * i];
            let im = f[2 * i + 1];
            let t_i = trig[i << shift as usize];
            let t_ni = trig[(n4 as usize - i) << shift as usize];

            // We'd scale up by 2 here, but instead it's done when mixing the windows
            let yr = s_mul(re, t_i) - s_mul(im, t_ni);
            let yi = s_mul(im, t_i) + s_mul(re, t_ni);
            f[2 * i] = yr - s_mul(yi, sine);
            f[2 * i + 1] = yi + s_mul(yr, sine);
        }
    }

    // --- De-shuffle ---
    // Rearrange complex output into real interleaved format for the
    // middle (non-overlapping) portion of the window.
    {
        let mut fp1 = 0usize; // index into f[]
        let mut fp2 = n2 as usize - 1; // index into f[], counting down
        for i in 0..n4 as usize {
            f2[2 * i] = -f[fp1];
            f2[2 * i + 1] = f[fp2];
            fp1 += 2;
            fp2 = fp2.wrapping_sub(2);
        }
    }

    // TDAC alignment: the C does `out -= (n2 - overlap) >> 1` and then
    // indexes forward; fold that into a base index instead.
    let base = out_off - (((n2 - overlap) >> 1) as usize);

    // --- Mirror + overlap-add (TDAC) ---
    // First half: copy non-overlapping region, then window and overlap-add
    {
        let mut fp1 = n4 as usize - 1; // index into f2[], counting down
        let mut xp1 = (n2 - 1) as usize; // index into out[], counting down
        let mut yp1 = (n4 - overlap / 2) as usize; // index into out[], counting up

        // Non-overlapping region: direct copy
        for _ in 0..(n4 - overlap / 2) as usize {
            out[base + xp1] = f2[fp1];
            xp1 = xp1.wrapping_sub(1);
            fp1 = fp1.wrapping_sub(1);
        }

        // Overlapping region: window and accumulate
        for wi in 0..overlap as usize / 2 {
            let x1 = f2[fp1];
            fp1 = fp1.wrapping_sub(1);
            let w1 = window[wi];
            let w2 = window[(overlap as usize - 1) - wi];
            out[base + yp1] += -mult16_32_q15(w1, x1);
            out[base + xp1] += mult16_32_q15(w2, x1);
            yp1 += 1;
            xp1 = xp1.wrapping_sub(1);
        }
    }

    // Second half: same pattern, forward direction
    {
        let mut fp2 = n4 as usize; // index into f2[], counting up
        let mut xp2 = n2 as usize; // index into out[], counting up
        let mut yp2 = (n - 1 - (n4 - overlap / 2)) as usize; // counting down

        // Non-overlapping region: direct copy
        for _ in 0..(n4 - overlap / 2) as usize {
            out[base + xp2] = f2[fp2];
            xp2 += 1;
            fp2 += 1;
        }

        // Overlapping region: window and write
        for wi in 0..overlap as usize / 2 {
            let x2 = f2[fp2];
            fp2 += 1;
            let w1 = window[wi];
            let w2 = window[(overlap as usize - 1) - wi];
            out[base + yp2] = mult16_32_q15(w1, x2);
            out[base + xp2] = mult16_32_q15(w2, x2);
            yp2 = yp2.wrapping_sub(1);
            xp2 += 1;
        }
    }
}
