// Translated from c/celt/mdct.c (RFC 6716).
//
// MDCT (Modified Discrete Cosine Transform) implementation using an N/4
// complex FFT. Only the inverse (backward) transform is needed for decoding.
//
// The algorithm: pre-rotate → inverse FFT → post-rotate → de-shuffle →
// window and overlap-add (TDAC). See Fabrice Bellard's FFMPEG MDCT for
// the original inspiration, though signs, ordering and scaling differ.

use std::os::raw::c_int;
use crate::arch::*;

/// S_MUL: scalar multiply. In float mode it's just a*b.
/// In fixed-point mode it's MULT16_32_Q15(b, a) — note the swap:
/// `a` is the fft_scalar (i32) and `b` is the twiddle_scalar (i16).
#[cfg(not(feature = "fixed-point"))]
#[inline(always)]
fn s_mul(a: OpusVal32, b: OpusVal16) -> OpusVal32 {
    a * b
}

#[cfg(feature = "fixed-point")]
#[inline(always)]
fn s_mul(a: OpusVal32, b: OpusVal16) -> OpusVal32 {
    mult16_32_q15(b, a)
}

// -- FFI types matching the C structs --

/// kiss_fft_cpx: complex number with (r, i) components.
/// kiss_fft_scalar = opus_val32 in both modes.
#[repr(C)]
#[derive(Copy, Clone)]
pub struct KissFftCpx {
    pub r: OpusVal32,
    pub i: OpusVal32,
}

/// Opaque kiss_fft_state — we only pass pointers to C's opus_ifft.
#[repr(C)]
pub struct KissFftState {
    _opaque: [u8; 0],
}

/// mdct_lookup: the MDCT state struct. Matches C's mdct_lookup in mdct.h.
#[repr(C)]
pub struct MdctLookup {
    pub n: c_int,
    pub maxshift: c_int,
    pub kfft: [*const KissFftState; 4],
    pub trig: *const OpusVal16,
}

unsafe extern "C" {
    fn opus_ifft(
        cfg: *const KissFftState,
        fin: *const KissFftCpx,
        fout: *mut KissFftCpx,
    );
}

/// Inverse MDCT (decoder path).
///
/// Transforms N frequency-domain coefficients back to N time-domain samples,
/// applying the synthesis window and overlap-add (TDAC) with the previous
/// frame's output.
///
/// The `out` pointer is adjusted internally to account for the non-overlapping
/// region: the caller passes the start of the full N-sample output buffer,
/// and this function writes to `out - (N/2 - overlap)/2` through
/// `out + N - 1 - (N/2 - overlap)/2`.
///
/// C implementation: mdct.c clt_mdct_backward(), lines 217-237.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn clt_mdct_backward(
    l: *const MdctLookup,
    inp: *mut OpusVal32,          // frequency-domain input (N/2 strided values)
    out: *mut OpusVal32,          // time-domain output (overlap-add target)
    window: *const OpusVal16,     // synthesis window (overlap samples)
    overlap: c_int,
    shift: c_int,
    stride: c_int,
) {
    unsafe {
    let l = &*l;
    let mut n = l.n;
    n >>= shift;
    let n2 = n >> 1;
    let n4 = n >> 2;

    // Allocate scratch buffers (N/2 scalars each = N/4 complex)
    let mut f = vec![0 as OpusVal32; n2 as usize];
    let mut f2 = vec![0 as OpusVal32; n2 as usize];

    // sin(pi/4N) approximation: sin(x) ≈ x for small x
    #[cfg(not(feature = "fixed-point"))]
    let sine: OpusVal16 = 2.0 * std::f32::consts::PI * 0.125 / n as f32;
    #[cfg(feature = "fixed-point")]
    let sine: OpusVal16 = {
        // TRIG_UPSCALE * (QCONST16(0.7853981, 15) + N2) / N
        // TRIG_UPSCALE = 1 in the default fixed-point build
        let q = (0.7853981 * 32768.0 + 0.5) as i32;  // QCONST16(0.7853981, 15)
        ((q + n2) / n) as i16
    };

    // --- Pre-rotate ---
    // Combine pairs of real input coefficients into complex values,
    // applying twiddle factors from the trig table.
    {
        let mut xp1 = inp;
        let mut xp2 = inp.add((stride * (n2 - 1)) as usize);
        let trig = l.trig;

        for i in 0..n4 as usize {
            let x1 = *xp1;
            let x2 = *xp2;
            let t_i = *trig.add(i << shift as usize);
            let t_ni = *trig.add((n4 as usize - i) << shift as usize);

            let yr = -s_mul(x2, t_i) + s_mul(x1, t_ni);
            let yi = -s_mul(x2, t_ni) - s_mul(x1, t_i);
            // Rotation by sine ≈ pi/(4N): compensates for not having
            // exact twiddles at half-sample offsets.
            f2[2 * i] = yr - s_mul(yi, sine);
            f2[2 * i + 1] = yi + s_mul(yr, sine);

            xp1 = xp1.add(2 * stride as usize);
            xp2 = xp2.sub(2 * stride as usize);
        }
    }

    // --- Inverse N/4 complex FFT ---
    // This should NOT downscale even in fixed-point.
    opus_ifft(
        l.kfft[shift as usize],
        f2.as_ptr() as *const KissFftCpx,
        f.as_mut_ptr() as *mut KissFftCpx,
    );

    // --- Post-rotate ---
    // Apply twiddle factors to convert back from frequency to time domain.
    {
        let trig = l.trig;
        for i in 0..n4 as usize {
            let re = f[2 * i];
            let im = f[2 * i + 1];
            let t_i = *trig.add(i << shift as usize);
            let t_ni = *trig.add((n4 as usize - i) << shift as usize);

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
        let mut fp1 = 0usize;        // index into f[]
        let mut fp2 = n2 as usize - 1;  // index into f[], counting down
        for i in 0..n4 as usize {
            f2[2 * i] = -f[fp1];
            f2[2 * i + 1] = f[fp2];
            fp1 += 2;
            fp2 = fp2.wrapping_sub(2);
        }
    }

    // Adjust output pointer for TDAC alignment
    let out = out.sub(((n2 - overlap) >> 1) as usize);

    // --- Mirror + overlap-add (TDAC) ---
    // First half: copy non-overlapping region, then window and overlap-add
    {
        let mut fp1 = n4 as usize - 1;   // index into f2[], counting down
        let mut xp1 = (n2 - 1) as usize; // index into out[], counting down
        let mut yp1 = (n4 - overlap / 2) as usize; // index into out[], counting up

        // Non-overlapping region: direct copy
        for _ in 0..(n4 - overlap / 2) as usize {
            *out.add(xp1) = f2[fp1];
            xp1 = xp1.wrapping_sub(1);
            fp1 = fp1.wrapping_sub(1);
        }

        // Overlapping region: window and accumulate
        let mut wi = 0usize;
        for _ in (n4 - overlap / 2) as usize..n4 as usize {
            let x1 = f2[fp1];
            fp1 = fp1.wrapping_sub(1);
            let w1 = *window.add(wi);
            let w2 = *window.add((overlap as usize - 1) - wi);
            *out.add(yp1) += -mult16_32_q15(w1, x1);
            *out.add(xp1) += mult16_32_q15(w2, x1);
            yp1 += 1;
            xp1 = xp1.wrapping_sub(1);
            wi += 1;
        }
    }

    // Second half: same pattern, forward direction
    {
        let mut fp2 = n4 as usize;       // index into f2[], counting up
        let mut xp2 = n2 as usize;       // index into out[], counting up
        let mut yp2 = (n - 1 - (n4 - overlap / 2)) as usize; // counting down

        // Non-overlapping region: direct copy
        for _ in 0..(n4 - overlap / 2) as usize {
            *out.add(xp2) = f2[fp2];
            xp2 += 1;
            fp2 += 1;
        }

        // Overlapping region: window and write
        let mut wi = 0usize;
        for _ in (n4 - overlap / 2) as usize..n4 as usize {
            let x2 = f2[fp2];
            fp2 += 1;
            let w1 = *window.add(wi);
            let w2 = *window.add((overlap as usize - 1) - wi);
            *out.add(yp2) = mult16_32_q15(w1, x2);
            *out.add(xp2) = mult16_32_q15(w2, x2);
            yp2 = yp2.wrapping_sub(1);
            xp2 += 1;
            wi += 1;
        }
    }
    } // unsafe
}
