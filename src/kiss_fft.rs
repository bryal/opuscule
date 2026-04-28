// Translated from c/celt/kiss_fft.c and c/celt/_kiss_fft_guts.h (RFC 6716).
//
// Complex FFT types, complex arithmetic helpers, and inverse butterfly
// functions (ki_bfly2/3/4/5) used by the inverse FFT (opus_ifft).
//
// Only the inverse (decoder) path is translated. Forward butterflies
// (kf_bfly*) and opus_fft were encoder-only and have been removed.

use crate::arch::*;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

/// Complex number using kiss_fft_scalar (= OpusVal32) components.
/// Matches C's kiss_fft_cpx: { kiss_fft_scalar r, i; }
#[repr(C)]
#[derive(Copy, Clone)]
pub struct KissFftCpx {
    pub r: OpusVal32,
    pub i: OpusVal32,
}

/// Complex twiddle factor using kiss_twiddle_scalar (= OpusVal16) components.
/// Matches C's kiss_twiddle_cpx: { kiss_twiddle_scalar r, i; }
#[repr(C)]
#[derive(Copy, Clone)]
pub struct KissTwiddleCpx {
    pub r: OpusVal16,
    pub i: OpusVal16,
}
impl KissTwiddleCpx {
    pub const fn from_array([r, i]: [OpusVal16; 2]) -> Self {
        Self { r, i }
    }
}

pub const MAXFACTORS: usize = 8;

/// FFT state struct. Matches C's kiss_fft_state in kiss_fft.h.
///
/// In fixed-point mode there is no `scale` field — the C struct uses
/// #ifndef FIXED_POINT around it. We handle this with cfg.
#[repr(C)]
pub struct KissFftState {
    pub nfft: i32,
    #[cfg(not(feature = "fixed-point"))]
    pub scale: OpusVal32,
    pub shift: i32,
    pub factors: [i16; 2 * MAXFACTORS],
    pub bitrev: *const i16,
    pub twiddles: *const KissTwiddleCpx,
}

// ---------------------------------------------------------------------------
// S_MUL: scalar × twiddle multiply
// ---------------------------------------------------------------------------

/// In float mode: plain multiply.
/// In fixed-point mode: MULT16_32_Q15(b, a) — note the argument swap;
/// `a` is the fft_scalar (i32) and `b` is the twiddle_scalar (i16).
#[cfg(not(feature = "fixed-point"))]
#[inline(always)]
pub fn s_mul(a: OpusVal32, b: OpusVal16) -> OpusVal32 {
    a * b
}

#[cfg(feature = "fixed-point")]
#[inline(always)]
pub fn s_mul(a: OpusVal32, b: OpusVal16) -> OpusVal32 {
    mult16_32_q15(b, a)
}

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (from _kiss_fft_guts.h)
// ---------------------------------------------------------------------------

/// C_MULC(m, a, b): complex multiply by conjugate of b.
///   Float:  m.r = a.r*b.r + a.i*b.i;  m.i = a.i*b.r - a.r*b.i
///   Fixed:  same but using S_MUL (MULT16_32_Q15) with ADD32/SUB32
#[inline(always)]
pub fn c_mulc(a: KissFftCpx, b: KissTwiddleCpx) -> KissFftCpx {
    KissFftCpx { r: s_mul(a.r, b.r) + s_mul(a.i, b.i), i: s_mul(a.i, b.r) - s_mul(a.r, b.i) }
}

/// C_ADD(a, b): component-wise addition.
#[inline(always)]
pub fn c_add(a: KissFftCpx, b: KissFftCpx) -> KissFftCpx {
    KissFftCpx { r: a.r + b.r, i: a.i + b.i }
}

/// C_SUB(a, b): component-wise subtraction.
#[inline(always)]
pub fn c_sub(a: KissFftCpx, b: KissFftCpx) -> KissFftCpx {
    KissFftCpx { r: a.r - b.r, i: a.i - b.i }
}

/// C_ADDTO(res, a): res += a (in-place).
#[inline(always)]
pub fn c_addto(res: &mut KissFftCpx, a: KissFftCpx) {
    res.r += a.r;
    res.i += a.i;
}

/// C_MULBYSCALAR(c, s): multiply both components by scalar s.
///   Float:  c.r *= s;  c.i *= s
///   Fixed:  c.r = S_MUL(c.r, s);  c.i = S_MUL(c.i, s)
#[inline(always)]
pub fn c_mulbyscalar(c: &mut KissFftCpx, s: OpusVal16) {
    c.r = s_mul(c.r, s);
    c.i = s_mul(c.i, s);
}

/// HALF_OF(x): in float mode = x * 0.5; in fixed-point mode = x >> 1.
#[cfg(not(feature = "fixed-point"))]
#[inline(always)]
pub fn half_of(x: OpusVal32) -> OpusVal32 {
    x * 0.5
}

#[cfg(feature = "fixed-point")]
#[inline(always)]
pub fn half_of(x: OpusVal32) -> OpusVal32 {
    x >> 1
}

// ---------------------------------------------------------------------------
// Inverse butterfly functions
// ---------------------------------------------------------------------------

/// Radix-2 inverse butterfly.
/// C: ki_bfly2() in kiss_fft.c
pub fn ki_bfly2(fout: &mut [KissFftCpx], fstride: usize, st: &KissFftState, m: usize, n: usize, mm: usize) {
    for i in 0..n {
        let base = i * mm;
        let twiddles = st.twiddles;
        for j in 0..m {
            let tw = unsafe { *twiddles.add(j * fstride) };
            let t = c_mulc(fout[base + m + j], tw);
            fout[base + m + j] = c_sub(fout[base + j], t);
            c_addto(&mut fout[base + j], t);
        }
    }
}

/// Radix-4 inverse butterfly.
/// C: ki_bfly4() in kiss_fft.c
pub fn ki_bfly4(fout: &mut [KissFftCpx], fstride: usize, st: &KissFftState, m: usize, n: usize, mm: usize) {
    let m2 = 2 * m;
    let m3 = 3 * m;

    for i in 0..n {
        let base = i * mm;
        let twiddles = st.twiddles;
        for j in 0..m {
            let tw1 = unsafe { *twiddles.add(j * fstride) };
            let tw2 = unsafe { *twiddles.add(j * fstride * 2) };
            let tw3 = unsafe { *twiddles.add(j * fstride * 3) };
            let mut scratch = [KissFftCpx { r: 0 as OpusVal32, i: 0 as OpusVal32 }; 6];

            scratch[0] = c_mulc(fout[base + j + m], tw1);
            scratch[1] = c_mulc(fout[base + j + m2], tw2);
            scratch[2] = c_mulc(fout[base + j + m3], tw3);

            scratch[5] = c_sub(fout[base + j], scratch[1]);
            c_addto(&mut fout[base + j], scratch[1]);
            scratch[3] = c_add(scratch[0], scratch[2]);
            scratch[4] = c_sub(scratch[0], scratch[2]);
            fout[base + j + m2] = c_sub(fout[base + j], scratch[3]);
            c_addto(&mut fout[base + j], scratch[3]);

            // Inverse butterfly: note the sign pattern differs from forward
            fout[base + j + m].r = scratch[5].r - scratch[4].i;
            fout[base + j + m].i = scratch[5].i + scratch[4].r;
            fout[base + j + m3].r = scratch[5].r + scratch[4].i;
            fout[base + j + m3].i = scratch[5].i - scratch[4].r;
        }
    }
}

/// Radix-3 inverse butterfly.
/// C: ki_bfly3() in kiss_fft.c
/// Only compiled when RADIX_TWO_ONLY is not defined (which is the default).
pub fn ki_bfly3(fout: &mut [KissFftCpx], fstride: usize, st: &KissFftState, m: usize, n: usize, mm: usize) {
    let m2 = 2 * m;
    let epi3 = unsafe { *st.twiddles.add(fstride * m) };

    for i in 0..n {
        let base = i * mm;
        let twiddles = st.twiddles;
        for j in 0..m {
            let tw1 = unsafe { *twiddles.add(j * fstride) };
            let tw2 = unsafe { *twiddles.add(j * fstride * 2) };
            let mut scratch = [KissFftCpx { r: 0 as OpusVal32, i: 0 as OpusVal32 }; 6];

            scratch[1] = c_mulc(fout[base + j + m], tw1);
            scratch[2] = c_mulc(fout[base + j + m2], tw2);

            scratch[3] = c_add(scratch[1], scratch[2]);
            scratch[0] = c_sub(scratch[1], scratch[2]);

            fout[base + j + m].r = fout[base + j].r - half_of(scratch[3].r);
            fout[base + j + m].i = fout[base + j].i - half_of(scratch[3].i);

            c_mulbyscalar(&mut scratch[0], -epi3.i);

            c_addto(&mut fout[base + j], scratch[3]);

            fout[base + j + m2].r = fout[base + j + m].r + scratch[0].i;
            fout[base + j + m2].i = fout[base + j + m].i - scratch[0].r;

            fout[base + j + m].r -= scratch[0].i;
            fout[base + j + m].i += scratch[0].r;
        }
    }
}

/// Radix-5 inverse butterfly.
/// C: ki_bfly5() in kiss_fft.c
/// Only compiled when RADIX_TWO_ONLY is not defined (which is the default).
pub fn ki_bfly5(fout: &mut [KissFftCpx], fstride: usize, st: &KissFftState, m: usize, n: usize, mm: usize) {
    let twiddles = st.twiddles;
    let ya = unsafe { *twiddles.add(fstride * m) };
    let yb = unsafe { *twiddles.add(fstride * 2 * m) };

    for i in 0..n {
        let base = i * mm;
        for u in 0..m {
            let mut scratch = [KissFftCpx { r: 0 as OpusVal32, i: 0 as OpusVal32 }; 13];

            scratch[0] = fout[base + u];
            scratch[1] = c_mulc(fout[base + u + m], unsafe { *twiddles.add(u * fstride) });
            scratch[2] = c_mulc(fout[base + u + 2 * m], unsafe { *twiddles.add(2 * u * fstride) });
            scratch[3] = c_mulc(fout[base + u + 3 * m], unsafe { *twiddles.add(3 * u * fstride) });
            scratch[4] = c_mulc(fout[base + u + 4 * m], unsafe { *twiddles.add(4 * u * fstride) });

            scratch[7] = c_add(scratch[1], scratch[4]);
            scratch[10] = c_sub(scratch[1], scratch[4]);
            scratch[8] = c_add(scratch[2], scratch[3]);
            scratch[9] = c_sub(scratch[2], scratch[3]);

            fout[base + u].r += scratch[7].r + scratch[8].r;
            fout[base + u].i += scratch[7].i + scratch[8].i;

            scratch[5].r = scratch[0].r + s_mul(scratch[7].r, ya.r) + s_mul(scratch[8].r, yb.r);
            scratch[5].i = scratch[0].i + s_mul(scratch[7].i, ya.r) + s_mul(scratch[8].i, yb.r);

            scratch[6].r = -s_mul(scratch[10].i, ya.i) - s_mul(scratch[9].i, yb.i);
            scratch[6].i = s_mul(scratch[10].r, ya.i) + s_mul(scratch[9].r, yb.i);

            fout[base + u + m] = c_sub(scratch[5], scratch[6]);
            fout[base + u + 4 * m] = c_add(scratch[5], scratch[6]);

            scratch[11].r = scratch[0].r + s_mul(scratch[7].r, yb.r) + s_mul(scratch[8].r, ya.r);
            scratch[11].i = scratch[0].i + s_mul(scratch[7].i, yb.r) + s_mul(scratch[8].i, ya.r);
            scratch[12].r = s_mul(scratch[10].i, yb.i) - s_mul(scratch[9].i, ya.i);
            scratch[12].i = -s_mul(scratch[10].r, yb.i) + s_mul(scratch[9].r, ya.i);

            fout[base + u + 2 * m] = c_add(scratch[11], scratch[12]);
            fout[base + u + 3 * m] = c_sub(scratch[11], scratch[12]);
        }
    }
}

// ---------------------------------------------------------------------------
// Inverse FFT entry point
// ---------------------------------------------------------------------------

/// Inverse FFT. Replaces C's opus_ifft() in kiss_fft.c.
///
/// Performs an out-of-place inverse complex FFT: bit-reverses the input
/// into the output buffer, then applies butterfly stages in reverse order
/// (smallest radix first, working up to the full FFT size).
///
/// The factorisation is pre-computed in st.factors[] as (radix, remaining)
/// pairs. For example, an N=120 FFT might factor as 4×30, then 30 = 2×15,
/// 15 = 3×5, 5 = 5×1 — giving factors = [4,30, 2,15, 3,5, 5,1].
/// The butterfly stages are applied in reverse: bfly5, bfly3, bfly2, bfly4.
pub unsafe fn opus_ifft(st: *const KissFftState, fin: *const KissFftCpx, fout: *mut KissFftCpx) {
    unsafe {
        let st = &*st;
        let nfft = st.nfft as usize;
        let shift = if st.shift > 0 { st.shift as usize } else { 0 };

        // Bit-reverse the input into the output buffer
        let fin_slice = std::slice::from_raw_parts(fin, nfft);
        let fout_slice = std::slice::from_raw_parts_mut(fout, nfft);
        for i in 0..nfft {
            let rev = *st.bitrev.add(i) as usize;
            fout_slice[rev] = fin_slice[i];
        }

        // Build fstride table and count stages
        let mut fstride = [0usize; MAXFACTORS];
        fstride[0] = 1;
        let mut l = 0usize;
        loop {
            let _p = st.factors[2 * l] as usize;
            let m = st.factors[2 * l + 1] as usize;
            fstride[l + 1] = fstride[l] * _p;
            l += 1;
            if m == 1 {
                break;
            }
        }

        // Apply butterfly stages in reverse order
        let mut m = st.factors[2 * l - 1] as usize;
        for i in (0..l).rev() {
            let m2 = if i != 0 { st.factors[2 * i - 1] as usize } else { 1 };
            match st.factors[2 * i] {
                2 => ki_bfly2(fout_slice, fstride[i] << shift, st, m, fstride[i], m2),
                4 => ki_bfly4(fout_slice, fstride[i] << shift, st, m, fstride[i], m2),
                3 => ki_bfly3(fout_slice, fstride[i] << shift, st, m, fstride[i], m2),
                5 => ki_bfly5(fout_slice, fstride[i] << shift, st, m, fstride[i], m2),
                _ => {}
            }
            m = m2;
        }
    } // unsafe
}
