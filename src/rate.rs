// Translated from c/celt/rate.c and c/celt/rate.h (RFC 6716).
//
// Bit allocation for the CELT layer. compute_allocation() distributes
// available bits across frequency bands, deciding how many go to PVQ
// pulses vs fine energy for each band.
//
// The allocation is done jointly for encoder and decoder (both must
// agree exactly). The `encode` parameter selects which side's entropy
// coder calls to make, but the allocation logic is identical.
//
// See RFC 6716 Section 4.3.3.

use std::os::raw::c_int;

use crate::entcode::{BITRES, ec_ctx};
use crate::entdec::{ec_dec_bit_logp, ec_dec_uint};

// -- Constants from rate.h --

const MAX_FINE_BITS: i32 = 8;
const FINE_OFFSET: i32 = 21;
const QTHETA_OFFSET: i32 = 4;
const QTHETA_OFFSET_TWOPHASE: i32 = 16;
const ALLOC_STEPS: i32 = 6;
const LOG_MAX_PSEUDO: i32 = 6;

static LOG2_FRAC_TABLE: [u8; 24] =
    [0, 8, 13, 16, 19, 21, 23, 24, 26, 27, 28, 29, 30, 31, 32, 32, 33, 34, 34, 35, 36, 36, 37, 37];

// -- Inline helpers from rate.h --

/// Expand pseudo-pulse count to actual pulse count.
#[inline]
pub fn get_pulses(i: i32) -> i32 {
    if i < 8 { i } else { (8 + (i & 7)) << ((i >> 3) - 1) }
}

/// Convert bits to pseudo-pulse count via binary search of the pulse cache.
///
/// # Safety
/// `m` must point to a valid CELTMode with initialized cache.
#[inline]
pub unsafe fn bits2pulses(m: *const CELTMode, band: i32, lm: i32, bits: i32) -> i32 {
    let m = unsafe { &*m };
    let lm1 = lm + 1;
    let cache_idx = unsafe { *m.cache.index.add((lm1 * m.nb_ebands + band) as usize) };
    let cache = unsafe { m.cache.bits.add(cache_idx as usize) };
    let mut lo = 0i32;
    let mut hi = unsafe { *cache } as i32;
    let bits = bits - 1;
    for _ in 0..LOG_MAX_PSEUDO {
        let mid = (lo + hi + 1) >> 1;
        if unsafe { *cache.add(mid as usize) } as i32 >= bits {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let lo_val = if lo == 0 { -1 } else { (unsafe { *cache.add(lo as usize) }) as i32 };
    if bits - lo_val <= unsafe { *cache.add(hi as usize) } as i32 - bits { lo } else { hi }
}

/// Convert pseudo-pulse count to bits via the pulse cache.
///
/// # Safety
/// `m` must point to a valid CELTMode with initialized cache.
#[inline]
pub unsafe fn pulses2bits(m: *const CELTMode, band: i32, lm: i32, pulses: i32) -> i32 {
    let m = unsafe { &*m };
    let lm1 = lm + 1;
    let cache_idx = unsafe { *m.cache.index.add((lm1 * m.nb_ebands + band) as usize) };
    let cache = unsafe { m.cache.bits.add(cache_idx as usize) };
    if pulses == 0 { 0 } else { (unsafe { *cache.add(pulses as usize) }) as i32 + 1 }
}

// -- C struct definitions (repr(C) to match layout) --

/// Pulse cache from modes.h.
#[repr(C)]
pub struct PulseCache {
    pub size: c_int,
    pub index: *const i16,
    pub bits: *const u8,
    pub caps: *const u8,
}

/// Opaque placeholder for mdct_lookup to maintain struct layout.
/// mdct_lookup = { int n, int maxshift, const kiss_fft_state *kfft[4],
///                 const kiss_twiddle_scalar *trig }
#[repr(C)]
struct MdctLookupOpaque {
    _n: c_int,
    _maxshift: c_int,
    _kfft: [*const u8; 4],
    _trig: *const u8,
}

/// CELTMode struct from modes.h (OpusCustomMode).
/// We only access the fields needed by rate/band allocation; the mdct
/// and window fields are opaque padding to maintain correct layout.
///
/// Note: the `preemph` field is opus_val16[4], which is f32[4] in
/// float mode and i16[4] in fixed-point mode. We use cfg to match.
#[repr(C)]
pub struct OpusCustomMode {
    pub fs: i32,
    pub overlap: c_int,
    pub nb_ebands: c_int,
    pub eff_ebands: c_int,
    #[cfg(not(feature = "fixed-point"))]
    _preemph: [f32; 4],
    #[cfg(feature = "fixed-point")]
    _preemph: [i16; 4],
    pub ebands: *const i16,
    pub max_lm: c_int,
    pub nb_short_mdcts: c_int,
    pub short_mdct_size: c_int,
    pub nb_alloc_vectors: c_int,
    pub alloc_vectors: *const u8,
    pub log_n: *const i16,
    #[cfg(not(feature = "fixed-point"))]
    pub window: *const f32,
    #[cfg(feature = "fixed-point")]
    pub window: *const i16,
    _mdct: MdctLookupOpaque,
    pub cache: PulseCache,
}

/// Alias used in C code.
pub type CELTMode = OpusCustomMode;

// Helpers matching C macros
#[inline]
fn imin(a: i32, b: i32) -> i32 {
    a.min(b)
}
#[inline]
fn imax(a: i32, b: i32) -> i32 {
    a.max(b)
}

/// Read an i16 from a pointer at offset, as i32.
///
/// # Safety
/// Pointer + offset must be valid.
#[inline]
unsafe fn eb(ebands: *const i16, i: i32) -> i32 {
    (unsafe { *ebands.add(i as usize) }) as i32
}

/// Inner bit allocation: interpolate between two allocation vectors,
/// decide which bands to skip, and distribute bits to PVQ vs fine energy.
///
/// # Safety
/// All pointer parameters must be valid. `m` must point to a valid CELTMode.
#[allow(clippy::too_many_arguments)]
unsafe fn interp_bits2pulses(
    m: *const CELTMode,
    start: i32,
    end: i32,
    skip_start: i32,
    bits1: *const i32,
    bits2: *const i32,
    thresh: *const i32,
    cap: *const i32,
    mut total: i32,
    balance_out: *mut i32,
    skip_rsv: i32,
    intensity: *mut i32,
    mut intensity_rsv: i32,
    dual_stereo: *mut i32,
    dual_stereo_rsv: i32,
    bits: *mut i32,
    ebits: *mut i32,
    fine_priority: *mut i32,
    c: i32,
    lm: i32,
    ec: *mut ec_ctx,
    encode: i32,
    prev: i32,
) -> i32 {
    let m_ref = unsafe { &*m };
    let bitres = BITRES as i32;
    let alloc_floor = c << bitres;
    let stereo = (c > 1) as i32;
    let log_m = lm << bitres;

    // Bisection to find interpolation factor
    let mut lo = 0i32;
    let mut hi = 1 << ALLOC_STEPS;
    for _ in 0..ALLOC_STEPS {
        let mid = (lo + hi) >> 1;
        let mut psum = 0i32;
        let mut done = false;
        for j in (start..end).rev() {
            let ju = j as usize;
            let tmp = unsafe { *bits1.add(ju) } + ((mid as i64 * unsafe { *bits2.add(ju) } as i64) >> ALLOC_STEPS) as i32;
            if tmp >= unsafe { *thresh.add(ju) } || done {
                done = true;
                psum += imin(tmp, unsafe { *cap.add(ju) });
            } else if tmp >= alloc_floor {
                psum += alloc_floor;
            }
        }
        if psum > total {
            hi = mid;
        } else {
            lo = mid;
        }
    }

    // Compute final allocation with the chosen interpolation factor
    let mut psum = 0i32;
    let mut done = false;
    for j in (start..end).rev() {
        let ju = j as usize;
        let mut tmp = unsafe { *bits1.add(ju) } + (lo * unsafe { *bits2.add(ju) } >> ALLOC_STEPS);
        if tmp < unsafe { *thresh.add(ju) } && !done {
            tmp = if tmp >= alloc_floor { alloc_floor } else { 0 };
        } else {
            done = true;
        }
        tmp = imin(tmp, unsafe { *cap.add(ju) });
        unsafe { *bits.add(ju) = tmp };
        psum += tmp;
    }

    // Decide which bands to skip, working backwards from the end
    let mut coded_bands = end;
    loop {
        let j = coded_bands - 1;
        if j <= skip_start {
            total += skip_rsv;
            break;
        }
        let left = total - psum;
        let percoeff = left / (unsafe { eb(m_ref.ebands, coded_bands) } - unsafe { eb(m_ref.ebands, start) });
        let left_rem = left - (unsafe { eb(m_ref.ebands, coded_bands) } - unsafe { eb(m_ref.ebands, start) }) * percoeff;
        let rem = imax(left_rem - (unsafe { eb(m_ref.ebands, j) } - unsafe { eb(m_ref.ebands, start) }), 0);
        let band_width = unsafe { eb(m_ref.ebands, coded_bands) } - unsafe { eb(m_ref.ebands, j) };
        let mut band_bits = unsafe { *bits.add(j as usize) } + percoeff * band_width + rem;

        if band_bits >= imax(unsafe { *thresh.add(j as usize) }, alloc_floor + (1 << bitres)) {
            if encode != 0 {
                // Encoder path -- skip decision signaling
                // We translate this minimally: the encoder would call
                // ec_enc_bit_logp here. Since we're decode-only, this
                // path is unreachable in practice, but must compile for
                // the shared function signature.
                let threshold = if j < prev { 7 } else { 9 };
                if band_bits > (threshold * band_width << lm << bitres) >> 4 {
                    // ec_enc_bit_logp(ec, 1, 1)
                    break;
                }
                // ec_enc_bit_logp(ec, 0, 1)
            } else if unsafe { ec_dec_bit_logp(ec, 1) } != 0 {
                break;
            }
            psum += 1 << bitres;
            band_bits -= 1 << bitres;
        }
        psum -= unsafe { *bits.add(j as usize) } + intensity_rsv;
        if intensity_rsv > 0 {
            intensity_rsv = LOG2_FRAC_TABLE[(j - start) as usize] as i32;
        }
        psum += intensity_rsv;
        if band_bits >= alloc_floor {
            psum += alloc_floor;
            unsafe { *bits.add(j as usize) = alloc_floor };
        } else {
            unsafe { *bits.add(j as usize) = 0 };
        }
        coded_bands -= 1;
    }

    debug_assert!(coded_bands > start);

    // Code the intensity and dual stereo parameters
    if intensity_rsv > 0 {
        if encode != 0 {
            unsafe {
                *intensity = imin(*intensity, coded_bands);
                // ec_enc_uint(ec, *intensity - start, coded_bands + 1 - start)
            }
        } else {
            unsafe {
                *intensity = start + ec_dec_uint(ec, (coded_bands + 1 - start) as u32) as i32;
            }
        }
    } else {
        unsafe { *intensity = 0 };
    }

    let mut dual_stereo_rsv = dual_stereo_rsv;
    if unsafe { *intensity } <= start {
        total += dual_stereo_rsv;
        dual_stereo_rsv = 0;
    }

    if dual_stereo_rsv > 0 {
        if encode != 0 {
            // ec_enc_bit_logp(ec, *dual_stereo, 1)
        } else {
            unsafe { *dual_stereo = ec_dec_bit_logp(ec, 1) };
        }
    } else {
        unsafe { *dual_stereo = 0 };
    }

    // Allocate the remaining bits
    let left = total - psum;
    let percoeff = left / (unsafe { eb(m_ref.ebands, coded_bands) } - unsafe { eb(m_ref.ebands, start) });
    let mut left = left - (unsafe { eb(m_ref.ebands, coded_bands) } - unsafe { eb(m_ref.ebands, start) }) * percoeff;

    for j in start..coded_bands {
        let bw = unsafe { eb(m_ref.ebands, j + 1) } - unsafe { eb(m_ref.ebands, j) };
        unsafe { *bits.add(j as usize) += percoeff * bw };
    }
    for j in start..coded_bands {
        let bw = unsafe { eb(m_ref.ebands, j + 1) } - unsafe { eb(m_ref.ebands, j) };
        let tmp = imin(left, bw);
        unsafe { *bits.add(j as usize) += tmp };
        left -= tmp;
    }

    // Distribute bits between fine energy and PVQ for each band
    let mut balance = 0i32;
    let mut j = start;
    while j < coded_bands {
        let n0 = unsafe { eb(m_ref.ebands, j + 1) } - unsafe { eb(m_ref.ebands, j) };
        let n = n0 << lm;
        unsafe { *bits.add(j as usize) += balance };

        let excess;
        if n > 1 {
            excess = imax(unsafe { *bits.add(j as usize) } - unsafe { *cap.add(j as usize) }, 0);
            unsafe { *bits.add(j as usize) -= excess };

            let den = c * n + if c == 2 && n > 2 && unsafe { *dual_stereo } == 0 && j < unsafe { *intensity } { 1 } else { 0 };

            let nc_log_n = den * (unsafe { *m_ref.log_n.add(j as usize) } as i32 + log_m);
            let mut offset = (nc_log_n >> 1) - den * FINE_OFFSET;
            if n == 2 {
                offset += den << bitres >> 2;
            }

            let bj = unsafe { *bits.add(j as usize) };
            if bj + offset < den * 2 << bitres {
                offset += nc_log_n >> 2;
            } else if bj + offset < den * 3 << bitres {
                offset += nc_log_n >> 3;
            }

            let mut eb_j = imax(0, (bj + offset + (den << (bitres - 1))) / (den << bitres));
            if c * eb_j > (bj >> bitres) {
                eb_j = bj >> stereo >> bitres;
            }
            eb_j = imin(eb_j, MAX_FINE_BITS);

            unsafe {
                *fine_priority.add(j as usize) = (eb_j * (den << bitres) >= bj + offset) as i32;
                *bits.add(j as usize) -= c * eb_j << bitres;
                *ebits.add(j as usize) = eb_j;
            }
        } else {
            // For N=1, all bits go to fine energy except for a single sign bit
            excess = imax(0, unsafe { *bits.add(j as usize) } - (c << bitres));
            unsafe {
                *bits.add(j as usize) -= excess;
                *ebits.add(j as usize) = 0;
                *fine_priority.add(j as usize) = 1;
            }
        }

        // Fine energy can't take advantage of re-balancing in quant_all_bands().
        // Instead, do the re-balancing here.
        let mut excess = excess;
        if excess > 0 {
            let extra_fine = imin(excess >> (stereo + bitres), MAX_FINE_BITS - unsafe { *ebits.add(j as usize) });
            unsafe { *ebits.add(j as usize) += extra_fine };
            let extra_bits = extra_fine * c << bitres;
            unsafe {
                *fine_priority.add(j as usize) = (extra_bits >= excess - balance) as i32;
            }
            excess -= extra_bits;
        }
        balance = excess;

        debug_assert!(unsafe { *bits.add(j as usize) } >= 0);
        debug_assert!(unsafe { *ebits.add(j as usize) } >= 0);
        j += 1;
    }

    // Save remaining bits for rebalancing in quant_all_bands()
    unsafe { *balance_out = balance };

    // Skipped bands use all their bits for fine energy
    while j < end {
        unsafe {
            *ebits.add(j as usize) = *bits.add(j as usize) >> stereo >> bitres;
            debug_assert!(c * *ebits.add(j as usize) << bitres == *bits.add(j as usize));
            *bits.add(j as usize) = 0;
            *fine_priority.add(j as usize) = (*ebits.add(j as usize) < 1) as i32;
        }
        j += 1;
    }

    coded_bands
}

/// Compute the pulse allocation for all bands.
///
/// This is the main bit allocation function called by both encoder and
/// decoder. It determines how many bits go to each band's PVQ coding
/// vs fine energy coding.
///
/// RFC 6716 Section 4.3.3.
///
/// # Safety
/// All pointer parameters must be valid. `m` must point to a valid CELTMode.
#[unsafe(no_mangle)]
#[allow(clippy::too_many_arguments)]
pub unsafe extern "C" fn compute_allocation(
    m: *const CELTMode,
    start: c_int,
    end: c_int,
    offsets: *const c_int,
    cap: *const c_int,
    alloc_trim: c_int,
    intensity: *mut c_int,
    dual_stereo: *mut c_int,
    total: i32,
    balance: *mut i32,
    pulses: *mut c_int,
    ebits: *mut c_int,
    fine_priority: *mut c_int,
    c: c_int,
    lm: c_int,
    ec: *mut ec_ctx,
    encode: c_int,
    prev: c_int,
) -> c_int {
    let m_ref = unsafe { &*m };
    let bitres = BITRES as i32;
    let mut total = imax(total, 0);
    let len = m_ref.nb_ebands;
    let mut skip_start = start;

    // Reserve a bit to signal the end of manually skipped bands
    let skip_rsv = if total >= 1 << bitres { 1 << bitres } else { 0 };
    total -= skip_rsv;

    // Reserve bits for the intensity and dual stereo parameters
    let mut intensity_rsv = 0i32;
    let mut dual_stereo_rsv = 0i32;
    if c == 2 {
        intensity_rsv = LOG2_FRAC_TABLE[(end - start) as usize] as i32;
        if intensity_rsv > total {
            intensity_rsv = 0;
        } else {
            total -= intensity_rsv;
            dual_stereo_rsv = if total >= 1 << bitres { 1 << bitres } else { 0 };
            total -= dual_stereo_rsv;
        }
    }

    let mut bits1_v = vec![0i32; len as usize];
    let mut bits2_v = vec![0i32; len as usize];
    let mut thresh_v = vec![0i32; len as usize];
    let mut trim_offset_v = vec![0i32; len as usize];

    for j in start..end {
        let ju = j as usize;
        let bw = unsafe { eb(m_ref.ebands, j + 1) } - unsafe { eb(m_ref.ebands, j) };
        // Below this threshold, we're sure not to allocate any PVQ bits
        thresh_v[ju] = imax(c << bitres, (3 * bw << lm << bitres) >> 4);
        // Tilt of the allocation curve
        trim_offset_v[ju] = c * bw * (alloc_trim - 5 - lm) * (end - j - 1) * (1 << (lm + bitres)) >> 6;
        // Giving less resolution to single-coefficient bands
        if bw << lm == 1 {
            trim_offset_v[ju] -= c << bitres;
        }
    }

    // Bisection to find the best allocation vector pair
    let mut lo = 1i32;
    let mut hi = m_ref.nb_alloc_vectors - 1;
    while lo <= hi {
        let mid = (lo + hi) >> 1;
        let mut done = false;
        let mut psum = 0i32;
        for j in (start..end).rev() {
            let ju = j as usize;
            let n = unsafe { eb(m_ref.ebands, j + 1) } - unsafe { eb(m_ref.ebands, j) };
            let mut bitsj = c * n * (unsafe { *m_ref.alloc_vectors.add((mid * len + j) as usize) } as i32) << lm >> 2;
            if bitsj > 0 {
                bitsj = imax(0, bitsj + trim_offset_v[ju]);
            }
            bitsj += unsafe { *offsets.add(ju) };
            if bitsj >= thresh_v[ju] || done {
                done = true;
                psum += imin(bitsj, unsafe { *cap.add(ju) });
            } else if bitsj >= c << bitres {
                psum += c << bitres;
            }
        }
        if psum > total {
            hi = mid - 1;
        } else {
            lo = mid + 1;
        }
    }
    hi = lo;
    lo -= 1;

    // Compute bits1 and bits2 for interpolation
    for j in start..end {
        let ju = j as usize;
        let n = unsafe { eb(m_ref.ebands, j + 1) } - unsafe { eb(m_ref.ebands, j) };
        let mut bits1j = c * n * (unsafe { *m_ref.alloc_vectors.add((lo * len + j) as usize) } as i32) << lm >> 2;
        let mut bits2j = if hi >= m_ref.nb_alloc_vectors {
            unsafe { *cap.add(ju) }
        } else {
            c * n * (unsafe { *m_ref.alloc_vectors.add((hi * len + j) as usize) } as i32) << lm >> 2
        };
        if bits1j > 0 {
            bits1j = imax(0, bits1j + trim_offset_v[ju]);
        }
        if bits2j > 0 {
            bits2j = imax(0, bits2j + trim_offset_v[ju]);
        }
        if lo > 0 {
            bits1j += unsafe { *offsets.add(ju) };
        }
        bits2j += unsafe { *offsets.add(ju) };
        if unsafe { *offsets.add(ju) } > 0 {
            skip_start = j;
        }
        bits2j = imax(0, bits2j - bits1j);
        bits1_v[ju] = bits1j;
        bits2_v[ju] = bits2j;
    }

    let coded_bands = unsafe {
        interp_bits2pulses(
            m,
            start,
            end,
            skip_start,
            bits1_v.as_ptr(),
            bits2_v.as_ptr(),
            thresh_v.as_ptr(),
            cap,
            total,
            balance,
            skip_rsv,
            intensity,
            intensity_rsv,
            dual_stereo,
            dual_stereo_rsv,
            pulses,
            ebits,
            fine_priority,
            c,
            lm,
            ec,
            encode,
            prev,
        )
    };
    coded_bands
}
