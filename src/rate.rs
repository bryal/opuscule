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

// Bit allocation is one cohesive algorithm built on per-band arrays all
// indexed by the same band number `j` (bits1/bits2/thresh/cap/bits/ebits/
// fine_priority/offsets), the mode tables (m.ebands/m.alloc_vectors/
// m.log_n/m.cache), and carried state (psum/balance/coded_bands) with
// interleaved range-decoder reads — often iterated backwards. Zipping the
// parallel arrays would be markedly less legible than indexed access and
// would obscure the decode order, so this stays indexed under a module-wide
// allow. Indices are band-bounded: j in [start, end) <= nb_ebands, the
// per-band arrays are nb_ebands long, and LOG2_FRAC_TABLE indices < 24.
#![allow(clippy::indexing_slicing)]

use std::os::raw::c_int;

use crate::entcode::{BITRES, EcCtx};
use crate::entdec::{ec_dec_bit_logp, ec_dec_uint};
use crate::modes::CELTMode;

// -- Constants from rate.h --

const MAX_FINE_BITS: i32 = 8;
const FINE_OFFSET: i32 = 21;
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
#[inline]
pub fn bits2pulses(m: &CELTMode, band: i32, lm: i32, bits: i32) -> i32 {
    let lm1 = lm + 1;
    let cache_idx = m.cache.index[(lm1 * m.nb_ebands + band) as usize];
    let cache = &m.cache.bits[cache_idx as usize..];
    let mut lo = 0i32;
    let mut hi = cache[0] as i32;
    let bits = bits - 1;
    for _ in 0..LOG_MAX_PSEUDO {
        let mid = (lo + hi + 1) >> 1;
        if cache[mid as usize] as i32 >= bits {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    let lo_val = if lo == 0 { -1 } else { cache[lo as usize] as i32 };
    if bits - lo_val <= cache[hi as usize] as i32 - bits { lo } else { hi }
}

/// Convert pseudo-pulse count to bits via the pulse cache.
#[inline]
pub fn pulses2bits(m: &CELTMode, band: i32, lm: i32, pulses: i32) -> i32 {
    let lm1 = lm + 1;
    let cache_idx = m.cache.index[(lm1 * m.nb_ebands + band) as usize];
    let cache = &m.cache.bits[cache_idx as usize..];
    if pulses == 0 { 0 } else { cache[pulses as usize] as i32 + 1 }
}

/// Read an i16 from a pointer at offset, as i32.
///
/// # Safety
#[inline]
fn eb(ebands: &[i16], i: i32) -> i32 {
    ebands[i as usize] as i32
}

/// Inner bit allocation: interpolate between two allocation vectors,
/// decide which bands to skip, and distribute bits to PVQ vs fine energy.
#[allow(clippy::too_many_arguments)]
fn interp_bits2pulses(
    m: &CELTMode,
    start: i32,
    end: i32,
    skip_start: i32,
    bits1: &[i32],
    bits2: &[i32],
    thresh: &[i32],
    cap: &[i32],
    mut total: i32,
    balance_out: &mut i32,
    skip_rsv: i32,
    intensity: &mut i32,
    mut intensity_rsv: i32,
    dual_stereo: &mut i32,
    dual_stereo_rsv: i32,
    bits: &mut [i32],
    ebits: &mut [i32],
    fine_priority: &mut [i32],
    c: i32,
    lm: i32,
    ec: &mut EcCtx,
    encode: i32,
    prev: i32,
) -> i32 {
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
            let tmp = bits1[ju] + ((mid as i64 * bits2[ju] as i64) >> ALLOC_STEPS) as i32;
            if tmp >= thresh[ju] || done {
                done = true;
                psum += tmp.min(cap[ju]);
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
        let mut tmp = bits1[ju] + (lo * bits2[ju] >> ALLOC_STEPS);
        if tmp < thresh[ju] && !done {
            tmp = if tmp >= alloc_floor { alloc_floor } else { 0 };
        } else {
            done = true;
        }
        tmp = tmp.min(cap[ju]);
        bits[ju] = tmp;
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
        let percoeff = left / (eb(m.ebands, coded_bands) - eb(m.ebands, start));
        let left_rem = left - (eb(m.ebands, coded_bands) - eb(m.ebands, start)) * percoeff;
        let rem = (left_rem - (eb(m.ebands, j) - eb(m.ebands, start))).max(0);
        let band_width = eb(m.ebands, coded_bands) - eb(m.ebands, j);
        let mut band_bits = bits[j as usize] + percoeff * band_width + rem;

        if band_bits >= thresh[j as usize].max(alloc_floor + (1 << bitres)) {
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
            } else if ec_dec_bit_logp(ec, 1) != 0 {
                break;
            }
            psum += 1 << bitres;
            band_bits -= 1 << bitres;
        }
        psum -= bits[j as usize] + intensity_rsv;
        if intensity_rsv > 0 {
            intensity_rsv = LOG2_FRAC_TABLE[(j - start) as usize] as i32;
        }
        psum += intensity_rsv;
        if band_bits >= alloc_floor {
            psum += alloc_floor;
            bits[j as usize] = alloc_floor;
        } else {
            bits[j as usize] = 0;
        }
        coded_bands -= 1;
    }

    debug_assert!(coded_bands > start);

    // Code the intensity and dual stereo parameters
    if intensity_rsv > 0 {
        if encode != 0 {
            *intensity = (*intensity).min(coded_bands);
            // ec_enc_uint(ec, *intensity - start, coded_bands + 1 - start)
        } else {
            *intensity = start + ec_dec_uint(ec, (coded_bands + 1 - start) as u32) as i32;
        }
    } else {
        *intensity = 0;
    }

    let mut dual_stereo_rsv = dual_stereo_rsv;
    if *intensity <= start {
        total += dual_stereo_rsv;
        dual_stereo_rsv = 0;
    }

    if dual_stereo_rsv > 0 {
        if encode != 0 {
            // ec_enc_bit_logp(ec, *dual_stereo, 1)
        } else {
            *dual_stereo = ec_dec_bit_logp(ec, 1);
        }
    } else {
        *dual_stereo = 0;
    }

    // Allocate the remaining bits
    let left = total - psum;
    let percoeff = left / (eb(m.ebands, coded_bands) - eb(m.ebands, start));
    let mut left = left - (eb(m.ebands, coded_bands) - eb(m.ebands, start)) * percoeff;

    for j in start..coded_bands {
        let bw = eb(m.ebands, j + 1) - eb(m.ebands, j);
        bits[j as usize] += percoeff * bw;
    }
    for j in start..coded_bands {
        let bw = eb(m.ebands, j + 1) - eb(m.ebands, j);
        let tmp = left.min(bw);
        bits[j as usize] += tmp;
        left -= tmp;
    }

    // Distribute bits between fine energy and PVQ for each band
    let mut balance = 0i32;
    let mut j = start;
    while j < coded_bands {
        let n0 = eb(m.ebands, j + 1) - eb(m.ebands, j);
        let n = n0 << lm;
        bits[j as usize] += balance;

        let excess;
        if n > 1 {
            excess = (bits[j as usize] - cap[j as usize]).max(0);
            bits[j as usize] -= excess;

            let den = c * n + if c == 2 && n > 2 && *dual_stereo == 0 && j < *intensity { 1 } else { 0 };

            let nc_log_n = den * (m.log_n[j as usize] as i32 + log_m);
            let mut offset = (nc_log_n >> 1) - den * FINE_OFFSET;
            if n == 2 {
                offset += den << bitres >> 2;
            }

            let bj = bits[j as usize];
            if bj + offset < den * 2 << bitres {
                offset += nc_log_n >> 2;
            } else if bj + offset < den * 3 << bitres {
                offset += nc_log_n >> 3;
            }

            let mut eb_j = ((bj + offset + (den << (bitres - 1))) / (den << bitres)).max(0);
            if c * eb_j > (bj >> bitres) {
                eb_j = bj >> stereo >> bitres;
            }
            eb_j = eb_j.min(MAX_FINE_BITS);

            fine_priority[j as usize] = (eb_j * (den << bitres) >= bj + offset) as i32;
            bits[j as usize] -= c * eb_j << bitres;
            ebits[j as usize] = eb_j;
        } else {
            // For N=1, all bits go to fine energy except for a single sign bit
            excess = (bits[j as usize] - (c << bitres)).max(0);
            bits[j as usize] -= excess;
            ebits[j as usize] = 0;
            fine_priority[j as usize] = 1;
        }

        // Fine energy can't take advantage of re-balancing in quant_all_bands().
        // Instead, do the re-balancing here.
        let mut excess = excess;
        if excess > 0 {
            let extra_fine = (excess >> (stereo + bitres)).min(MAX_FINE_BITS - ebits[j as usize]);
            ebits[j as usize] += extra_fine;
            let extra_bits = extra_fine * c << bitres;
            fine_priority[j as usize] = (extra_bits >= excess - balance) as i32;
            excess -= extra_bits;
        }
        balance = excess;

        debug_assert!(bits[j as usize] >= 0);
        debug_assert!(ebits[j as usize] >= 0);
        j += 1;
    }

    // Save remaining bits for rebalancing in quant_all_bands()
    *balance_out = balance;

    // Skipped bands use all their bits for fine energy
    while j < end {
        ebits[j as usize] = bits[j as usize] >> stereo >> bitres;
        debug_assert!(c * ebits[j as usize] << bitres == bits[j as usize]);
        bits[j as usize] = 0;
        fine_priority[j as usize] = (ebits[j as usize] < 1) as i32;
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
#[allow(clippy::too_many_arguments)]
pub fn compute_allocation(
    m: &CELTMode,
    start: c_int,
    end: c_int,
    offsets: &[c_int],
    cap: &[c_int],
    alloc_trim: c_int,
    intensity: &mut c_int,
    dual_stereo: &mut c_int,
    total: i32,
    balance: &mut i32,
    pulses: &mut [c_int],
    ebits: &mut [c_int],
    fine_priority: &mut [c_int],
    c: c_int,
    lm: c_int,
    ec: &mut EcCtx,
    encode: c_int,
    prev: c_int,
) -> c_int {
    let bitres = BITRES as i32;
    let mut total = total.max(0);
    let len = m.nb_ebands;
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
        let bw = eb(m.ebands, j + 1) - eb(m.ebands, j);
        // Below this threshold, we're sure not to allocate any PVQ bits
        thresh_v[ju] = (c << bitres).max((3 * bw << lm << bitres) >> 4);
        // Tilt of the allocation curve
        trim_offset_v[ju] = c * bw * (alloc_trim - 5 - lm) * (end - j - 1) * (1 << (lm + bitres)) >> 6;
        // Giving less resolution to single-coefficient bands
        if bw << lm == 1 {
            trim_offset_v[ju] -= c << bitres;
        }
    }

    // Bisection to find the best allocation vector pair
    let mut lo = 1i32;
    let mut hi = m.nb_alloc_vectors - 1;
    while lo <= hi {
        let mid = (lo + hi) >> 1;
        let mut done = false;
        let mut psum = 0i32;
        for j in (start..end).rev() {
            let ju = j as usize;
            let n = eb(m.ebands, j + 1) - eb(m.ebands, j);
            let mut bitsj = c * n * (m.alloc_vectors[(mid * len + j) as usize] as i32) << lm >> 2;
            if bitsj > 0 {
                bitsj = (bitsj + trim_offset_v[ju]).max(0);
            }
            bitsj += offsets[ju];
            if bitsj >= thresh_v[ju] || done {
                done = true;
                psum += bitsj.min(cap[ju]);
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
        let n = eb(m.ebands, j + 1) - eb(m.ebands, j);
        let mut bits1j = c * n * (m.alloc_vectors[(lo * len + j) as usize] as i32) << lm >> 2;
        let mut bits2j = if hi >= m.nb_alloc_vectors {
            cap[ju]
        } else {
            c * n * (m.alloc_vectors[(hi * len + j) as usize] as i32) << lm >> 2
        };
        if bits1j > 0 {
            bits1j = (bits1j + trim_offset_v[ju]).max(0);
        }
        if bits2j > 0 {
            bits2j = (bits2j + trim_offset_v[ju]).max(0);
        }
        if lo > 0 {
            bits1j += offsets[ju];
        }
        bits2j += offsets[ju];
        if offsets[ju] > 0 {
            skip_start = j;
        }
        bits2j = (bits2j - bits1j).max(0);
        bits1_v[ju] = bits1j;
        bits2_v[ju] = bits2j;
    }

    interp_bits2pulses(
        m,
        start,
        end,
        skip_start,
        &bits1_v,
        &bits2_v,
        &thresh_v,
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
}
