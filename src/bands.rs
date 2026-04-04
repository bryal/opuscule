// Translated from c/celt/bands.c (RFC 6716).
//
// CELT band processing functions: energy computation, normalisation,
// stereo handling, and spectral folding/unfolding.
//
// Functions are translated incrementally, innermost helpers first.

use std::os::raw::c_int;

use crate::arch::{CeltEner, CeltNorm, CeltSig, mult16_32_q15, shl32, shr32};
use crate::modes::CELTMode;

/// Linear congruential generator used for pseudo-random noise injection
/// in the CELT decoder (PLC comfort noise, anti-collapse, spectral folding).
/// Constants match Numerical Recipes / Knuth MMIX.
#[unsafe(no_mangle)]
pub extern "C" fn celt_lcg_rand(seed: u32) -> u32 {
    seed.wrapping_mul(1664525).wrapping_add(1013904223)
}

/// De-normalise band energies to produce synthesis output.
///
/// Scales each unit-energy band by its decoded energy envelope, producing
/// the frequency-domain signal that feeds into the inverse MDCT.
/// Bands beyond `end` are zeroed (above the Nyquist for the coded bandwidth).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn denormalise_bands(
    m: *const CELTMode,
    x: *const CeltNorm,
    freq: *mut CeltSig,
    band_e: *const CeltEner,
    end: c_int,
    c_channels: c_int,
    m_factor: c_int,
) {
    unsafe {
        let mode = &*m;
        let ebands = mode.ebands;
        let n = m_factor * mode.short_mdct_size;

        let mut c = 0;
        loop {
            let f_base = freq.add((c * n) as usize);
            let x_base = x.add((c * n) as usize);
            for i in 0..end as usize {
                let g = shr32(*band_e.add(i + (c as usize) * mode.nb_ebands as usize), 1);
                let j_start = (m_factor * (*ebands.add(i) as c_int)) as usize;
                let band_end = (m_factor * (*ebands.add(i + 1) as c_int)) as usize;
                for j in j_start..band_end {
                    *f_base.add(j) = shl32(mult16_32_q15(*x_base.add(j), g), 2);
                }
            }

            // Zero above the coded bandwidth
            let zero_start = (m_factor * (*ebands.add(end as usize) as c_int)) as usize;
            for i in zero_start..n as usize {
                *f_base.add(i) = 0 as CeltSig;
            }

            c += 1;
            if c >= c_channels {
                break;
            }
        }
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
