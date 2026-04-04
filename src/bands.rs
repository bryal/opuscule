// Translated from c/celt/bands.c (RFC 6716).
//
// CELT band processing functions: energy computation, normalisation,
// stereo handling, and spectral folding/unfolding.
//
// Functions are translated incrementally, innermost helpers first.

/// Linear congruential generator used for pseudo-random noise injection
/// in the CELT decoder (PLC comfort noise, anti-collapse, spectral folding).
/// Constants match Numerical Recipes / Knuth MMIX.
#[unsafe(no_mangle)]
pub extern "C" fn celt_lcg_rand(seed: u32) -> u32 {
    seed.wrapping_mul(1664525).wrapping_add(1013904223)
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
