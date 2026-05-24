// Translated from c/celt/laplace.c (RFC 6716).
//
// Laplace-distributed symbol coding for coarse energy quantization.
// The decoder uses a two-sided geometric (Laplace) distribution to
// decode energy deltas, with an exponentially decaying tail that
// bottoms out at a minimum probability floor.
//
// Only the decode function is translated; encoding is not needed for
// a decode-only implementation.

use std::os::raw::c_int;

use crate::entcode::ec_dec;
use crate::entdec::{ec_dec_update, ec_decode_bin};

/// Minimum probability of an energy delta (out of 32768).
const LAPLACE_LOG_MINP: u32 = 0;
const LAPLACE_MINP: u32 = 1 << LAPLACE_LOG_MINP;
/// Minimum number of guaranteed representable energy deltas (one direction).
const LAPLACE_NMIN: u32 = 16;

/// Compute the probability of the value +/- 1 from the base probability
/// and decay factor.
#[inline]
fn ec_laplace_get_freq1(fs0: u32, decay: c_int) -> u32 {
    let ft = 32768 - LAPLACE_MINP * (2 * LAPLACE_NMIN) - fs0;
    (ft * (16384 - decay as u32)) >> 15
}

/// Decode a value from a Laplace distribution.
///
/// Used by unquant_coarse_energy() in quant_bands.c to decode the coarse
/// energy values for each band (RFC 6716 Section 4.3.1).
///
/// `fs` is the probability of zero (times 32768).
/// `decay` controls how fast the tail probability decays.
pub fn ec_laplace_decode(dec: &mut ec_dec, fs: u32, decay: c_int) -> c_int {
    let mut val: c_int = 0;
    let mut fl: u32;
    let mut fs = fs;

    let fm = ec_decode_bin(dec, 15);
    fl = 0;

    if fm >= fs {
        val += 1;
        fl = fs;
        fs = ec_laplace_get_freq1(fs, decay) + LAPLACE_MINP;

        // Search the decaying part of the PDF.
        while fs > LAPLACE_MINP && fm >= fl + 2 * fs {
            fs *= 2;
            fl += fs;
            fs = ((fs - 2 * LAPLACE_MINP) as i64 * decay as i64 >> 15) as u32;
            fs += LAPLACE_MINP;
            val += 1;
        }

        // Everything beyond that has probability LAPLACE_MINP.
        if fs <= LAPLACE_MINP {
            let di = (fm - fl) >> (LAPLACE_LOG_MINP + 1);
            val += di as c_int;
            fl += 2 * di * LAPLACE_MINP;
        }

        if fm < fl + fs {
            val = -val;
        } else {
            fl += fs;
        }
    }

    debug_assert!(fl < 32768);
    debug_assert!(fs > 0);
    debug_assert!(fl <= fm);
    debug_assert!(fm < (fl + fs).min(32768));

    ec_dec_update(dec, fl, (fl + fs).min(32768), 32768);
    val
}
