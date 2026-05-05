//! Translated from `c/silk/NLSF2A.c` (RFC 6716).
//!
//! Conversion from normalized line spectral frequencies (NLSFs, Q15) to
//! Q12 monic LPC whitening-filter coefficients. The algorithm builds
//! two polynomials (P = prod(1 - 2*cos(LSF_even)*z^-1 + z^-2),
//! Q = prod(1 + 2*cos(LSF_odd)*z^-1 + z^-2)) via recursive convolution
//! and then clamps/bandwidth-expands the result until the corresponding
//! prediction filter is stable (≤ MAX_PREDICTION_POWER_GAIN).
//!
//! 2*cos(NLSF) is evaluated by linear interpolation of the
//! `silk_LSFCosTab_FIX_Q12` table, using a manually-chosen ordering
//! that was found to maximize numerical accuracy.

use super::bwexpander_32::silk_bwexpander_32;
use super::lpc_inv_pred_gain::silk_lpc_inverse_pred_gain;
use super::macros::{silk_lshift, silk_rshift_round, silk_rshift_round64, silk_sat16, silk_smull};
use super::table_lsf_cos::SILK_LSF_COS_TAB_FIX_Q12;

const QA: i32 = 16;

const SILK_MAX_ORDER_LPC: usize = 16;
const LSF_COS_TAB_SZ_FIX: i32 = 128;
const MAX_LPC_STABILIZE_ITERATIONS: i32 = 16;

/// `1.0 / MAX_PREDICTION_POWER_GAIN` = `1e-4`, constant-evaluated in Q30
/// as in the C `SILK_FIX_CONST( 1.0 / 1e4, 30 )`.
const ONE_OVER_MAX_PREDICTION_POWER_GAIN_Q30: i32 = (1.0 / 1e4 * (1u32 << 30) as f64 + 0.5) as i32;
/// `SILK_FIX_CONST( 0.999, 16 )`.
const SC_BASE_Q16: i32 = (0.999 * (1u32 << 16) as f64 + 0.5) as i32;

/// `silk_NLSF2A_find_poly` — build a polynomial by recursive convolution.
#[inline]
unsafe fn silk_nlsf2a_find_poly(out: *mut i32, c_lsf: *const i32, dd: i32) {
    unsafe {
        *out.offset(0) = silk_lshift(1, QA);
        *out.offset(1) = -*c_lsf.offset(0);
        let mut k = 1i32;
        while k < dd {
            let ftmp = *c_lsf.offset((2 * k) as isize); /* QA */
            *out.offset((k + 1) as isize) = silk_lshift(*out.offset((k - 1) as isize), 1)
                - silk_rshift_round64(silk_smull(ftmp, *out.offset(k as isize)), QA) as i32;
            let mut n = k;
            while n > 1 {
                *out.offset(n as isize) += *out.offset((n - 2) as isize)
                    - silk_rshift_round64(silk_smull(ftmp, *out.offset((n - 1) as isize)), QA) as i32;
                n -= 1;
            }
            *out.offset(1) -= ftmp;
            k += 1;
        }
    }
}

/// `silk_NLSF2A` — compute LPC whitening filter coefficients from NLSFs.
pub unsafe fn silk_nlsf2a(a_q12: *mut i16, nlsf: *const i16, d: i32) {
    unsafe {
        /* This ordering was found to maximize quality. It improves numerical
         * accuracy of silk_NLSF2A_find_poly() compared to "standard" ordering. */
        static ORDERING16: [u8; 16] = [0, 15, 8, 7, 4, 11, 12, 3, 2, 13, 10, 5, 6, 9, 14, 1];
        static ORDERING10: [u8; 10] = [0, 9, 6, 3, 4, 5, 8, 1, 2, 7];

        let mut cos_lsf_qa = [0i32; SILK_MAX_ORDER_LPC];
        let mut p_poly = [0i32; SILK_MAX_ORDER_LPC / 2 + 1];
        let mut q_poly = [0i32; SILK_MAX_ORDER_LPC / 2 + 1];
        let mut a32_qa1 = [0i32; SILK_MAX_ORDER_LPC];

        /* silk_assert(LSF_COS_TAB_SZ_FIX == 128); */
        /* silk_assert(d == 10 || d == 16); */

        /* convert LSFs to 2*cos(LSF), using piecewise linear curve from table */
        let ordering: &[u8] = if d == 16 { &ORDERING16 } else { &ORDERING10 };
        let mut k = 0i32;
        while k < d {
            /* silk_assert(nlsf[k] >= 0); */

            /* f_int on a scale 0-127 (rounded down) */
            let f_int = *nlsf.offset(k as isize) as i32 >> (15 - 7);

            /* f_frac, range: 0..255 */
            let f_frac = *nlsf.offset(k as isize) as i32 - silk_lshift(f_int, 15 - 7);

            /* silk_assert(f_int >= 0); */
            /* silk_assert(f_int < LSF_COS_TAB_SZ_FIX); */
            let _ = LSF_COS_TAB_SZ_FIX;

            /* Read start and end value from table */
            let cos_val = SILK_LSF_COS_TAB_FIX_Q12[f_int as usize] as i32; /* Q12 */
            let delta = SILK_LSF_COS_TAB_FIX_Q12[(f_int + 1) as usize] as i32 - cos_val; /* Q12, range 0..200 */

            /* Linear interpolation */
            cos_lsf_qa[ordering[k as usize] as usize] = silk_rshift_round(silk_lshift(cos_val, 8) + delta * f_frac, 20 - QA); /* QA */
            k += 1;
        }

        let dd = d >> 1;

        /* generate even and odd polynomials using convolution */
        silk_nlsf2a_find_poly(p_poly.as_mut_ptr(), cos_lsf_qa.as_ptr().offset(0), dd);
        silk_nlsf2a_find_poly(q_poly.as_mut_ptr(), cos_lsf_qa.as_ptr().offset(1), dd);

        /* convert even and odd polynomials to opus_int32 Q12 filter coefs */
        let mut k = 0i32;
        while k < dd {
            let ptmp = p_poly[(k + 1) as usize] + p_poly[k as usize];
            let qtmp = q_poly[(k + 1) as usize] - q_poly[k as usize];

            /* the Ptmp and Qtmp values at this stage need to fit in int32 */
            a32_qa1[k as usize] = -qtmp - ptmp; /* QA+1 */
            a32_qa1[(d - k - 1) as usize] = qtmp - ptmp; /* QA+1 */
            k += 1;
        }

        /* Limit the maximum absolute value of the prediction coefficients, so they'll fit in int16 */
        let mut i = 0i32;
        let mut idx: i32 = 0;
        while i < 10 {
            /* Find maximum absolute value and its index */
            let mut maxabs: i32 = 0;
            let mut k = 0i32;
            while k < d {
                let absval = a32_qa1[k as usize].wrapping_abs();
                if absval > maxabs {
                    maxabs = absval;
                    idx = k;
                }
                k += 1;
            }
            let maxabs = silk_rshift_round(maxabs, QA + 1 - 12); /* QA+1 -> Q12 */

            if maxabs > i16::MAX as i32 {
                /* Reduce magnitude of prediction coefficients */
                let maxabs = maxabs.min(163838); /* (silk_int32_MAX >> 14) + silk_int16_MAX = 163838 */
                let sc_q16 = SC_BASE_Q16 - silk_lshift(maxabs - i16::MAX as i32, 14) / (maxabs * (idx + 1) >> 2);
                silk_bwexpander_32(a32_qa1.as_mut_ptr(), d, sc_q16);
            } else {
                break;
            }
            i += 1;
        }

        if i == 10 {
            /* Reached the last iteration, clip the coefficients */
            let mut k = 0i32;
            while k < d {
                *a_q12.offset(k as isize) = silk_sat16(silk_rshift_round(a32_qa1[k as usize], QA + 1 - 12)) as i16; /* QA+1 -> Q12 */
                a32_qa1[k as usize] = silk_lshift(*a_q12.offset(k as isize) as i32, QA + 1 - 12);
                k += 1;
            }
        } else {
            let mut k = 0i32;
            while k < d {
                *a_q12.offset(k as isize) = silk_rshift_round(a32_qa1[k as usize], QA + 1 - 12) as i16; /* QA+1 -> Q12 */
                k += 1;
            }
        }

        let mut i = 0i32;
        while i < MAX_LPC_STABILIZE_ITERATIONS {
            if silk_lpc_inverse_pred_gain(a_q12, d) < ONE_OVER_MAX_PREDICTION_POWER_GAIN_Q30 {
                /* Prediction coefficients are (too close to) unstable; apply bandwidth expansion on
                 * the unscaled coefficients, convert to Q12 and measure again */
                silk_bwexpander_32(a32_qa1.as_mut_ptr(), d, 65536 - silk_lshift(2, i));
                let mut k = 0i32;
                while k < d {
                    *a_q12.offset(k as isize) = silk_rshift_round(a32_qa1[k as usize], QA + 1 - 12) as i16; /* QA+1 -> Q12 */
                    k += 1;
                }
            } else {
                break;
            }
            i += 1;
        }
    }
}
