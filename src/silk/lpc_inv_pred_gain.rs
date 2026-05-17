//! Translated from `c/silk/LPC_inv_pred_gain.c` (RFC 6716).
//!
//! Compute inverse of LPC prediction gain and test stability (all poles
//! within unit circle).

use super::macros::{silk_clz32, silk_inverse32_varq, silk_lshift, silk_rshift_round64, silk_smmul, silk_smull};

const SILK_MAX_ORDER_LPC: usize = 16;

const QA: i32 = 24;

/// `SILK_FIX_CONST(0.99975, 24)` → `(0.99975 * (1 << 24) + 0.5) as i32`
const A_LIMIT: i32 = ((0.99975f64 * (1i64 << QA) as f64) + 0.5) as i32;

/// `MUL32_FRAC_Q(a, b, Q)` — `(i32)(round(a * b >> Q))`.
#[inline]
fn mul32_frac_q(a32: i32, b32: i32, q: i32) -> i32 {
    silk_rshift_round64(silk_smull(a32, b32), q) as i32
}

/// Core inner routine — operates on coefficients already in QA domain.
fn lpc_inverse_pred_gain_qa(a_qa: &mut [[i32; SILK_MAX_ORDER_LPC]; 2], order: i32) -> i32 {
    let mut anew_qa = &mut a_qa[(order & 1) as usize] as *mut [i32; SILK_MAX_ORDER_LPC] as *mut i32;

    let mut inv_gain_q30: i32 = 1 << 30;
    let mut k = order - 1;
    while k > 0 {
        /* Check for stability */
        unsafe {
            if *anew_qa.offset(k as isize) > A_LIMIT || *anew_qa.offset(k as isize) < -A_LIMIT {
                return 0;
            }
        }

        /* Set RC equal to negated AR coef */
        let rc_q31 = unsafe { -silk_lshift(*anew_qa.offset(k as isize), 31 - QA) };

        /* rc_mult1_Q30 range: [ 1 : 2^30 ] */
        let rc_mult1_q30 = (1i32 << 30) - silk_smmul(rc_q31, rc_q31);

        /* rc_mult2 range: [ 2^30 : silk_int32_MAX ] */
        let mult2q = 32 - silk_clz32(rc_mult1_q30.wrapping_abs());
        let rc_mult2 = silk_inverse32_varq(rc_mult1_q30, mult2q + 30);

        /* Update inverse gain */
        inv_gain_q30 = silk_lshift(silk_smmul(inv_gain_q30, rc_mult1_q30), 2);

        /* Swap pointers */
        let aold_qa = anew_qa;
        anew_qa = &mut a_qa[(k & 1) as usize] as *mut [i32; SILK_MAX_ORDER_LPC] as *mut i32;

        /* Update AR coefficient */
        let mut n = 0;
        while n < k {
            unsafe {
                let tmp_qa = *aold_qa.offset(n as isize) - mul32_frac_q(*aold_qa.offset((k - n - 1) as isize), rc_q31, 31);
                *anew_qa.offset(n as isize) = mul32_frac_q(tmp_qa, rc_mult2, mult2q);
            }
            n += 1;
        }
        k -= 1;
    }

    /* Check for stability */
    unsafe {
        if *anew_qa.offset(0) > A_LIMIT || *anew_qa.offset(0) < -A_LIMIT {
            return 0;
        }
    }

    /* Set RC equal to negated AR coef */
    let rc_q31 = unsafe { -silk_lshift(*anew_qa.offset(0), 31 - QA) };

    /* Range: [ 1 : 2^30 ] */
    let rc_mult1_q30 = (1i32 << 30) - silk_smmul(rc_q31, rc_q31);

    /* Update inverse gain */
    inv_gain_q30 = silk_lshift(silk_smmul(inv_gain_q30, rc_mult1_q30), 2);

    inv_gain_q30
}

/// `silk_LPC_inverse_pred_gain` — for input coefficients in Q12 domain.
///
/// Returns inverse prediction gain in energy domain, Q30.
/// Returns 0 if the filter is unstable.
pub fn silk_lpc_inverse_pred_gain(a_q12: &[i16], order: i32) -> i32 {
    let mut atmp_qa = [[0i32; SILK_MAX_ORDER_LPC]; 2];
    let anew_qa = &mut atmp_qa[(order & 1) as usize];

    let mut dc_resp: i32 = 0;
    /* Increase Q domain of the AR coefficients */
    for k in 0..order as usize {
        let a = a_q12[k] as i32;
        dc_resp += a;
        anew_qa[k] = silk_lshift(a, QA - 12);
    }
    /* If the DC is unstable, we don't even need to do the full calculations */
    if dc_resp >= 4096 {
        return 0;
    }
    lpc_inverse_pred_gain_qa(&mut atmp_qa, order)
}

/// `silk_LPC_inverse_pred_gain_Q24` — for input coefficients in Q24 domain.
///
/// Only used by the fixed-point build.
pub fn silk_lpc_inverse_pred_gain_q24(a_q24: *const i32, order: i32) -> i32 {
    let mut atmp_qa = [[0i32; SILK_MAX_ORDER_LPC]; 2];
    let anew_qa = &mut atmp_qa[(order & 1) as usize];

    /* Increase Q domain of the AR coefficients */
    let mut k = 0;
    while k < order {
        anew_qa[k as usize] = unsafe { *a_q24.offset(k as isize) } >> (24 - QA);
        k += 1;
    }

    lpc_inverse_pred_gain_qa(&mut atmp_qa, order)
}
