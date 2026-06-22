//! Translated from `c/silk/LPC_inv_pred_gain.c` (RFC 6716).
//!
//! Compute inverse of LPC prediction gain and test stability (all poles
//! within unit circle).

#![allow(clippy::indexing_slicing)] // dense SILK kernels; voice path is deprioritized vs CELT

use super::macros::{
    silk_clz32, silk_inverse32_varq, silk_lshift, silk_rshift_round64, silk_smmul, silk_smull, silk_sub_sat32,
};

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
///
/// The C swapped two raw pointers between the rows of `a_qa`; here we track
/// the "current" row as an index instead. The two rows always have opposite
/// parity (`new` becomes `k & 1`, `old` the previous), so reads of `old` and
/// writes to `new` never alias.
fn lpc_inverse_pred_gain_qa(a_qa: &mut [[i32; SILK_MAX_ORDER_LPC]; 2], order: i32) -> i32 {
    let mut new_idx = (order & 1) as usize;

    let mut inv_gain_q30: i32 = 1 << 30;
    let mut k = order - 1;
    while k > 0 {
        let kk = k as usize;

        /* Check for stability */
        if a_qa[new_idx][kk] > A_LIMIT || a_qa[new_idx][kk] < -A_LIMIT {
            return 0;
        }

        /* Set RC equal to negated AR coef */
        let rc_q31 = -silk_lshift(a_qa[new_idx][kk], 31 - QA);

        /* rc_mult1_Q30 range: [ 1 : 2^30 ] */
        let rc_mult1_q30 = (1i32 << 30) - silk_smmul(rc_q31, rc_q31);

        /* rc_mult2 range: [ 2^30 : silk_int32_MAX ] */
        let mult2q = 32 - silk_clz32(rc_mult1_q30.wrapping_abs());
        let rc_mult2 = silk_inverse32_varq(rc_mult1_q30, mult2q + 30);

        /* Update inverse gain */
        inv_gain_q30 = silk_lshift(silk_smmul(inv_gain_q30, rc_mult1_q30), 2);

        /* Swap rows */
        let old_idx = new_idx;
        new_idx = (k & 1) as usize;

        /* Update AR coefficient (reads old row, writes new row) */
        for n in 0..kk {
            // RFC 8251 section 6: a saturating subtract, and a 64-bit range
            // check on the rescaled coefficient - fuzzed bitstreams can drive
            // this past 32 bits (undefined in C); treat such filters as
            // unstable (return 0) rather than wrapping.
            let tmp_qa = silk_sub_sat32(a_qa[old_idx][n], mul32_frac_q(a_qa[old_idx][kk - n - 1], rc_q31, 31));
            let tmp64 = silk_rshift_round64(silk_smull(tmp_qa, rc_mult2), mult2q);
            if tmp64 > i32::MAX as i64 || tmp64 < i32::MIN as i64 {
                return 0;
            }
            a_qa[new_idx][n] = tmp64 as i32;
        }
        k -= 1;
    }

    /* Check for stability */
    if a_qa[new_idx][0] > A_LIMIT || a_qa[new_idx][0] < -A_LIMIT {
        return 0;
    }

    /* Set RC equal to negated AR coef */
    let rc_q31 = -silk_lshift(a_qa[new_idx][0], 31 - QA);

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
