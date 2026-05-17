//! Translated from `c/silk/NLSF_VQ_weights_laroia.c` (RFC 6716).
//!
//! Low-complexity NLSF VQ weights after Laroia, Phamdo & Farvardin (1991).

const NLSF_W_Q: i32 = 2;

/// `silk_NLSF_VQ_weights_laroia` — compute per-coefficient VQ weights for
/// an NLSF vector. The weights are inversely proportional to the inter-
/// coefficient spacing, so narrow gaps get higher weight.
pub fn silk_nlsf_vq_weights_laroia(p_nlsfw_q_out: &mut [i16], p_nlsf_q15: &[i16]) {
    let d = p_nlsf_q15.len();

    /* First value */
    let mut tmp1_int = (p_nlsf_q15[0] as i32).max(1);
    tmp1_int = (1 << (15 + NLSF_W_Q)) / tmp1_int;
    let mut tmp2_int = (p_nlsf_q15[1] as i32 - p_nlsf_q15[0] as i32).max(1);
    tmp2_int = (1 << (15 + NLSF_W_Q)) / tmp2_int;
    p_nlsfw_q_out[0] = (tmp1_int + tmp2_int).min(i16::MAX as i32) as i16;

    /* Main loop */
    let mut k = 1;
    while k < d - 1 {
        tmp1_int = (p_nlsf_q15[k + 1] as i32 - p_nlsf_q15[k] as i32).max(1);
        tmp1_int = (1 << (15 + NLSF_W_Q)) / tmp1_int;
        p_nlsfw_q_out[k] = (tmp1_int + tmp2_int).min(i16::MAX as i32) as i16;

        tmp2_int = (p_nlsf_q15[k + 2] as i32 - p_nlsf_q15[k + 1] as i32).max(1);
        tmp2_int = (1 << (15 + NLSF_W_Q)) / tmp2_int;
        p_nlsfw_q_out[k + 1] = (tmp1_int + tmp2_int).min(i16::MAX as i32) as i16;
        k += 2;
    }

    /* Last value */
    tmp1_int = ((1 << 15) - p_nlsf_q15[d - 1] as i32).max(1);
    tmp1_int = (1 << (15 + NLSF_W_Q)) / tmp1_int;
    p_nlsfw_q_out[d - 1] = (tmp1_int + tmp2_int).min(i16::MAX as i32) as i16;
}
