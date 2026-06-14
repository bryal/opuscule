// Translated from c/celt/quant_bands.c (RFC 6716).
//
// Energy quantization/dequantization for CELT bands. Only the decoder
// (unquant) functions are translated — encoder functions were stripped
// in the decoder-only build.
//
// Five functions:
// - unquant_coarse_energy: decode coarse energy using Laplace-like model
// - unquant_fine_energy: decode fine energy refinement bits
// - unquant_energy_finalise: spend remaining bits on energy refinement
// - log2Amp: convert log-domain energies to linear amplitudes
// - amp2Log2: convert linear amplitudes to log-domain energies

use std::os::raw::c_int;

use crate::arch::*;
use crate::entcode::EcCtx;
use crate::entdec::ec_tell;
use crate::modes::CELTMode;

// -- Constants --

const DB_SHIFT: i32 = 10;
const MAX_FINE_BITS: i32 = 8;

/// Mean energy per band (Q6, converted to float in float mode).
#[cfg(feature = "fixed-point")]
static E_MEANS: [i8; 25] =
    [103, 100, 92, 85, 81, 77, 72, 70, 78, 75, 73, 71, 78, 74, 69, 72, 70, 74, 76, 71, 60, 60, 60, 60, 60];

#[cfg(not(feature = "fixed-point"))]
static E_MEANS: [f32; 25] = [
    6.437500, 6.250000, 5.750000, 5.312500, 5.062500, 4.812500, 4.500000, 4.375000, 4.875000, 4.687500, 4.562500, 4.437500,
    4.875000, 4.625000, 4.312500, 4.500000, 4.375000, 4.625000, 4.750000, 4.437500, 3.750000, 3.750000, 3.750000, 3.750000,
    3.750000,
];

/// Prediction coefficients: 0.9, 0.8, 0.65, 0.5
#[cfg(feature = "fixed-point")]
static PRED_COEF: [OpusVal16; 4] = [29440, 26112, 21248, 16384];
#[cfg(not(feature = "fixed-point"))]
static PRED_COEF: [OpusVal16; 4] = [29440.0 / 32768.0, 26112.0 / 32768.0, 21248.0 / 32768.0, 16384.0 / 32768.0];

#[cfg(feature = "fixed-point")]
static BETA_COEF: [OpusVal16; 4] = [30147, 22282, 12124, 6554];
#[cfg(not(feature = "fixed-point"))]
static BETA_COEF: [OpusVal16; 4] = [30147.0 / 32768.0, 22282.0 / 32768.0, 12124.0 / 32768.0, 6554.0 / 32768.0];

#[cfg(feature = "fixed-point")]
const BETA_INTRA: OpusVal16 = 4915;
#[cfg(not(feature = "fixed-point"))]
const BETA_INTRA: OpusVal16 = 4915.0 / 32768.0;

/// Parameters of the Laplace-like probability models used for coarse energy.
/// [frame_size][inter/intra][42 params]
static E_PROB_MODEL: [[[u8; 42]; 2]; 4] = [
    // 120 sample frames
    [
        // Inter
        [
            72, 127, 65, 129, 66, 128, 65, 128, 64, 128, 62, 128, 64, 128, 64, 128, 92, 78, 92, 79, 92, 78, 90, 79, 116, 41,
            115, 40, 114, 40, 132, 26, 132, 26, 145, 17, 161, 12, 176, 10, 177, 11,
        ],
        // Intra
        [
            24, 179, 48, 138, 54, 135, 54, 132, 53, 134, 56, 133, 55, 132, 55, 132, 61, 114, 70, 96, 74, 88, 75, 88, 87, 74,
            89, 66, 91, 67, 100, 59, 108, 50, 120, 40, 122, 37, 97, 43, 78, 50,
        ],
    ],
    // 240 sample frames
    [
        [
            83, 78, 84, 81, 88, 75, 86, 74, 87, 71, 90, 73, 93, 74, 93, 74, 109, 40, 114, 36, 117, 34, 117, 34, 143, 17, 145,
            18, 146, 19, 162, 12, 165, 10, 178, 7, 189, 6, 190, 8, 177, 9,
        ],
        [
            23, 178, 54, 115, 63, 102, 66, 98, 69, 99, 74, 89, 71, 91, 73, 91, 78, 89, 86, 80, 92, 66, 93, 64, 102, 59, 103,
            60, 104, 60, 117, 52, 123, 44, 138, 35, 133, 31, 97, 38, 77, 45,
        ],
    ],
    // 480 sample frames
    [
        [
            61, 90, 93, 60, 105, 42, 107, 41, 110, 45, 116, 38, 113, 38, 112, 38, 124, 26, 132, 27, 136, 19, 140, 20, 155, 14,
            159, 16, 158, 18, 170, 13, 177, 10, 187, 8, 192, 6, 175, 9, 159, 10,
        ],
        [
            21, 178, 59, 110, 71, 86, 75, 85, 84, 83, 91, 66, 88, 73, 87, 72, 92, 75, 98, 72, 105, 58, 107, 54, 115, 52, 114,
            55, 112, 56, 129, 51, 132, 40, 150, 33, 140, 29, 98, 35, 77, 42,
        ],
    ],
    // 960 sample frames
    [
        [
            42, 121, 96, 66, 108, 43, 111, 40, 117, 44, 123, 32, 120, 36, 119, 33, 127, 33, 134, 34, 139, 21, 147, 23, 152, 20,
            158, 25, 154, 26, 166, 21, 173, 16, 184, 13, 184, 10, 150, 13, 139, 15,
        ],
        [
            22, 178, 63, 114, 74, 82, 84, 83, 92, 82, 103, 62, 96, 72, 96, 67, 101, 73, 107, 72, 113, 55, 118, 52, 125, 52,
            118, 52, 117, 55, 135, 49, 137, 39, 157, 32, 145, 29, 97, 33, 77, 40,
        ],
    ],
];

static SMALL_ENERGY_ICDF: [u8; 3] = [2, 1, 0];

use crate::entdec::{ec_dec_bit_logp, ec_dec_bits, ec_dec_icdf};
use crate::laplace::ec_laplace_decode;

// -- Decoder functions --

/// Decode coarse energy from the bitstream using a Laplace-like model.
/// RFC 6716 Section 4.3.1.
///
/// Uses inter-frame prediction (unless `intra`), applying the probability
/// model selected by frame size (`LM`) and prediction mode.
///
/// Interleaves sequential range-decoder reads (budget-dependent Laplace /
/// ICDF / bit) with band-model-table lookups (`prob_model[2*min(i,20)]`)
/// and a per-channel `old_ebands[i + c*nb_ebands]` read-modify-write plus
/// `prev[c]` state. Indices are bounded (i < end <= nb_ebands, c < 2,
/// 2*min(i,20)+1 < 42), so it stays as explicit indexed decode rather
/// than obscuring the read order with strided iterators.
#[allow(clippy::indexing_slicing)]
pub fn unquant_coarse_energy(
    m: &CELTMode,
    start: c_int,
    end: c_int,
    old_ebands: &mut [OpusVal16],
    intra: c_int,
    dec: &mut EcCtx,
    c_channels: c_int,
    lm: c_int,
) {
    let prob_model = &E_PROB_MODEL[lm as usize][intra as usize];
    let nb_ebands = m.nb_ebands as usize;
    let mut prev: [OpusVal32; 2] = [0 as OpusVal32; 2];

    let coef: OpusVal16;
    let beta: OpusVal16;
    if intra != 0 {
        coef = 0 as OpusVal16;
        beta = BETA_INTRA;
    } else {
        beta = BETA_COEF[lm as usize];
        coef = PRED_COEF[lm as usize];
    }

    let budget = dec.storage as i32 * 8;

    for i in (start as usize)..(end as usize) {
        let mut c: usize = 0;
        loop {
            debug_assert!(c < 2);
            let tell = ec_tell(dec);
            let qi: c_int;
            if budget - tell >= 15 {
                let pi = 2 * (i as i32).min(20) as usize;
                qi = ec_laplace_decode(dec, (prob_model[pi] as u32) << 7, ((prob_model[pi + 1] as u32) << 6) as c_int);
            } else if budget - tell >= 2 {
                let raw = ec_dec_icdf(dec, &SMALL_ENERGY_ICDF, 2);
                qi = (raw >> 1) ^ -(raw & 1);
            } else if budget - tell >= 1 {
                qi = -ec_dec_bit_logp(dec, 1);
            } else {
                qi = -1;
            }
            let q: OpusVal32 = shl32(extend32(qi as OpusVal16), DB_SHIFT);

            let idx = i + c * nb_ebands;
            old_ebands[idx] = max16(-qconst16(9.0, DB_SHIFT), old_ebands[idx]);
            #[allow(unused_mut)]
            let mut tmp: OpusVal32 = pshr32(mult16_16(coef, old_ebands[idx]), 8) + prev[c] + shl32(q, 7);
            #[cfg(feature = "fixed-point")]
            let tmp = max32(-qconst32(28.0, DB_SHIFT + 7), tmp);
            old_ebands[idx] = pshr32(tmp, 7) as OpusVal16;
            prev[c] = prev[c] + shl32(q, 7) - mult16_16(beta, pshr32(q, 8) as OpusVal16);

            c += 1;
            if c as c_int >= c_channels {
                break;
            }
        }
    }
}

/// Decode fine energy refinement bits.
/// RFC 6716 Section 4.3.1.
///
/// Reads `fine_quant[i]` bits per band to refine the coarse energy estimate.
///
/// Sequential per-band/per-channel range-decoder reads driving a
/// `old_ebands[i + c*nb_ebands]` update; bounds obvious (i < end <=
/// nb_ebands, c < c_channels <= 2). Kept as explicit indexed decode.
#[allow(clippy::indexing_slicing)]
pub fn unquant_fine_energy(
    m: &CELTMode,
    start: c_int,
    end: c_int,
    old_ebands: &mut [OpusVal16],
    fine_quant: &[c_int],
    dec: &mut EcCtx,
    c_channels: c_int,
) {
    let nb_ebands = m.nb_ebands as usize;

    for i in (start as usize)..(end as usize) {
        if fine_quant[i] <= 0 {
            continue;
        }
        let mut c: usize = 0;
        loop {
            let q2 = ec_dec_bits(dec, fine_quant[i] as u32) as i32;
            let offset: OpusVal16;
            #[cfg(feature = "fixed-point")]
            {
                offset = sub16(
                    shr32(shl32(extend32(q2 as i16), DB_SHIFT) + qconst16(0.5, DB_SHIFT) as i32, fine_quant[i]) as i16,
                    qconst16(0.5, DB_SHIFT),
                );
            }
            #[cfg(not(feature = "fixed-point"))]
            {
                offset = (q2 as f32 + 0.5) * ((1 << (14 - fine_quant[i])) as f32) * (1.0 / 16384.0) - 0.5;
            }
            old_ebands[i + c * nb_ebands] += offset;

            c += 1;
            if c as c_int >= c_channels {
                break;
            }
        }
    }
}

/// Spend remaining bits on final energy refinement.
/// RFC 6716 Section 4.3.1.
///
/// Iterates by priority (0 then 1), reading 1 bit per band to adjust
/// the energy estimate by half a fine-quant step.
///
/// Sequential range-decoder reads gated by `bits_left` and per-band
/// `fine_quant[i]`/`fine_priority[i]`, updating `old_ebands[i +
/// c*nb_ebands]`; bounds obvious (i < end <= nb_ebands, c < c_channels
/// <= 2). Kept as explicit indexed decode.
#[allow(clippy::indexing_slicing)]
pub fn unquant_energy_finalise(
    m: &CELTMode,
    start: c_int,
    end: c_int,
    old_ebands: &mut [OpusVal16],
    fine_quant: &[c_int],
    fine_priority: &[c_int],
    mut bits_left: c_int,
    dec: &mut EcCtx,
    c_channels: c_int,
) {
    let nb_ebands = m.nb_ebands as usize;

    for prio in 0..2 {
        let mut i = start as usize;
        while i < end as usize && bits_left >= c_channels {
            if fine_quant[i] >= MAX_FINE_BITS as c_int || fine_priority[i] != prio {
                i += 1;
                continue;
            }
            let mut c: usize = 0;
            loop {
                let q2 = ec_dec_bits(dec, 1) as i32;
                let offset: OpusVal16;
                #[cfg(feature = "fixed-point")]
                {
                    offset = shr16(shl16(q2 as i16, DB_SHIFT) - qconst16(0.5, DB_SHIFT), fine_quant[i] + 1);
                }
                #[cfg(not(feature = "fixed-point"))]
                {
                    offset = (q2 as f32 - 0.5) * ((1 << (14 - fine_quant[i] - 1)) as f32) * (1.0 / 16384.0);
                }
                old_ebands[i + c * nb_ebands] += offset;
                bits_left -= 1;

                c += 1;
                if c as c_int >= c_channels {
                    break;
                }
            }
            i += 1;
        }
    }
}

/// Convert log-domain band energies to linear amplitudes.
///
/// Computes eBands[i] = 2^(oldEBands[i] + eMeans[i]) / 16 for active bands,
/// zeroing bands outside [start, end).
pub fn log2amp(m: &CELTMode, start: c_int, end: c_int, e_bands: &mut [OpusVal32], old_ebands: &[OpusVal16], c_channels: c_int) {
    let nb_ebands = m.nb_ebands as usize;
    let band_range = start as usize..end as usize;
    for (e_ch, old_ch) in e_bands.chunks_mut(nb_ebands).zip(old_ebands.chunks(nb_ebands)).take(c_channels as usize) {
        for (i, ((eb, &oe), &em)) in e_ch.iter_mut().zip(old_ch).zip(E_MEANS.iter()).enumerate() {
            *eb = if band_range.contains(&i) {
                pshr32(celt_exp2(add16(oe, shl16(em as OpusVal16, 6))), 4)
            } else {
                0 as OpusVal32
            };
        }
    }
}

/// Convert linear band amplitudes to log-domain energies.
///
/// Computes bandLogE[i] = log2(bandE[i] * 4) - eMeans[i] for active bands,
/// setting inactive bands to -14.0 (Q10).
pub fn amp2log2(
    m: &CELTMode,
    eff_end: c_int,
    end: c_int,
    band_e: &[OpusVal32],
    band_log_e: &mut [OpusVal16],
    c_channels: c_int,
) {
    let nb_ebands = m.nb_ebands as usize;
    let eff_end = eff_end as usize;
    let end = end as usize;
    for (le_ch, be_ch) in band_log_e.chunks_mut(nb_ebands).zip(band_e.chunks(nb_ebands)).take(c_channels as usize) {
        for (i, ((le, &be), &em)) in le_ch.iter_mut().zip(be_ch).zip(E_MEANS.iter()).enumerate() {
            if i < eff_end {
                *le = celt_log2(shl32(be, 2)) - shl16(em as OpusVal16, 6);
            } else if i < end {
                *le = -qconst16(14.0, DB_SHIFT);
            }
            // bands >= end are left untouched, as in the C
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_e_prob_model_spot_check() {
        // Verify a few values from the probability model tables
        // 120 samples, inter, first pair
        assert_eq!(E_PROB_MODEL[0][0][0], 72);
        assert_eq!(E_PROB_MODEL[0][0][1], 127);
        // 960 samples, intra, last pair
        assert_eq!(E_PROB_MODEL[3][1][40], 77);
        assert_eq!(E_PROB_MODEL[3][1][41], 40);
    }

    #[test]
    fn test_small_energy_icdf() {
        assert_eq!(SMALL_ENERGY_ICDF, [2, 1, 0]);
    }

    #[test]
    fn test_e_means_spot_check() {
        #[cfg(not(feature = "fixed-point"))]
        {
            assert!((E_MEANS[0] - 6.4375).abs() < 1e-6);
            assert!((E_MEANS[24] - 3.75).abs() < 1e-6);
        }
        #[cfg(feature = "fixed-point")]
        {
            assert_eq!(E_MEANS[0], 103);
            assert_eq!(E_MEANS[24], 60);
        }
    }

    #[test]
    fn test_pred_coef_spot_check() {
        #[cfg(not(feature = "fixed-point"))]
        {
            assert!((PRED_COEF[0] - 29440.0 / 32768.0).abs() < 1e-6);
            assert!((PRED_COEF[3] - 16384.0 / 32768.0).abs() < 1e-6);
        }
        #[cfg(feature = "fixed-point")]
        {
            assert_eq!(PRED_COEF[0], 29440);
            assert_eq!(PRED_COEF[3], 16384);
        }
    }
}
