// Translated from c/celt/cwrs.c (RFC 6716).
//
// Combinatorial pulse coding (CWRS) -- Pyramid Vector Quantization.
// Encodes/decodes pulse vectors as indices into a combinatorial codebook.
// Only the decode path (decode_pulses) is needed for a decode-only build,
// but we also translate the supporting combinatorial functions it depends on.
//
// The codebook is indexed by two functions:
//   V(N,K) = number of N-dimensional unit pulse vectors with K pulses
//   U(N,K) = number of combinations where N-1 objects taken at most K-1 times
// See RFC 6716 Section 4.3.4.1 and the extensive comments in cwrs.c.

use std::os::raw::c_int;

use crate::entcode::ec_dec;
use crate::entdec::ec_dec_uint;
use crate::mathops::isqrt32;

const MASK32: u32 = 0xFFFFFFFF;

/// Multiplicative inverses of (2*i+1) mod 2^32, for i in [0..52].
static INV_TABLE: [u32; 53] = [
    0x00000001, 0xAAAAAAAB, 0xCCCCCCCD, 0xB6DB6DB7, 0x38E38E39, 0xBA2E8BA3, 0xC4EC4EC5, 0xEEEEEEEF, 0xF0F0F0F1, 0x286BCA1B,
    0x3CF3CF3D, 0xE9BD37A7, 0xC28F5C29, 0x684BDA13, 0x4F72C235, 0xBDEF7BDF, 0x3E0F83E1, 0x8AF8AF8B, 0x914C1BAD, 0x96F96F97,
    0xC18F9C19, 0x2FA0BE83, 0xA4FA4FA5, 0x677D46CF, 0x1A1F58D1, 0xFAFAFAFB, 0x8C13521D, 0x586FB587, 0xB823EE09, 0xA08AD8F3,
    0xC10C9715, 0xBEFBEFBF, 0xC0FC0FC1, 0x07A44C6B, 0xA33F128D, 0xE327A977, 0xC7E3F1F9, 0x962FC963, 0x3F2B3885, 0x613716AF,
    0x781948B1, 0x2B2E43DB, 0xFCFCFCFD, 0x6FD0EB67, 0xFA3F47E9, 0xD2FD2FD3, 0x3F4FD3F5, 0xD4E25B9F, 0x5F02A3A1, 0xBF5A814B,
    0x7C32B16D, 0xD3431B57, 0xD8FD8FD9,
];

/// Compute (a*b - c) / (2*d+1) when the quotient is exact.
/// Valid for d <= 52.
#[inline]
fn imusdiv32odd(a: u32, b: u32, c: u32, d: usize) -> u32 {
    debug_assert!(d <= 52);
    a.wrapping_mul(b).wrapping_sub(c).wrapping_mul(INV_TABLE[d]) & MASK32
}

/// Compute (a*b - c) / d when the quotient is exact and d is even.
/// Valid for d <= 54.
#[inline]
fn imusdiv32even(a: u32, b: u32, c: u32, d: usize) -> u32 {
    debug_assert!(d > 0);
    debug_assert!(d <= 54);
    // EC_ILOG on 32-bit value: floor(log2(x)) + 1
    let shift = crate::entcode::ec_ilog((d ^ (d - 1)) as u32) as u32;
    let inv = INV_TABLE[(d - 1) >> shift as usize];
    let shift = shift - 1;
    let one = 1u32 << shift;
    let mask = one - 1;
    (a.wrapping_mul(b >> shift)
        .wrapping_sub(c >> shift)
        .wrapping_add((a.wrapping_mul(b & mask).wrapping_add(one).wrapping_sub(c & mask)) >> shift)
        .wrapping_sub(1))
    .wrapping_mul(inv)
        & MASK32
}

// -- U(N,K) and V(N,K) for small N --

/// U(2, k) -- may be called with k = 32768.
#[inline]
fn ucwrs2(k: u32) -> u32 {
    debug_assert!(k > 0);
    k + (k - 1)
}

/// V(2, k)
#[inline]
fn ncwrs2(k: u32) -> u32 {
    debug_assert!(k > 0);
    4 * k
}

/// U(3, k) -- may be called with k = 32768.
#[inline]
fn ucwrs3(k: u32) -> u32 {
    debug_assert!(k > 0);
    (2u32.wrapping_mul(k).wrapping_sub(2)).wrapping_mul(k).wrapping_add(1)
}

/// V(3, k)
#[inline]
fn ncwrs3(k: u32) -> u32 {
    debug_assert!(k > 0);
    2u32.wrapping_mul(2u32.wrapping_mul(k).wrapping_mul(k).wrapping_add(1))
}

/// U(4, k)
#[inline]
fn ucwrs4(k: i32) -> u32 {
    debug_assert!(k > 0);
    let k = k as u32;
    imusdiv32odd(2u32.wrapping_mul(k), (2u32.wrapping_mul(k).wrapping_sub(3)).wrapping_mul(k).wrapping_add(4), 3, 1)
}

/// V(4, k)
#[inline]
fn ncwrs4(k: i32) -> u32 {
    debug_assert!(k > 0);
    let k = k as u32;
    (k.wrapping_mul(k).wrapping_add(2)).wrapping_mul(k) / 3 << 3
}

// -- Row computation --

/// Compute the next row/column of the recurrence u[i][j] = u[i-1][j] + u[i][j-1] + u[i-1][j-1].
fn unext(ui: &mut [u32], ui0_init: u32) {
    let len = ui.len();
    debug_assert!(len >= 2);
    let mut ui0 = ui0_init;
    let mut j = 1;
    loop {
        let ui1 = ui[j].wrapping_add(ui[j - 1]).wrapping_add(ui0);
        ui[j - 1] = ui0;
        ui0 = ui1;
        j += 1;
        if j >= len {
            break;
        }
    }
    ui[j - 1] = ui0;
}

/// Compute the previous row/column of the recurrence.
fn uprev(ui: &mut [u32], n: usize, ui0_init: u32) {
    debug_assert!(n >= 2);
    let mut ui0 = ui0_init;
    let mut j = 1;
    loop {
        let ui1 = ui[j].wrapping_sub(ui[j - 1]).wrapping_sub(ui0);
        ui[j - 1] = ui0;
        ui0 = ui1;
        j += 1;
        if j >= n {
            break;
        }
    }
    ui[j - 1] = ui0;
}

/// Compute V(n, k), filling u[0..=k+1] with U(n, 0..=k+1).
fn ncwrs_urow(n: usize, k: usize, u: &mut [u32]) -> u32 {
    let len = k + 2;
    debug_assert!(len >= 3);
    u[0] = 0;
    u[1] = 1;

    if n <= 6 {
        debug_assert!(n >= 2);
        debug_assert!(k > 0);
        for i in 2..len {
            u[i] = (i as u32) * 2 - 1;
        }
        for _ in 2..n {
            unext(&mut u[1..k + 2], 1);
        }
    } else {
        let n2m1 = (n as u32) * 2 - 1;
        let mut um1 = n2m1;
        let mut um2 = 1u32;
        u[2] = n2m1;
        let mut i = 3;
        while i < len {
            // U(N,K) = ((2*N-1)*U(N,K-1) - U(N,K-2))/(K-1) + U(N,K-2)
            um2 = imusdiv32even(n2m1, um1, um2, i - 1).wrapping_add(um2);
            u[i] = um2;
            i += 1;
            if i >= len {
                break;
            }
            um1 = imusdiv32odd(n2m1, um2, um1, (i - 1) >> 1).wrapping_add(um1);
            u[i] = um1;
            i += 1;
        }
    }
    u[k].wrapping_add(u[k + 1])
}

// -- Decoding: index -> pulse vector --

/// Decode 1-element combination.
#[inline]
fn cwrsi1(k: i32, i: u32, y: &mut [c_int]) {
    let s = -(i as i32);
    y[0] = (k + s) ^ s;
}

/// Decode 2-element combination.
#[inline]
fn cwrsi2(k: i32, i: u32, y: &mut [c_int]) {
    let mut i = i;
    let p = ucwrs2(k as u32 + 1);
    let s = if i >= p { -1i32 } else { 0 };
    i -= p & (s as u32);
    let mut yj = k;
    let k = ((i + 1) >> 1) as i32;
    let p = if k != 0 { ucwrs2(k as u32) } else { 0 };
    i -= p;
    yj -= k;
    y[0] = (yj + s) ^ s;
    cwrsi1(k, i, &mut y[1..]);
}

/// Decode 3-element combination.
fn cwrsi3(k: i32, i: u32, y: &mut [c_int]) {
    let mut i = i;
    let p = ucwrs3(k as u32 + 1);
    let s = if i >= p { -1i32 } else { 0 };
    i -= p & (s as u32);
    let mut yj = k;
    // Find max k such that ucwrs3(k) <= i.
    let k = if i > 0 { ((isqrt32(2u32.wrapping_mul(i).wrapping_sub(1)) + 1) >> 1) as i32 } else { 0 };
    let p = if k != 0 { ucwrs3(k as u32) } else { 0 };
    i -= p;
    yj -= k;
    y[0] = (yj + s) ^ s;
    cwrsi2(k, i, &mut y[1..]);
}

/// Decode 4-element combination.
fn cwrsi4(k: i32, i: u32, y: &mut [c_int]) {
    let mut i = i;
    let p = ucwrs4(k + 1);
    let s = if i >= p { -1i32 } else { 0 };
    i -= p & (s as u32);
    let mut yj = k;
    // Binary search for max k such that ucwrs4(k) <= i.
    let mut kl = 0i32;
    let mut kr = k;
    let mut k;
    let mut p;
    loop {
        k = (kl + kr) >> 1;
        p = if k != 0 { ucwrs4(k) } else { 0 };
        if p < i {
            if k >= kr {
                break;
            }
            kl = k + 1;
        } else if p > i {
            kr = k - 1;
        } else {
            break;
        }
    }
    i -= p;
    yj -= k;
    y[0] = (yj + s) ^ s;
    cwrsi3(k, i, &mut y[1..]);
}

/// General N-element decode: given index i and row U(n, 0..=k+1),
/// recover the pulse vector y[0..n].
fn cwrsi(n: usize, k: i32, i: u32, y: &mut [c_int], u: &mut [u32]) {
    debug_assert!(n > 0);
    let mut i = i;
    let mut k = k;
    for j in 0..n {
        let p_hi = u[k as usize + 1];
        let s = if i >= p_hi { -1i32 } else { 0 };
        i -= p_hi & (s as u32);
        let mut yj = k;
        let mut p = u[k as usize];
        while p > i {
            k -= 1;
            p = u[k as usize];
        }
        i -= p;
        yj -= k;
        y[j] = (yj + s) ^ s;
        uprev(u, k as usize + 2, 0);
    }
}

/// Decode a pulse vector of dimension n with k pulses from the entropy
/// decoder.
///
/// Called by alg_unquant() in vq.c during spectral decoding
/// (RFC 6716 Section 4.3.4.1).
///
/// # Safety
/// `y` must point to at least `n` writable i32 elements.
/// `dec` must be a valid decoder context.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn decode_pulses(y: *mut c_int, n: c_int, k: c_int, dec: *mut ec_dec) {
    debug_assert!(k > 0);
    let n = n as usize;
    // SAFETY: caller guarantees y points to n elements.
    let y = unsafe { std::slice::from_raw_parts_mut(y, n) };

    match n {
        2 => {
            let nc = ncwrs2(k as u32);
            // SAFETY: dec is a valid decoder.
            let i = unsafe { ec_dec_uint(dec, nc) };
            cwrsi2(k, i, y);
        }
        3 => {
            let nc = ncwrs3(k as u32);
            let i = unsafe { ec_dec_uint(dec, nc) };
            cwrsi3(k, i, y);
        }
        4 => {
            let nc = ncwrs4(k);
            let i = unsafe { ec_dec_uint(dec, nc) };
            cwrsi4(k, i, y);
        }
        _ => {
            let mut u = vec![0u32; k as usize + 2];
            let nc = ncwrs_urow(n, k as usize, &mut u);
            let i = unsafe { ec_dec_uint(dec, nc) };
            cwrsi(n, k, i, y, &mut u);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ucwrs2() {
        // U(2,1) = 1, U(2,2) = 3, U(2,3) = 5
        assert_eq!(ucwrs2(1), 1);
        assert_eq!(ucwrs2(2), 3);
        assert_eq!(ucwrs2(3), 5);
    }

    #[test]
    fn test_ucwrs3() {
        // U(3,1) = 1, U(3,2) = 5, U(3,3) = 13
        assert_eq!(ucwrs3(1), 1);
        assert_eq!(ucwrs3(2), 5);
        assert_eq!(ucwrs3(3), 13);
    }

    #[test]
    fn test_ncwrs_urow_n2() {
        // V(2, 3) = 12 (from the table in cwrs.c)
        let mut u = vec![0u32; 5]; // k+2 = 5
        let v = ncwrs_urow(2, 3, &mut u);
        assert_eq!(v, 12);
        // U(2,0)=0, U(2,1)=1, U(2,2)=3, U(2,3)=5, U(2,4)=7
        assert_eq!(u[0], 0);
        assert_eq!(u[1], 1);
        assert_eq!(u[2], 3);
        assert_eq!(u[3], 5);
        assert_eq!(u[4], 7);
    }

    #[test]
    fn test_ncwrs_urow_n3() {
        // V(3, 2) = 18 (from the table)
        let mut u = vec![0u32; 4];
        let v = ncwrs_urow(3, 2, &mut u);
        assert_eq!(v, 18);
    }

    #[test]
    fn test_cwrsi2_roundtrip() {
        // V(2,1) = 4, so there are 4 valid indices: 0,1,2,3
        // The pulse vectors for N=2, K=1 should be:
        // [1,0], [0,1], [0,-1], [-1,0]
        let mut y = [0i32; 2];
        cwrsi2(1, 0, &mut y);
        assert_eq!(y, [1, 0]);
        cwrsi2(1, 1, &mut y);
        assert_eq!(y, [0, 1]);
        cwrsi2(1, 2, &mut y);
        assert_eq!(y, [0, -1]);
        cwrsi2(1, 3, &mut y);
        assert_eq!(y, [-1, 0]);
    }
}
