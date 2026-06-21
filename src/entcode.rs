// Translated from c/celt/entcode.c and c/celt/entcode.h (RFC 6716).
//
// Range coder shared state and utility functions.
// See RFC 6716 Section 4.1 for the range coding scheme.

// -- Constants from mfrngcod.h --

/// The number of bits to output at a time.
pub const EC_SYM_BITS: u32 = 8;
/// The total number of bits in each of the state registers.
pub const EC_CODE_BITS: u32 = 32;
/// The maximum symbol value.
pub const EC_SYM_MAX: u32 = (1 << EC_SYM_BITS) - 1;
/// Carry bit of the high-order range symbol.
pub const EC_CODE_TOP: u32 = 1 << (EC_CODE_BITS - 1);
/// The number of bits available for the last, partial symbol in the code field.
pub const EC_CODE_EXTRA: u32 = (EC_CODE_BITS - 2) % EC_SYM_BITS + 1;

// -- Constants from entcode.h --

/// The number of bits to use for the range-coded part of unsigned integers.
pub const EC_UINT_BITS: u32 = 8;
/// The resolution of fractional-precision bit usage measurements (3 => 1/8th bits).
pub const BITRES: u32 = 3;
/// ec_window is u32 (at least 32 bits).
pub const EC_WINDOW_SIZE: u32 = 32;

/// The entropy encoder/decoder context.
/// We use the same structure for both. See c/celt/entcode.h struct ec_ctx.
/// The decoder only ever reads the payload, so `buf` is a shared slice
/// borrowed for the decoder's lifetime (the C used a raw `*mut u8`).
pub struct EcCtx<'a> {
    /// Buffered input/output.
    pub buf: &'a [u8],
    /// The size of the buffer.
    pub storage: u32,
    /// The offset at which the last byte containing raw bits was read/written.
    pub end_offs: u32,
    /// Bits that will be read from/written at the end.
    pub end_window: u32,
    /// Number of valid bits in end_window.
    pub nend_bits: i32,
    /// The total number of whole bits read/written.
    pub nbits_total: i32,
    /// The offset at which the next range coder byte will be read/written.
    pub offs: u32,
    /// The number of values in the current range.
    pub rng: u32,
    /// Decoder: top of range minus input value minus one.
    /// Encoder: low end of current range.
    pub val: u32,
    /// Decoder: saved normalization factor from ec_decode().
    /// Encoder: number of outstanding carry propagating symbols.
    pub ext: u32,
    /// A buffered input/output symbol, awaiting carry propagation.
    pub rem: i32,
    /// Nonzero if an error occurred.
    pub error: i32,
}

// Type aliases matching C's typedefs.
#[allow(non_camel_case_types)]
pub type ec_dec<'a> = EcCtx<'a>;

impl EcCtx<'_> {
    /// All-zero context with an empty buffer — the safe replacement for
    /// the C pattern of declaring an uninitialised `ec_dec` on the stack
    /// before `ec_dec_init` fills it in.
    pub const fn empty() -> EcCtx<'static> {
        EcCtx {
            buf: &[],
            storage: 0,
            end_offs: 0,
            end_window: 0,
            nend_bits: 0,
            nbits_total: 0,
            offs: 0,
            rng: 0,
            val: 0,
            ext: 0,
            rem: 0,
            error: 0,
        }
    }
}

/// Integer logarithm: returns floor(log2(v)) + 1, or 0 if v == 0.
/// Branchless implementation matching c/celt/entcode.c ec_ilog().
///
/// On platforms with __builtin_clz (GCC/Clang), the C code uses that
/// instead via the EC_ILOG macro. This Rust version is the portable
/// fallback and is also used directly by Rust callers.
#[inline]
pub fn ec_ilog(mut v: u32) -> i32 {
    // On a Pentium M, this branchless version tested as the fastest on
    // 1,000,000,000 random 32-bit integers.
    let mut ret = (v != 0) as i32;
    let mut m = ((v & 0xFFFF0000) != 0) as i32 * 16;
    v >>= m as u32;
    ret |= m;
    m = ((v & 0xFF00) != 0) as i32 * 8;
    v >>= m as u32;
    ret |= m;
    m = ((v & 0xF0) != 0) as i32 * 4;
    v >>= m as u32;
    ret |= m;
    m = ((v & 0xC) != 0) as i32 * 2;
    v >>= m as u32;
    ret |= m;
    ret += ((v & 0x2) != 0) as i32;
    ret
}

/// Returns the number of bits "used" by the encoded or decoded symbols so far,
/// scaled by 2^BITRES (i.e. in 1/8th-bit precision).
///
/// Translated from c/celt/entcode.c ec_tell_frac().
/// See RFC 6716 Section 4.1.
pub fn ec_tell_frac(this: &EcCtx) -> u32 {
    let nbits = (this.nbits_total as u32) << BITRES;
    let mut l = ec_ilog(this.rng);
    let mut r = this.rng >> (l as u32 - 16);
    for _ in (0..BITRES).rev() {
        r = r * r >> 15;
        let b = (r >> 16) as i32;
        l = l << 1 | b;
        r >>= b as u32;
    }
    nbits - l as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ec_ilog() {
        assert_eq!(ec_ilog(0), 0);
        assert_eq!(ec_ilog(1), 1);
        assert_eq!(ec_ilog(2), 2);
        assert_eq!(ec_ilog(3), 2);
        assert_eq!(ec_ilog(4), 3);
        assert_eq!(ec_ilog(255), 8);
        assert_eq!(ec_ilog(256), 9);
        assert_eq!(ec_ilog(0x80000000), 32);
        assert_eq!(ec_ilog(0xFFFFFFFF), 32);
    }
}
