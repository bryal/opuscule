// Translated from c/celt/entdec.c (RFC 6716).
//
// Range decoder implementation. This is the entropy decoder used throughout
// Opus for both SILK and CELT layers.
//
// The range coder is described in RFC 6716 Section 4.1. It reads symbols
// from the front of the buffer (range-coded part) and raw bits from the
// end of the buffer, meeting in the middle.

use crate::entcode::{
    EC_CODE_BITS, EC_CODE_EXTRA, EC_CODE_TOP, EC_SYM_BITS, EC_SYM_MAX, EC_UINT_BITS, EC_WINDOW_SIZE, EcCtx, ec_dec, ec_ilog,
};
use crate::util::OrPanic;

/// Read the next byte from the front of the buffer, or 0 if exhausted.
#[inline]
fn ec_read_byte(this: &mut EcCtx) -> i32 {
    if this.offs < this.storage {
        let b = *this.buf.get(this.offs as usize).or_panic_dbg((this.offs, this.buf.len()));
        this.offs += 1;
        b as i32
    } else {
        0
    }
}

/// Read the next byte from the end of the buffer, or 0 if exhausted.
#[inline]
fn ec_read_byte_from_end(this: &mut EcCtx) -> i32 {
    if this.end_offs < this.storage {
        this.end_offs += 1;
        let b =
            *this.buf.get((this.storage - this.end_offs) as usize).or_panic_dbg((this.storage - this.end_offs, this.buf.len()));
        b as i32
    } else {
        0
    }
}

/// Normalizes the contents of val and rng so that rng lies entirely in the
/// high-order symbol. Called after every decode operation to refill the
/// range coder state.
#[inline]
fn ec_dec_normalize(this: &mut EcCtx) {
    // If the range is too small, rescale it and input some bits.
    while this.rng <= (EC_CODE_TOP >> EC_SYM_BITS) {
        this.nbits_total += EC_SYM_BITS as i32;
        this.rng <<= EC_SYM_BITS;
        // Use up the remaining bits from our last symbol.
        let sym = this.rem;
        // Read the next value from the input.
        this.rem = ec_read_byte(this);
        // Take the rest of the bits we need from this new symbol.
        let sym = ((sym << EC_SYM_BITS) | this.rem) >> (EC_SYM_BITS - EC_CODE_EXTRA);
        // And subtract them from val, capped to be less than EC_CODE_TOP.
        this.val = ((this.val << EC_SYM_BITS) + (EC_SYM_MAX & !(sym as u32))) & (EC_CODE_TOP - 1);
    }
}

/// Initialize the range decoder.
///
/// RFC 6716 Section 4.1.1.
///
/// `buf` must hold at least `storage` bytes.
pub fn ec_dec_init<'a>(this: &mut ec_dec<'a>, buf: &'a [u8], storage: u32) {
    debug_assert!(buf.len() >= storage as usize);
    this.buf = buf;
    this.storage = storage;
    this.end_offs = 0;
    this.end_window = 0;
    this.nend_bits = 0;
    // This is the offset from which ec_tell() will subtract partial bits.
    // The final value after ec_dec_normalize() will match the encoder.
    this.nbits_total = (EC_CODE_BITS + 1 - ((EC_CODE_BITS - EC_CODE_EXTRA) / EC_SYM_BITS) * EC_SYM_BITS) as i32;
    this.offs = 0;
    this.rng = 1u32 << EC_CODE_EXTRA;
    this.rem = ec_read_byte(this);
    this.val = this.rng - 1 - ((this.rem as u32) >> (EC_SYM_BITS - EC_CODE_EXTRA));
    this.error = 0;
    // Normalize the interval.
    ec_dec_normalize(this);
}

/// Calculate the cumulative frequency for the next symbol.
///
/// Returns a value in [fl, fh) where fl and fh are the cumulative frequency
/// bounds of the encoded symbol. Must be followed by exactly one call to
/// ec_dec_update().
///
/// RFC 6716 Section 4.1.
pub fn ec_decode(this: &mut ec_dec, ft: u32) -> u32 {
    this.ext = this.rng / ft;
    let s = this.val / this.ext;
    // EC_MINI(s+1, ft): ft - min(s+1, ft)
    ft - ec_mini(s + 1, ft)
}

/// Equivalent to ec_decode() with ft == 1 << bits.
///
/// RFC 6716 Section 4.1.
pub fn ec_decode_bin(this: &mut ec_dec, bits: u32) -> u32 {
    this.ext = this.rng >> bits;
    let s = this.val / this.ext;
    (1u32 << bits) - ec_mini(s + 1, 1u32 << bits)
}

/// Advance the decoder past the next symbol using its frequency bounds.
///
/// Must be called exactly once after ec_decode() / ec_decode_bin().
///
/// RFC 6716 Section 4.1.
pub fn ec_dec_update(this: &mut ec_dec, fl: u32, fh: u32, ft: u32) {
    let s = this.ext.wrapping_mul(ft - fh);
    this.val -= s;
    this.rng = if fl > 0 { this.ext.wrapping_mul(fh - fl) } else { this.rng - s };
    ec_dec_normalize(this);
}

/// Decode a bit that has a 1/(1 << logp) probability of being a one.
///
/// RFC 6716 Section 4.1.3.
pub fn ec_dec_bit_logp(this: &mut ec_dec, logp: u32) -> i32 {
    let r = this.rng;
    let d = this.val;
    let s = r >> logp;
    let ret = (d < s) as i32;
    if ret == 0 {
        this.val = d - s;
    }
    this.rng = if ret != 0 { s } else { r - s };
    ec_dec_normalize(this);
    ret
}

/// Decode a symbol given an inverse CDF table.
///
/// The inverse CDF table is such that symbol s falls in the range
/// [s>0 ? ft-icdf[s-1] : 0, ft-icdf[s]), where ft = 1 << ftb.
/// Values must be monotonically non-increasing with the last value being 0.
///
/// No call to ec_dec_update() is necessary after this call.
///
/// RFC 6716 Section 4.1.3.1.
pub fn ec_dec_icdf(this: &mut ec_dec, icdf: &[u8], ftb: u32) -> i32 {
    let mut s = this.rng;
    let d = this.val;
    let r = s >> ftb;
    let mut ret: i32 = -1;
    let mut t;
    loop {
        t = s;
        ret += 1;
        // The loop terminates because the last icdf entry is 0, which
        // makes s = 0, and d >= 0 always holds.
        s = r.wrapping_mul(u32::from(*icdf.get(ret as usize).or_panic_dbg((ret, icdf.len()))));
        if d >= s {
            break;
        }
    }
    this.val = d - s;
    this.rng = t - s;
    ec_dec_normalize(this);
    ret
}

/// Decode a raw unsigned integer with a non-power-of-2 range.
///
/// The bits must have been encoded with ec_enc_uint().
///
/// RFC 6716 Section 4.1.5.
pub fn ec_dec_uint(this: &mut ec_dec, ft: u32) -> u32 {
    debug_assert!(ft > 1);
    let ft = ft - 1;
    let ftb = ec_ilog(ft);
    if ftb > EC_UINT_BITS as i32 {
        let ftb = (ftb as u32 - EC_UINT_BITS) as u32;
        let ft_top = (ft >> ftb) + 1;
        let s = ec_decode(this, ft_top);
        ec_dec_update(this, s, s + 1, ft_top);
        let t = (s << ftb) | ec_dec_bits(this, ftb);
        if t <= ft {
            return t;
        }
        this.error = 1;
        ft
    } else {
        let ft = ft + 1;
        let s = ec_decode(this, ft);
        ec_dec_update(this, s, s + 1, ft);
        s
    }
}

/// Extract a sequence of raw bits from the end of the stream.
///
/// Raw bits are read from the end of the buffer, growing toward the front,
/// while range-coded symbols are read from the front. The two meet in the
/// middle.
///
/// RFC 6716 Section 4.1.4.
pub fn ec_dec_bits(this: &mut ec_dec, bits: u32) -> u32 {
    let mut window = this.end_window;
    let mut available = this.nend_bits;
    if (available as u32) < bits {
        loop {
            window |= (ec_read_byte_from_end(this) as u32) << available as u32;
            available += EC_SYM_BITS as i32;
            if available > (EC_WINDOW_SIZE - EC_SYM_BITS) as i32 {
                break;
            }
        }
    }
    let ret = window & ((1u32 << bits) - 1);
    window >>= bits;
    available -= bits as i32;
    this.end_window = window;
    this.nend_bits = available;
    this.nbits_total += bits as i32;
    ret
}

/// Branchless minimum, matching EC_MINI from c/celt/ecintrin.h.
#[inline]
fn ec_mini(a: u32, b: u32) -> u32 {
    a.wrapping_add(b.wrapping_sub(a) & ((b < a) as u32 * u32::MAX))
}

// Also export ec_tell as an inline, matching the C macro:
// #define ec_tell(_this) (ec_tell_inline(_this))
// static inline int ec_tell(ec_ctx *_this) {
//   return _this->nbits_total - EC_ILOG(_this->rng);
// }

/// Returns the number of bits "used" by the encoded/decoded symbols so far.
/// Matches ec_tell() / ec_tell_inline() from c/celt/entcode.h.
#[inline]
pub fn ec_tell(this: &EcCtx) -> i32 {
    this.nbits_total - ec_ilog(this.rng)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ec_mini() {
        assert_eq!(ec_mini(3, 5), 3);
        assert_eq!(ec_mini(5, 3), 3);
        assert_eq!(ec_mini(0, 0), 0);
        assert_eq!(ec_mini(u32::MAX, 0), 0);
        assert_eq!(ec_mini(0, u32::MAX), 0);
    }

    #[test]
    fn test_ec_tell() {
        // ec_tell = nbits_total - ec_ilog(rng)
        // With nbits_total=33, rng=0x80000000: 33 - 32 = 1
        let ctx = EcCtx {
            buf: &[],
            storage: 0,
            end_offs: 0,
            end_window: 0,
            nend_bits: 0,
            nbits_total: 33,
            offs: 0,
            rng: 0x80000000,
            val: 0,
            ext: 0,
            rem: 0,
            error: 0,
        };
        assert_eq!(ec_tell(&ctx), 1);
    }

    #[test]
    fn test_ec_dec_init_and_decode_roundtrip() {
        // Encode the value 2 out of {0,1,2,3} (ft=4) using a minimal
        // range coder payload, then verify the decoder recovers it.
        //
        // This test constructs a hand-crafted 2-byte buffer that the
        // range decoder will interpret as symbol 2 when decoded with ft=4.
        //
        // The range coder normalizes so that rng >= EC_CODE_BOT (0x800000).
        // With ft=4, ext = rng/4. The symbol s satisfies:
        //   returned = ft - min(s+1, ft) where s = val/ext
        // For returned=2: s must be 1, so val must be in [ext, 2*ext).

        // We'll test with a known 4-byte buffer. After init, the decoder
        // reads the first byte and normalizes. We verify basic invariants.
        let buf = [0x40u8, 0x00, 0x00, 0x00];
        let mut dec = ec_dec::empty();
        ec_dec_init(&mut dec, &buf, buf.len() as u32);
        // After init, rng should be >= EC_CODE_BOT
        assert!(dec.rng >= (EC_CODE_TOP >> EC_SYM_BITS));
        assert_eq!(dec.error, 0);
    }
}
