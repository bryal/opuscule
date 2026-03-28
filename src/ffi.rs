// FFI declarations for C Opus API functions that have not yet been
// translated to Rust. As each function is translated, its declaration
// here gets replaced by a re-export of the Rust implementation.
//
// These are the public Opus API functions from c/include/opus.h and
// c/include/opus_defines.h.

use std::os::raw::{c_char, c_int, c_uchar};

/// Opaque decoder type - we only hold a pointer, never inspect the layout.
#[repr(C)]
pub struct OpusDecoder {
    _opaque: [u8; 0],
}

// -- Constants from opus_defines.h --

pub const OPUS_OK: c_int = 0;
pub const OPUS_GET_FINAL_RANGE_REQUEST: c_int = 4031;

unsafe extern "C" {
    pub fn opus_get_version_string() -> *const c_char;
    pub fn opus_strerror(error: c_int) -> *const c_char;
    pub fn opus_decoder_create(fs: i32, channels: c_int, error: *mut c_int) -> *mut OpusDecoder;
    pub fn opus_decode(
        st: *mut OpusDecoder,
        data: *const c_uchar,
        len: i32,
        pcm: *mut i16,
        frame_size: c_int,
        decode_fec: c_int,
    ) -> c_int;
    pub fn opus_decoder_ctl(st: *mut OpusDecoder, request: c_int, ...) -> c_int;
    pub fn opus_decoder_destroy(st: *mut OpusDecoder);
}

/// Convert a C string pointer to a Rust &str.
///
/// # Safety
/// The pointer must be non-null and point to a valid, static, UTF-8 C string.
pub unsafe fn c_str_to_str(ptr: *const c_char) -> &'static str {
    unsafe { std::ffi::CStr::from_ptr(ptr) }.to_str().expect("non-UTF8 C string")
}
