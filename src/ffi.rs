// FFI helpers for the Rust binaries that consume the Rust Opus API.

use core::ffi::c_char;

/// Convert a C string pointer to a Rust &str.
///
/// # Safety
/// The pointer must be non-null and point to a valid, static, UTF-8 C string.
pub unsafe fn c_str_to_str(ptr: *const c_char) -> &'static str {
    // SAFETY: by this fn's contract, `ptr` is non-null and points to a valid C string.
    unsafe { core::ffi::CStr::from_ptr(ptr) }.to_str().unwrap_or_else(|e| panic!("non-UTF8 C string: {e:?}"))
}
