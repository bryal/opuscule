// FFI helpers for the Rust binaries that consume the Rust Opus API.

use std::os::raw::c_char;

/// Convert a C string pointer to a Rust &str.
///
/// # Safety
/// The pointer must be non-null and point to a valid, static, UTF-8 C string.
pub unsafe fn c_str_to_str(ptr: *const c_char) -> &'static str {
    unsafe { std::ffi::CStr::from_ptr(ptr) }.to_str().expect("non-UTF8 C string")
}
