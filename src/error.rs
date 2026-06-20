//! Decoder error type.
//!
//! Internally the decoder uses libopus-style negative `c_int` codes; the
//! public API maps them to this enum at the boundary.

use core::ffi::c_int;
use core::fmt;

/// An error returned while decoding.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum Error {
    /// An argument was out of range or inconsistent (`OPUS_BAD_ARG`).
    BadArg,
    /// The output buffer was too small for the decoded frame
    /// (`OPUS_BUFFER_TOO_SMALL`).
    BufferTooSmall,
    /// An internal decoder invariant failed (`OPUS_INTERNAL_ERROR`).
    InternalError,
    /// The compressed packet was malformed (`OPUS_INVALID_PACKET`).
    InvalidPacket,
}

impl Error {
    /// Map an internal negative error code to an `Error` (used at the public
    /// API boundary; only called on the error path, so any unrecognised code
    /// is reported as [`Error::InternalError`]).
    pub(crate) fn from_code(code: c_int) -> Error {
        match code {
            -1 => Error::BadArg,         // OPUS_BAD_ARG
            -2 => Error::BufferTooSmall, // OPUS_BUFFER_TOO_SMALL
            -4 => Error::InvalidPacket,  // OPUS_INVALID_PACKET
            _ => Error::InternalError,   // OPUS_INTERNAL_ERROR and anything else
        }
    }

    fn message(self) -> &'static str {
        match self {
            Error::BadArg => "invalid argument",
            Error::BufferTooSmall => "output buffer too small",
            Error::InternalError => "internal decoder error",
            Error::InvalidPacket => "corrupted stream",
        }
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.message())
    }
}

impl core::error::Error for Error {}
