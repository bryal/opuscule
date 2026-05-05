// Translated from c/celt/modes.c, c/celt/modes.h, and the static_modes headers (RFC 6716).
//
// Defines the CELTMode struct and the single standard Opus mode
// (48 kHz, 960 samples). Custom modes are not supported — only the
// static lookup path is implemented.

use crate::arch::OpusVal16;
use crate::kiss_fft::KissFftState;
use crate::mdct::MdctLookup;
use std::os::raw::c_int;

#[cfg(feature = "fixed-point")]
use crate::static_modes_fixed::*;
#[cfg(not(feature = "fixed-point"))]
use crate::static_modes_float::*;

// ---------------------------------------------------------------------------
// Structs
// ---------------------------------------------------------------------------

/// Pulse cache (precomputed bit allocation data).
/// Matches C's PulseCache in modes.h.
pub struct PulseCache {
    pub size: c_int,
    pub index: &'static [i16],
    pub bits: &'static [u8],
    pub caps: &'static [u8],
}

/// The mode definition struct.
/// C name: `struct OpusCustomMode`; `CELTMode` is a typedef alias.
pub struct OpusCustomMode {
    pub fs: i32,
    pub overlap: c_int,
    pub nb_ebands: c_int,
    pub eff_ebands: c_int,
    pub preemph: [OpusVal16; 4],
    pub ebands: &'static [i16],
    pub max_lm: c_int,
    pub nb_short_mdcts: c_int,
    pub short_mdct_size: c_int,
    pub nb_alloc_vectors: c_int,
    pub alloc_vectors: &'static [u8],
    pub log_n: &'static [i16],
    pub window: &'static [OpusVal16],
    pub mdct: MdctLookup,
    pub cache: PulseCache,
}

/// Matches `typedef struct OpusCustomMode CELTMode` in modes.h.
pub type CELTMode = OpusCustomMode;

unsafe impl Sync for OpusCustomMode {}

// ---------------------------------------------------------------------------
// Data arrays from modes.c
// ---------------------------------------------------------------------------

/// Bark-scale band edge indices for 5 ms short blocks at 48 kHz.
/// 21 bands + terminator = 22 entries.
pub static EBAND_5MS: [i16; 22] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 20, 24, 28, 34, 40, 48, 60, 78, 100];

/// Bit allocation table: 11 rate points × 21 bands = 231 entries.
/// Units of 1/32 bit/sample (0.1875 dB SNR).
pub static BAND_ALLOCATION: [u8; 231] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 90, 80, 75, 69, 63, 56, 49, 40, 34, 29, 20, 18, 10, 0, 0, 0,
    0, 0, 0, 0, 0, 110, 100, 90, 84, 78, 71, 65, 58, 51, 45, 39, 32, 26, 20, 12, 0, 0, 0, 0, 0, 0, 118, 110, 103, 93, 86, 80,
    75, 70, 65, 59, 53, 47, 40, 31, 23, 15, 4, 0, 0, 0, 0, 126, 119, 112, 104, 95, 89, 83, 78, 72, 66, 60, 54, 47, 39, 32, 25,
    17, 12, 1, 0, 0, 134, 127, 120, 114, 103, 97, 91, 85, 78, 72, 66, 60, 54, 47, 41, 35, 29, 23, 16, 10, 1, 144, 137, 130,
    124, 113, 107, 101, 95, 88, 82, 76, 70, 64, 57, 51, 45, 39, 33, 26, 15, 1, 152, 145, 138, 132, 123, 117, 111, 105, 98, 92,
    86, 80, 74, 67, 61, 55, 49, 43, 36, 20, 1, 162, 155, 148, 142, 133, 127, 121, 115, 108, 102, 96, 90, 84, 77, 71, 65, 59,
    53, 46, 30, 1, 172, 165, 158, 152, 143, 137, 131, 125, 118, 112, 106, 100, 94, 87, 81, 75, 69, 63, 56, 45, 20, 200, 200,
    200, 200, 200, 200, 200, 200, 198, 193, 188, 183, 178, 173, 168, 163, 158, 153, 148, 129, 104,
];

// ---------------------------------------------------------------------------
// FFT states (one per shift level 0..3)
// ---------------------------------------------------------------------------

// Safety: KissFftState contains raw pointers to static arrays.
unsafe impl Sync for KissFftState {}

#[cfg(not(feature = "fixed-point"))]
static FFT_STATE_0: KissFftState = KissFftState {
    nfft: 480,
    scale: 0.002083333,
    shift: -1,
    factors: [4, 120, 4, 30, 2, 15, 3, 5, 5, 1, 0, 0, 0, 0, 0, 0],
    bitrev: &FFT_BITREV480,
    twiddles: &FFT_TWIDDLES48000_960,
};

#[cfg(not(feature = "fixed-point"))]
static FFT_STATE_1: KissFftState = KissFftState {
    nfft: 240,
    scale: 0.004166667,
    shift: 1,
    factors: [4, 60, 4, 15, 3, 5, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    bitrev: &FFT_BITREV240,
    twiddles: &FFT_TWIDDLES48000_960,
};

#[cfg(not(feature = "fixed-point"))]
static FFT_STATE_2: KissFftState = KissFftState {
    nfft: 120,
    scale: 0.008333333,
    shift: 2,
    factors: [4, 30, 2, 15, 3, 5, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    bitrev: &FFT_BITREV120,
    twiddles: &FFT_TWIDDLES48000_960,
};

#[cfg(not(feature = "fixed-point"))]
static FFT_STATE_3: KissFftState = KissFftState {
    nfft: 60,
    scale: 0.016666667,
    shift: 3,
    factors: [4, 15, 3, 5, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    bitrev: &FFT_BITREV60,
    twiddles: &FFT_TWIDDLES48000_960,
};

#[cfg(feature = "fixed-point")]
static FFT_STATE_0: KissFftState = KissFftState {
    nfft: 480,
    shift: -1,
    factors: [4, 120, 4, 30, 2, 15, 3, 5, 5, 1, 0, 0, 0, 0, 0, 0],
    bitrev: &FFT_BITREV480,
    twiddles: &FFT_TWIDDLES48000_960,
};

#[cfg(feature = "fixed-point")]
static FFT_STATE_1: KissFftState = KissFftState {
    nfft: 240,
    shift: 1,
    factors: [4, 60, 4, 15, 3, 5, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    bitrev: &FFT_BITREV240,
    twiddles: &FFT_TWIDDLES48000_960,
};

#[cfg(feature = "fixed-point")]
static FFT_STATE_2: KissFftState = KissFftState {
    nfft: 120,
    shift: 2,
    factors: [4, 30, 2, 15, 3, 5, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0],
    bitrev: &FFT_BITREV120,
    twiddles: &FFT_TWIDDLES48000_960,
};

#[cfg(feature = "fixed-point")]
static FFT_STATE_3: KissFftState = KissFftState {
    nfft: 60,
    shift: 3,
    factors: [4, 15, 3, 5, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    bitrev: &FFT_BITREV60,
    twiddles: &FFT_TWIDDLES48000_960,
};

// ---------------------------------------------------------------------------
// The single standard Opus CELTMode
// ---------------------------------------------------------------------------


/// Float-mode preemphasis coefficients: {0.85000610, 0.0, 1.0, 1.0}
#[cfg(not(feature = "fixed-point"))]
const PREEMPH: [OpusVal16; 4] = [0.85000610, 0.0000000, 1.0000000, 1.0000000];

/// Fixed-point preemphasis coefficients: {27853, 0, 4096, 8192}
#[cfg(feature = "fixed-point")]
const PREEMPH: [OpusVal16; 4] = [27853, 0, 4096, 8192];

static MODE_48000_960_120: CELTMode = CELTMode {
    fs: 48000,
    overlap: 120,
    nb_ebands: 21,
    eff_ebands: 21,
    preemph: PREEMPH,
    ebands: &EBAND_5MS,
    max_lm: 3,
    nb_short_mdcts: 8,
    short_mdct_size: 120,
    nb_alloc_vectors: 11,
    alloc_vectors: &BAND_ALLOCATION,
    log_n: &LOG_N400,
    window: &WINDOW120,
    mdct: MdctLookup {
        n: 1920,
        maxshift: 3,
        kfft: [&FFT_STATE_0, &FFT_STATE_1, &FFT_STATE_2, &FFT_STATE_3],
        trig: &MDCT_TWIDDLES960,
    },
    cache: PulseCache { size: 392, index: &CACHE_INDEX50, bits: &CACHE_BITS50, caps: &CACHE_CAPS50 },
};

// ---------------------------------------------------------------------------
// opus_custom_mode_create — static lookup (no CUSTOM_MODES)
// ---------------------------------------------------------------------------

const OPUS_OK: c_int = 0;
const OPUS_BAD_ARG: c_int = -1;

/// Returns the standard CELTMode for the given (Fs, frame_size), or NULL.
/// Without CUSTOM_MODES this is just a table lookup — the only valid
/// combination is (48000, 960/480/240/120).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn opus_custom_mode_create(fs: i32, frame_size: c_int, error: *mut c_int) -> *const CELTMode {
    let mode = &MODE_48000_960_120;
    for j in 0..4 {
        if fs == mode.fs && (frame_size << j) == mode.short_mdct_size * mode.nb_short_mdcts {
            if !error.is_null() {
                unsafe { *error = OPUS_OK };
            }
            return mode as *const CELTMode;
        }
    }
    if !error.is_null() {
        unsafe { *error = OPUS_BAD_ARG };
    }
    std::ptr::null()
}
