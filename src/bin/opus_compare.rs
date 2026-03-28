// Translated from c/src/opus_compare.c (RFC 6716 reference).
//
// Perceptual quality comparison tool for raw PCM16 audio files.
// Computes per-band spectral energy using a brute-force DFT, applies
// psychoacoustic masking (frequency, temporal, cross-talk), and produces
// a quality metric (pseudo-NMR on Bark-derived CELT bands).

use std::env;
use std::fs::File;
use std::io::Read;
use std::process;

const OPUS_PI: f32 = 3.14159265;

const NBANDS: usize = 21;
const NFREQS: usize = 240;

/// Bark-derived CELT bands for pseudo-NMR computation.
const BANDS: [usize; NBANDS + 1] = [0, 2, 4, 6, 8, 10, 12, 14, 16, 20, 24, 28, 32, 40, 48, 56, 68, 80, 96, 120, 156, 200];

const TEST_WIN_SIZE: usize = 480;
const TEST_WIN_STEP: usize = 120;

/// Read raw 16-bit little-endian PCM from a file, returning samples as f32.
/// Returns the number of frames (each frame = nchannels samples).
fn read_pcm16(fin: &mut File, nchannels: usize) -> (Vec<f32>, usize) {
    let mut raw = Vec::new();
    fin.read_to_end(&mut raw).expect("failed to read PCM file");
    let bytes_per_frame = 2 * nchannels;
    let nframes = raw.len() / bytes_per_frame;
    let mut samples = Vec::with_capacity(nframes * nchannels);
    for xi in 0..nframes {
        for ci in 0..nchannels {
            let off = 2 * (xi * nchannels + ci);
            let s = (raw[off + 1] as u32) << 8 | (raw[off] as u32);
            let s = ((s & 0xFFFF) ^ 0x8000).wrapping_sub(0x8000) as i32;
            samples.push(s as f32);
        }
    }
    (samples, nframes)
}

/// Compute per-band spectral energy via brute-force DFT with a Hann window.
///
/// - `out`: if Some, receives per-band average energy [nframes * nbands * nchannels]
/// - `ps`: power spectrum output [nframes * (window_sz/2) * nchannels]
/// - `bands`: band boundary table (nbands+1 entries)
/// - `input`: interleaved PCM samples
fn band_energy(
    mut out: Option<&mut [f32]>,
    ps: &mut [f32],
    bands: &[usize],
    nbands: usize,
    input: &[f32],
    nchannels: usize,
    nframes: usize,
    window_sz: usize,
    step: usize,
    downsample: usize,
) {
    let ps_sz = window_sz / 2;

    // Pre-compute Hann window, cos table, sin table
    let mut window = vec![0.0f32; window_sz];
    let mut c = vec![0.0f32; window_sz];
    let mut s = vec![0.0f32; window_sz];
    for xj in 0..window_sz {
        window[xj] = 0.5 - 0.5 * ((2.0 * OPUS_PI / (window_sz as f32 - 1.0)) * xj as f32).cos() as f32;
    }
    for xj in 0..window_sz {
        c[xj] = ((2.0 * OPUS_PI / window_sz as f32) * xj as f32).cos() as f32;
    }
    for xj in 0..window_sz {
        s[xj] = ((2.0 * OPUS_PI / window_sz as f32) * xj as f32).sin() as f32;
    }

    // Scratch for windowed samples: nchannels * window_sz
    let mut x = vec![0.0f32; nchannels * window_sz];

    for xi in 0..nframes {
        // Apply window
        for ci in 0..nchannels {
            for xk in 0..window_sz {
                x[ci * window_sz + xk] = window[xk] * input[(xi * step + xk) * nchannels + ci];
            }
        }
        // DFT per band
        let mut xj = 0usize;
        for bi in 0..nbands {
            let mut p = [0.0f32; 2];
            while xj < bands[bi + 1] {
                for ci in 0..nchannels {
                    let mut re = 0.0f32;
                    let mut im = 0.0f32;
                    let mut ti = 0usize;
                    for xk in 0..window_sz {
                        re += c[ti] * x[ci * window_sz + xk];
                        im -= s[ti] * x[ci * window_sz + xk];
                        ti += xj;
                        if ti >= window_sz {
                            ti -= window_sz;
                        }
                    }
                    re *= downsample as f32;
                    im *= downsample as f32;
                    ps[(xi * ps_sz + xj) * nchannels + ci] = re * re + im * im + 100000.0;
                    p[ci] += ps[(xi * ps_sz + xj) * nchannels + ci];
                }
                xj += 1;
            }
            if let Some(ref mut out) = out {
                let band_width = (bands[bi + 1] - bands[bi]) as f32;
                out[(xi * nbands + bi) * nchannels] = p[0] / band_width;
                if nchannels == 2 {
                    out[(xi * nbands + bi) * nchannels + 1] = p[1] / band_width;
                }
            }
        }
    }
}

fn main() {
    let argv: Vec<String> = env::args().collect();
    let argc = argv.len();
    if argc < 3 || argc > 6 {
        eprintln!("Usage: {} [-s] [-r rate2] <file1.sw> <file2.sw>", argv[0]);
        process::exit(1);
    }

    let mut argi = 1;
    let mut nchannels = 1usize;
    if argv[argi] == "-s" {
        nchannels = 2;
        argi += 1;
    }

    let mut rate = 48000u32;
    let mut ybands = NBANDS;
    let mut yfreqs = NFREQS;
    let mut downsample = 1usize;
    if argv[argi] == "-r" {
        rate = argv[argi + 1].parse().expect("invalid rate");
        if ![8000, 12000, 16000, 24000, 48000].contains(&rate) {
            eprintln!("Sampling rate must be 8000, 12000, 16000, 24000, or 48000");
            process::exit(1);
        }
        downsample = 48000 / rate as usize;
        ybands = match rate {
            8000 => 13,
            12000 => 15,
            16000 => 17,
            24000 => 19,
            _ => NBANDS,
        };
        yfreqs = NFREQS / downsample;
        argi += 2;
    }

    let mut fin1 = File::open(&argv[argi]).unwrap_or_else(|_| {
        eprintln!("Error opening '{}'.", argv[argi]);
        process::exit(1);
    });
    let mut fin2 = File::open(&argv[argi + 1]).unwrap_or_else(|_| {
        eprintln!("Error opening '{}'.", argv[argi + 1]);
        process::exit(1);
    });

    // File 1 (reference) is always read as stereo, then downmixed if mono
    let (mut x, xlength) = read_pcm16(&mut fin1, 2);
    if nchannels == 1 {
        for xi in 0..xlength {
            x[xi] = 0.5 * (x[2 * xi] + x[2 * xi + 1]);
        }
    }
    let (y, ylength) = read_pcm16(&mut fin2, nchannels);

    if xlength != ylength * downsample {
        eprintln!("Sample counts do not match ({}!={}).", xlength, ylength * downsample);
        process::exit(1);
    }
    if xlength < TEST_WIN_SIZE {
        eprintln!("Insufficient sample data ({}<{}).", xlength, TEST_WIN_SIZE);
        process::exit(1);
    }

    let nframes = (xlength - TEST_WIN_SIZE + TEST_WIN_STEP) / TEST_WIN_STEP;
    let mut xb = vec![0.0f32; nframes * NBANDS * nchannels];
    let mut big_x = vec![0.0f32; nframes * NFREQS * nchannels];
    let mut big_y = vec![0.0f32; nframes * yfreqs * nchannels];

    // Compute per-band spectral energy of the reference signal
    band_energy(Some(&mut xb), &mut big_x, &BANDS, NBANDS, &x, nchannels, nframes, TEST_WIN_SIZE, TEST_WIN_STEP, 1);

    // Compute power spectrum of the test signal
    band_energy(
        None,
        &mut big_y,
        &BANDS,
        ybands,
        &y,
        nchannels,
        nframes,
        TEST_WIN_SIZE / downsample,
        TEST_WIN_STEP / downsample,
        downsample,
    );

    // Psychoacoustic masking
    for xi in 0..nframes {
        // Frequency masking (low to high): 10 dB/Bark slope
        for bi in 1..NBANDS {
            for ci in 0..nchannels {
                let prev = xb[(xi * NBANDS + bi - 1) * nchannels + ci];
                xb[(xi * NBANDS + bi) * nchannels + ci] += 0.1 * prev;
            }
        }
        // Frequency masking (high to low): 15 dB/Bark slope
        for bi in (0..NBANDS - 1).rev() {
            for ci in 0..nchannels {
                let next = xb[(xi * NBANDS + bi + 1) * nchannels + ci];
                xb[(xi * NBANDS + bi) * nchannels + ci] += 0.03 * next;
            }
        }
        // Temporal masking: -3 dB/2.5ms slope
        if xi > 0 {
            for bi in 0..NBANDS {
                for ci in 0..nchannels {
                    let prev_frame = xb[((xi - 1) * NBANDS + bi) * nchannels + ci];
                    xb[(xi * NBANDS + bi) * nchannels + ci] += 0.5 * prev_frame;
                }
            }
        }
        // Cross-talk (stereo only)
        if nchannels == 2 {
            for bi in 0..NBANDS {
                let l = xb[(xi * NBANDS + bi) * nchannels];
                let r = xb[(xi * NBANDS + bi) * nchannels + 1];
                xb[(xi * NBANDS + bi) * nchannels] += 0.01 * r;
                xb[(xi * NBANDS + bi) * nchannels + 1] += 0.01 * l;
            }
        }
        // Apply masking to power spectra
        for bi in 0..ybands {
            for xj in BANDS[bi]..BANDS[bi + 1] {
                for ci in 0..nchannels {
                    let mask = 0.1 * xb[(xi * NBANDS + bi) * nchannels + ci];
                    big_x[(xi * NFREQS + xj) * nchannels + ci] += mask;
                    big_y[(xi * yfreqs + xj) * nchannels + ci] += mask;
                }
            }
        }
    }

    // Average consecutive frames to reduce sensitivity
    for bi in 0..ybands {
        for xj in BANDS[bi]..BANDS[bi + 1] {
            for ci in 0..nchannels {
                let mut xtmp = big_x[xj * nchannels + ci];
                let mut ytmp = big_y[xj * nchannels + ci];
                for xi in 1..nframes {
                    let xtmp2 = big_x[(xi * NFREQS + xj) * nchannels + ci];
                    let ytmp2 = big_y[(xi * yfreqs + xj) * nchannels + ci];
                    big_x[(xi * NFREQS + xj) * nchannels + ci] += xtmp;
                    big_y[(xi * yfreqs + xj) * nchannels + ci] += ytmp;
                    xtmp = xtmp2;
                    ytmp = ytmp2;
                }
            }
        }
    }

    // At lower rates, skip the last 300 Hz to allow for different transition
    // bands. At 12 kHz the last band already skips 400 Hz.
    let max_compare = if rate == 48000 {
        BANDS[NBANDS]
    } else if rate == 12000 {
        BANDS[ybands]
    } else {
        BANDS[ybands] - 3
    };

    // Compute weighted error
    let mut err = 0.0f64;
    for xi in 0..nframes {
        let mut ef = 0.0f64;
        for bi in 0..ybands {
            let mut eb = 0.0f64;
            for xj in BANDS[bi]..BANDS[bi + 1] {
                if xj >= max_compare {
                    break;
                }
                for ci in 0..nchannels {
                    let re = big_y[(xi * yfreqs + xj) * nchannels + ci] / big_x[(xi * NFREQS + xj) * nchannels + ci];
                    let mut im = re as f64 - (re as f64).ln() - 1.0;
                    // Less sensitive around SILK/CELT cross-over
                    if xj >= 79 && xj <= 81 {
                        im *= 0.1;
                    }
                    if xj == 80 {
                        im *= 0.1;
                    }
                    eb += im;
                }
            }
            eb /= (BANDS[bi + 1] - BANDS[bi]) as f64 * nchannels as f64;
            ef += eb * eb;
        }
        // Fixed normalization across all rates
        ef /= NBANDS as f64;
        ef *= ef;
        err += ef * ef;
    }
    err = (err / nframes as f64).powf(1.0 / 16.0);
    let q = 100.0 * (1.0 - 0.5 * (1.0 + err).ln() / 1.13f64.ln());

    if q < 0.0 {
        eprintln!("Test vector FAILS");
        eprintln!("Internal weighted error is {:.6}", err);
        process::exit(1);
    } else {
        eprintln!("Test vector PASSES");
        eprintln!("Opus quality metric: {:.1} % (internal weighted error is {:.6})", q, err);
    }
}
