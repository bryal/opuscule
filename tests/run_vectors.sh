#!/bin/sh
# Test vector runner for both floating-point and fixed-point modes.
# Builds the binaries via cargo, then decodes all 12 test vectors
# as mono and stereo, comparing against the reference .dec files.
#
# Usage: tests/run_vectors.sh [rate]
#   rate defaults to 48000. Must be run from the repo root.

set -e

RATE=${1:-48000}
VECTOR_PATH=opus_testvectors

if [ ! -d "$VECTOR_PATH" ]; then
    echo "ERROR: Test vectors not found in $VECTOR_PATH"
    exit 1
fi

run_test_suite() {
    mode_label=$1   # "float" or "fixed-point"
    opus_demo=$2
    opus_compare=$3

    echo "========================================"
    echo "  Mode: $mode_label"
    echo "========================================"
    echo

    rm -f logs_mono.txt logs_stereo.txt

    echo "Testing mono"
    echo "------------"
    for file in 01 02 03 04 05 06 07 08 09 10 11 12; do
        if [ ! -e "$VECTOR_PATH/testvector$file.bit" ]; then
            echo "Bitstream file not found: testvector$file.bit"
            exit 1
        fi
        printf "  testvector%s ... " "$file"
        if ! "$opus_demo" -d "$RATE" 1 "$VECTOR_PATH/testvector$file.bit" tmp.out >> logs_mono.txt 2>&1; then
            echo "DECODE FAILED"
            exit 1
        fi
        if ! "$opus_compare" -r "$RATE" "$VECTOR_PATH/testvector$file.dec" tmp.out >> logs_mono.txt 2>&1; then
            echo "MISMATCH"
            exit 1
        fi
        echo "ok"
    done
    echo

    echo "Testing stereo"
    echo "--------------"
    for file in 01 02 03 04 05 06 07 08 09 10 11 12; do
        if [ ! -e "$VECTOR_PATH/testvector$file.bit" ]; then
            echo "Bitstream file not found: testvector$file.bit"
            exit 1
        fi
        printf "  testvector%s ... " "$file"
        if ! "$opus_demo" -d "$RATE" 2 "$VECTOR_PATH/testvector$file.bit" tmp.out >> logs_stereo.txt 2>&1; then
            echo "DECODE FAILED"
            exit 1
        fi
        if ! "$opus_compare" -s -r "$RATE" "$VECTOR_PATH/testvector$file.dec" tmp.out >> logs_stereo.txt 2>&1; then
            echo "MISMATCH"
            exit 1
        fi
        echo "ok"
    done
    echo

    mono_avg=$(grep quality logs_mono.txt | awk '{sum+=$4}END{printf "%.4f", sum/NR}')
    stereo_avg=$(grep quality logs_stereo.txt | awk '{sum+=$4}END{printf "%.4f", sum/NR}')
    echo "  Mono quality:   $mono_avg %"
    echo "  Stereo quality: $stereo_avg %"
    echo
}

# -- Float mode --
echo "Building (float)..."
cargo build --release 2>&1 | grep -E "^error" && exit 1 || true
run_test_suite "float" target/release/opus_demo target/release/opus_compare

# -- Fixed-point mode --
echo "Building (fixed-point)..."
cargo build --release --features fixed-point 2>&1 | grep -E "^error" && exit 1 || true
run_test_suite "fixed-point" target/release/opus_demo target/release/opus_compare

rm -f tmp.out logs_mono.txt logs_stereo.txt
echo "All tests passed (both float and fixed-point)."
