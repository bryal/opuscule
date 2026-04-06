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

# -- Quality baseline checking --

BASELINE_FILE="tests/quality_baseline.txt"

read_baseline() {
    if [ ! -f "$BASELINE_FILE" ]; then
        echo "ERROR: Baseline file not found: $BASELINE_FILE"
        exit 1
    fi
    . "$BASELINE_FILE"
}

# Compare two quality scores. Returns 0 if actual >= expected, 1 otherwise.
# Uses awk because shell arithmetic doesn't handle floats.
is_worse() {
    awk "BEGIN { exit ($1 < $2) ? 0 : 1 }"
}

is_better() {
    awk "BEGIN { exit ($1 > $2) ? 0 : 1 }"
}

check_quality() {
    label=$1
    actual=$2
    expected=$3
    if is_worse "$actual" "$expected"; then
        echo "QUALITY REGRESSION: $label: expected $expected %, got $actual %"
        return 1
    fi
}

# -- Float mode --
echo "Building (float)..."
cargo build --release 2>&1 | grep -E "^error" && exit 1 || true
run_test_suite "float" target/release/opus_demo target/release/opus_compare
result_float_mono=$mono_avg
result_float_stereo=$stereo_avg

# -- Fixed-point mode --
echo "Building (fixed-point)..."
cargo build --release --features fixed-point 2>&1 | grep -E "^error" && exit 1 || true
run_test_suite "fixed-point" target/release/opus_demo target/release/opus_compare
result_fixed_mono=$mono_avg
result_fixed_stereo=$stereo_avg

rm -f tmp.out logs_mono.txt logs_stereo.txt

# -- Check against baseline --
read_baseline
failed=0
check_quality "float mono"    "$result_float_mono"    "$float_mono"    || failed=1
check_quality "float stereo"  "$result_float_stereo"  "$float_stereo"  || failed=1
check_quality "fixed mono"    "$result_fixed_mono"    "$fixed_mono"    || failed=1
check_quality "fixed stereo"  "$result_fixed_stereo"  "$fixed_stereo"  || failed=1

if [ "$failed" -eq 1 ]; then
    echo "FAILED: quality regression detected."
    exit 1
fi

# -- Update baseline if any score improved --
improved=0
is_better "$result_float_mono"    "$float_mono"    && improved=1
is_better "$result_float_stereo"  "$float_stereo"  && improved=1
is_better "$result_fixed_mono"    "$fixed_mono"    && improved=1
is_better "$result_fixed_stereo"  "$fixed_stereo"  && improved=1

if [ "$improved" -eq 1 ]; then
    cat > "$BASELINE_FILE" <<BASELINE
float_mono=$result_float_mono
float_stereo=$result_float_stereo
fixed_mono=$result_fixed_mono
fixed_stereo=$result_fixed_stereo
BASELINE
    echo "Baseline updated (quality improved)."
fi

echo "All tests passed (both float and fixed-point)."
