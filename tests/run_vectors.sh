#!/bin/sh
# Test-vector runner against the RFC 8251 reference decode.
#
# Decodes every vector in opus_testvectors/ as mono and stereo, in both the
# float and fixed-point builds, and compares each against the right reference.
# It reports EVERY vector's quality (it does not stop at the first mismatch).
#
# Reference per channel mode (RFC 8251 section 10): a mono decoder disables
# phase inversion, so it targets the phase-inversion-disabled reference,
# testvectorNNm.dec; a stereo decoder keeps phase inversion and targets the
# standard testvectorNN.dec. tests/quality_baseline.txt records the expected
# failing set per build/channel, and ANY change - a passer regressing OR a
# known failure starting to pass - fails the run, so the baseline is updated
# by hand.
#
# Usage: tests/run_vectors.sh [rate]   (rate defaults to 48000; run from repo root)

set -e

RATE=${1:-48000}
VECTOR_PATH=opus_testvectors
BASELINE_FILE=tests/quality_baseline.txt
VECTORS="01 02 03 04 05 06 07 08 09 10 11 12"

[ -d "$VECTOR_PATH" ] || { echo "ERROR: test vectors not found in $VECTOR_PATH"; exit 1; }
[ -f "$BASELINE_FILE" ] || { echo "ERROR: baseline not found: $BASELINE_FILE"; exit 1; }
. "$BASELINE_FILE"

deviation=0

# Decode + compare every vector for one channel mode, printing each result and
# checking the resulting failing set against the baseline. Uses globals DEMO,
# CMP (set by run_suite) and sets `deviation` on any mismatch.
run_channel() {
    cname=$1; cnum=$2; sflag=$3; ref_suffix=$4; expected=$(echo $5)
    echo "  $cname:"
    fails=
    for f in $VECTORS; do
        "$DEMO" -d "$RATE" "$cnum" "$VECTOR_PATH/testvector$f.bit" tmp.out >/dev/null 2>&1 || true
        out=$("$CMP" $sflag -r "$RATE" "$VECTOR_PATH/testvector$f$ref_suffix" tmp.out 2>&1) || true
        if printf '%s\n' "$out" | grep -q PASSES; then
            q=$(printf '%s\n' "$out" | grep -oE 'metric: [0-9.]+' | grep -oE '[0-9.]+')
            printf "    testvector%s ... %s%%\n" "$f" "$q"
        else
            printf "    testvector%s ... FAIL\n" "$f"
            fails="$fails $f"
        fi
    done
    fails=$(echo $fails)
    if [ "$fails" = "$expected" ]; then
        if [ -n "$fails" ]; then echo "    (failing: $fails - matches baseline)"; else echo "    (all pass)"; fi
    else
        echo "    DEVIATION: failing now [$fails], baseline [$expected]"
        deviation=1
    fi
}

run_suite() {
    mode=$1; DEMO=$2; CMP=$3
    echo "===== $mode ====="
    # Mono disables phase inversion -> m.dec; stereo keeps it -> .dec.
    run_channel mono   1 ""   "m.dec" "$4"
    run_channel stereo 2 "-s" ".dec"  "$5"
    echo
}

echo "Building (float)..."
cargo build --release 2>&1 | grep -E "^error" && exit 1 || true
run_suite "float" target/release/opus_demo target/release/opus_compare "$float_mono_fail" "$float_stereo_fail"

echo "Building (fixed-point)..."
cargo build --release --features fixed-point 2>&1 | grep -E "^error" && exit 1 || true
run_suite "fixed-point" target/release/opus_demo target/release/opus_compare "$fixed_mono_fail" "$fixed_stereo_fail"

rm -f tmp.out

if [ "$deviation" -ne 0 ]; then
    echo "FAILED: the pass/fail set differs from the baseline."
    echo "Investigate; if the change is intended, update $BASELINE_FILE deliberately."
    exit 1
fi
echo "All vectors match the recorded baseline."
