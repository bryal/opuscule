#!/bin/sh
# Test vector runner using C opus_demo + Rust opus_compare.
# Adapted from c/run_vectors.sh.
#
# Usage: tests/run_vectors.sh <rate>
#   Must be run from the repo root.

set -e

if [ "$#" -ne "1" ]; then
    echo "usage: tests/run_vectors.sh <rate>"
    exit 1
fi

RATE=$1
OPUS_DEMO=target/release/opus_demo
OPUS_COMPARE=target/release/opus_compare
VECTOR_PATH=opus_testvectors

if [ ! -x "$OPUS_DEMO" ]; then
    echo "ERROR: opus_demo not found at $OPUS_DEMO (run 'cargo build --release' first)"
    exit 1
fi

if [ ! -x "$OPUS_COMPARE" ]; then
    echo "ERROR: Rust opus_compare not found at $OPUS_COMPARE (run 'cargo build' first)"
    exit 1
fi

if [ ! -d "$VECTOR_PATH" ]; then
    echo "ERROR: Test vectors not found in $VECTOR_PATH"
    exit 1
fi

rm -f logs_mono.txt logs_stereo.txt

echo "=============="
echo "Testing mono"
echo "=============="
echo

for file in 01 02 03 04 05 06 07 08 09 10 11 12
do
    if [ -e $VECTOR_PATH/testvector$file.bit ]; then
        echo "Testing testvector$file"
    else
        echo "Bitstream file not found: testvector$file.bit"
    fi
    if $OPUS_DEMO -d $RATE 1 $VECTOR_PATH/testvector$file.bit tmp.out >> logs_mono.txt 2>&1; then
        echo "successfully decoded"
    else
        echo "ERROR: decoding failed"
        exit 1
    fi
    $OPUS_COMPARE -r $RATE $VECTOR_PATH/testvector$file.dec tmp.out >> logs_mono.txt 2>&1
    float_ret=$?
    if [ "$float_ret" -eq "0" ]; then
        echo "output matches reference"
    else
        echo "ERROR: output does not match reference"
        exit 1
    fi
    echo
done

echo "=============="
echo "Testing stereo"
echo "=============="
echo

for file in 01 02 03 04 05 06 07 08 09 10 11 12
do
    if [ -e $VECTOR_PATH/testvector$file.bit ]; then
        echo "Testing testvector$file"
    else
        echo "Bitstream file not found: testvector$file"
    fi
    if $OPUS_DEMO -d $RATE 2 $VECTOR_PATH/testvector$file.bit tmp.out >> logs_stereo.txt 2>&1; then
        echo "successfully decoded"
    else
        echo "ERROR: decoding failed"
        exit 1
    fi
    $OPUS_COMPARE -s -r $RATE $VECTOR_PATH/testvector$file.dec tmp.out >> logs_stereo.txt 2>&1
    float_ret=$?
    if [ "$float_ret" -eq "0" ]; then
        echo "output matches reference"
    else
        echo "ERROR: output does not match reference"
        exit 1
    fi
    echo
done

echo "All tests have passed successfully"
grep quality logs_mono.txt | awk '{sum+=$4}END{print "Average mono quality is", sum/NR, "%"}'
grep quality logs_stereo.txt | awk '{sum+=$4}END{print "Average stereo quality is", sum/NR, "%"}'
