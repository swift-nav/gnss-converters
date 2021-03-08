#!/bin/bash
set -e

CRASHES_DIR=${1}
shift
TMP_OUTPUT_DIR=${1}
shift

mkdir -p $TMP_OUTPUT_DIR

afl-cmin -i $CRASHES_DIR -o $TMP_OUTPUT_DIR -- $@

rm $CRASHES_DIR/*
cp $TMP_OUTPUT_DIR/* $CRASHES_DIR
rm $TMP_OUTPUT_DIR -rf