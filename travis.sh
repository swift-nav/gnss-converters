#!/bin/bash

# Run Travis setup

set -e
set -x
set -o errexit
set -o pipefail

function build_haskell () {
    cd haskell
    stack build --test
    cd ../
}

function build_c() {
    cd c
    mkdir build
    cd build
    cmake ../
    make -j8 VERBOSE=1
    cd ../
    cd ../
}

function build_codecov() {
    cd c
    mkdir build
    cd build
    cmake -DCODE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug ../
    make -j8 ccov-all
    cd ../
    cd ../
}

if [ "$TESTENV" == "stack" ]; then
  build_haskell
elif [ "$TESTENV" == "codecov" ]; then
  build_codecov
else
  build_c
fi
