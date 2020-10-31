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

function build_rust () {
  cargo build --release
}

function build_c() {
    cd c
    mkdir build
    cd build
    /usr/bin/cmake -DCMAKE_INSTALL_PREFIX=./test_install ../
    make -j8 VERBOSE=1
    make install
    cd ../
    cd ../
}

function build_codecov() {
    cd c
    mkdir build
    cd build
    /usr/bin/cmake -DCODE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug ../
    make -j8 ccov-all
    cd ../
    cd ../
}

if [ "$TESTENV" == "stack" ]; then
  build_haskell
elif [ "$TESTENV" == "codecov" ]; then
  build_codecov
elif [ "$TESTENV" == "rust" ]; then
  build_rust
else
  build_c
fi
