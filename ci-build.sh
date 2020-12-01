#!/bin/bash

# Run Travis setup

set -e
set -x
set -o errexit
set -o pipefail

function build_haskell () {
    cd haskell
    stack build --test
    cd ..
}

function build_rust () {
  VERBOSE=1 cargo build --all-features --all-targets --release -vv
}

function build_c() {
    cd c
    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=test_install -Dnov2sbp_BUILD=true ..
    make -j8 VERBOSE=1
    make do-all-tests
    make run_test_novatel_parser
    make install
    cd ../..
}

function build_codecov() {
    cd c
    mkdir build
    cd build
    cmake -DCODE_COVERAGE=ON -DCMAKE_BUILD_TYPE=Debug -Dnov2sbp_BUILD=true .. 2>&1 >cmake.log
    tail cmake.log
    make -j8 ccov-all 2>&1 >ccov.log
    tail ccov.log
    cd ../..
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

if [[ "$TESTENV" == "rust" ]] && [[ "$OS_NAME" == "windows" ]]; then
  cd target/release;
  strip.exe rtcm3tosbp.exe sbp2rtcm.exe ubx2sbp.exe ixcom2sbp.exe nov2sbp.exe;
  7z a -tzip ../../gnss_converters_windows.zip rtcm3tosbp.exe sbp2rtcm.exe ubx2sbp.exe ixcom2sbp.exe nov2sbp.exe;
  cd ../..;
  VERSION="$(git describe --always --tags)";
  BUILD_TRIPLET="$($CC -dumpmachine)";
  mv gnss_converters_windows.zip "gnss_converters-${VERSION}-windows-${BUILD_TRIPLET}.zip";
  echo "gnss_converters-${VERSION}-windows-${BUILD_TRIPLET}.zip" >release-archive.filename;
  ls -l;
fi

if [[ "$TESTENV" == "rust" ]] && [[ "$OS_NAME" == "osx" ]]; then
  (cd target/release; strip rtcm3tosbp sbp2rtcm ubx2sbp ixcom2sbp nov2sbp);
  tar -C "target/release" -czf gnss_converters_osx.tar.gz rtcm3tosbp sbp2rtcm ubx2sbp ixcom2sbp nov2sbp;
  VERSION="$(git describe --always --tags --dirty)";
  BUILD_TRIPLET="$($CC -dumpmachine)";
  mv gnss_converters_osx.tar.gz "gnss_converters-${VERSION}-${BUILD_TRIPLET}.tar.gz";
  echo "gnss_converters-${VERSION}-${BUILD_TRIPLET}.tar.gz" >release-archive.filename;
  ls -l;
fi

if [[ "$TESTENV" == "rust" ]] && [[ "$OS_NAME" == "linux" ]]; then
  (cd target/release; strip rtcm3tosbp sbp2rtcm ubx2sbp ixcom2sbp nov2sbp);
  tar -C "target/release" -czf gnss_converters_linux.tar.gz rtcm3tosbp sbp2rtcm ubx2sbp ixcom2sbp nov2sbp;
  VERSION="$(git describe --always --tags --dirty)";
  BUILD_TRIPLET="$($CC -dumpmachine)";
  mv gnss_converters_linux.tar.gz "gnss_converters-${VERSION}-${BUILD_TRIPLET}.tar.gz";
  echo "gnss_converters-${VERSION}-${BUILD_TRIPLET}.tar.gz" >release-archive.filename;
  ls -l;
fi
