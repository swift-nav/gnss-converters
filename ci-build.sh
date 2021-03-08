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

    mkdir "${HOME}/.sonar"

    # download build-wrapper
    curl -sSLo "${HOME}/.sonar/build-wrapper-linux-x86.zip" https://sonarcloud.io/static/cpp/build-wrapper-linux-x86.zip
    unzip -o "${HOME}/.sonar/build-wrapper-linux-x86.zip" -d "${HOME}/.sonar/"
    export PATH=${HOME}/.sonar/build-wrapper-linux-x86:${PATH}

    # configure
    cmake --version
    cmake \
      "-DCODE_COVERAGE=ON" \
      "-DCMAKE_BUILD_TYPE=Debug" \
      "-Dnov2sbp_BUILD=true" \
      -S ./c -B ./build

    # build with wrapper
    build-wrapper-linux-x86-64 --out-dir ./bw-output cmake --build ./build --target ccov-all-export -j8

    if [[ -z "${SONAR_SCANNER_VERSION}" ]]; then
	echo "Error: SONAR_SCANNER_VERSION must be configured" >&2
	exit 1
    fi

    export SONAR_SCANNER_HOME="${HOME}/.sonar/sonar-scanner-${SONAR_SCANNER_VERSION}-linux"

    # download sonar-scanner
    curl -sSLo "${HOME}/.sonar/sonar-scanner.zip" \
      "https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-${SONAR_SCANNER_VERSION}-linux.zip"
    unzip -o "${HOME}/.sonar/sonar-scanner.zip" -d "${HOME}/.sonar/"
    export PATH=${SONAR_SCANNER_HOME}/bin:${PATH}
    export SONAR_SCANNER_OPTS="-server"

    # Run sonar scanner
    [[ -n "${SONAR_TOKEN:-}" ]] && SONAR_TOKEN_CMD_ARG="-Dsonar.login=${SONAR_TOKEN}"
    [[ -n "${SONAR_ORGANIZATION:-}" ]] && SONAR_ORGANIZATION_CMD_ARG="-Dsonar.organization=${SONAR_ORGANIZATION}"
    [[ -n "${SONAR_PROJECT_NAME:-}" ]] && SONAR_PROJECT_NAME_CMD_ARG="-Dsonar.projectName=${SONAR_PROJECT_NAME}"

    # TODO: setup sonar.projectVersion so that it actually does something useful
    #  see https://swift-nav.atlassian.net/browse/DEVINFRA-504
    SONAR_OTHER_ARGS="\
	-Dsonar.projectVersion=1.0 \
	-Dsonar.sources=. \
	-Dsonar.cfamily.build-wrapper-output=./bw-output \
	-Dsonar.cfamily.threads=1 \
	-Dsonar.cfamily.cache.enabled=false \
	-Dsonar.sourceEncoding=UTF-8"

    # shellcheck disable=SC2086
    sonar-scanner \
      "-Dsonar.cfamily.llvm-cov.reportPath=./build/ccov/coverage.txt" \
      "-Dsonar.host.url=${SONAR_HOST_URL}" \
      "-Dsonar.projectKey=${SONAR_PROJECT_KEY}" \
      "-Dsonar.exclusions=c/tests/**" \
      ${SONAR_OTHER_ARGS} \
      "${SONAR_PROJECT_NAME_CMD_ARG}" \
      "${SONAR_TOKEN_CMD_ARG}" \
      "${SONAR_ORGANIZATION_CMD_ARG}"
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
