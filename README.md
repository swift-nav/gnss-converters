gnss-converters
===========

[![CI](https://github.com/swift-nav/gnss-converters/actions/workflows/ci.yaml/badge.svg)](https://github.com/swift-nav/gnss-converters/actions/workflows/ci.yaml)
[![codecov.io status][3]][4]

## Quick install

Pre-built binaries for various platforms are availale on the [releases page][5].
To quickly build and install the latest version, you can use Rust `cargo` tool.

First, [install rust][6]:

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Install build dependencies, and a nightly toolchain for Rust:

```
sudo apt install build-essential cmake clang sqlite3
```

Then build and install with `cargo`:

```
git clone --recursive git@github.com:swift-nav/gnss-converters.git
cd gnss-converters
cargo install --path .
```

To build and install the `nov2sbp` tool as well, also run:

```
cargo install --path . --bin nov2sbp --features nov2sbp
```

Or to build and install everything at once, run:

```
cargo install --path . --all-features
```

The following tools will be installed in `~/.cargo/bin` (which should be in the path):

- sbp2rtcm
- rtcm3tosbp
- ubx2sbp
- ixcom2sbp
- nov2sbp

## Build locally

This repository is a bit of a chimera; its products are the
gnss-converters library which provides funtionality for converting
RTCM to SBP, a tool written in C for converting RTCM to SBP, and a
variety of haskell tools that will convert between RTCM, SBP, and
json.

To build the C tool and the library, just follow the usual steps:

```
git submodule update --init --recursive
cd c/
mkdir build
cd build/
cmake ..
make -j8
```

Here is an example of how to run the C tool.  This should (eventually)
result in some colorful json on your terminal (the ntripping tool
can be found [on GitHub](https://github.com/swift-nav/ntripping)):

```
ntripping --url http://bmookerji:bmookerji@tiburon.geo.berkeley.edu:2101/MONB_RTCM3 | ./rtcm3tosbp | sbp2json | jq .
```

To install the Haskell tools:

```
stack install --resolver lts-10.10 sbp rtcm
```


[3]: https://img.shields.io/codecov/c/github/swift-nav/gnss-converters.svg?label=codecov.io&logo=codecov&style=flat-square
[4]: https://codecov.io/gh/swift-nav/gnss-converters
[5]: https://github.com/swift-nav/gnss-converters/releases
[6]: https://www.rust-lang.org/tools/install
