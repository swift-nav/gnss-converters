gnss-converters
===========

[![Build status][1]][2]
[![codecov.io status][3]][4]


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
result in some colorful json on your terminal:

```
ntripping --url http://bmookerji:bmookerji@tiburon.geo.berkeley.edu:2101/MONB_RTCM3 | ./rtcm3tosbp | sbp2json | jq .
```

To install the Haskell tools:

```
stack install --resolver lts-10.10 sbp rtcm
```


[1]: https://img.shields.io/travis/swift-nav/gnss-converters.svg?label=travis-build&logo=travis&style=flat-square
[2]: https://travis-ci.org/swift-nav/gnss-converters
[3]: https://img.shields.io/codecov/c/github/swift-nav/gnss-converters.svg?label=codecov.io&logo=codecov&style=flat-square
[4]: https://codecov.io/gh/swift-nav/gnss-converters
