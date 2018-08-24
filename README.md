gnss-converters
===========

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
