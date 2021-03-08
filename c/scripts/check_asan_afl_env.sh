#!/bin/bash

if [ x"${AFL_USE_ASAN}" == "x1" ]; then
  echo "asan check ok"
  exit 0
fi

echo "asan-targets must be compiled with the environment var AFL_USE_ASAN=1"
echo "eg: "
echo "AFL_USE_ASAN=1 make asan-targets"

exit 1
