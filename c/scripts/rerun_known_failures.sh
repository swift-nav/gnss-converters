#!/bin/bash

CRASHES_DIR=${1}
shift
HANGS_DIR=${1}
shift
FIXED_DIR=${1}
shift

mkdir -p ${FIXED_DIR}/crashes ${FIXED_DIR}/hangs

CRASHES=0
FIXED=0
TOTAL_CRASHES=$(ls ${CRASHES_DIR}/* | wc -l)
TOTAL_HANGS=$(ls ${HANGS_DIR}/* | wc -l)

for i in ${CRASHES_DIR}/*; do
  if ! $* < ${i} > /dev/null 2>&1; then
    CRASHES=$((${CRASHES} + 1))
    echo ${i} is still failing
  else
    FIXED=$((${FIXED} + 1))
    echo ${i} has been fixed
    mv ${i} ${FIXED_DIR}/crashes
  fi
done

HANGS=0
for i in ${HANGS_DIR}/*; do
  echo "Testing hang ${i}..."
  if ! $* < ${i} > /dev/null 2>&1; then
    HANGS=$((${HANGS} + 1))
    echo still failing
  else
    FIXED=$((${FIXED} + 1))
    echo "fixed"
    mv ${i} ${FIXED_DIR}/hangs
  fi
done

echo ${CRASHES} of ${TOTAL_CRASHES} crashes still failing
echo ${HANGS} of ${TOTAL_HANGS} hangs stil failing
echo Fixed a total of ${FIXED} test cases with most recent change
