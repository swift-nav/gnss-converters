#!/bin/bash

INPUT_FINDINGS_DIR=${1}
OUTPUT_CRASHES_DIR=${1}/crashes
OUTPUT_HANGS_DIR=${1}/hangs

for i in ${INPUT_FINDINGS_DIR}/*; do
  echo "Collecting $(ls ${i}/crashes/* | wc -l) crashes from ${i}"
  cp -f ${i}/crashes/* ${OUTPUT_CRASHES_DIR}/
  echo "Collecting $(ls ${i}/hangs/* | wc -l) hangs from ${i}"
  cp -f ${i}/hangs/* ${OUTPUT_HANGS_DIR}/
done

echo "Collected $(ls ${OUTPUT_CRASHES_DIR} | wc -l) crashes and $(ls ${OUTPUT_HANGS_DIR} | wc -l) hangs"
