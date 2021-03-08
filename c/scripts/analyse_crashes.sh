#!/bin/bash

TEST_DIR=$1
shift

ret=0
for t in ${TEST_DIR}/*; do
  echo $t
  $* < $t > /dev/null 2>> analyse.log
  if [ $? -ne 0 ] ; then
    ret=1
  fi
done

exit $ret
