#!/bin/bash

export AFL_NO_UI=1

function terminate()
{
  trap '' SIGTERM
  echo 'Shutdown initiated...'
  kill 0
  wait
  echo 'Done'
  exit 0
}
trap terminate SIGINT SIGTERM

afl-fuzz -M master "$@" &

for i in $(seq $(("$(nproc --all)" - 1))) ; do
  afl-fuzz -S "slave-${i}" "$@"  &
done

wait