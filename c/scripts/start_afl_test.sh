#!/bin/sh


echo "Start AFL test - $1 $2 $3 $*"

tmux_master="${1}-master"
tmux_sec1="${1}-sec1"
tmux_sec2="${1}-sec2"
tmux_sec3="${1}-sec3"
shift
testcases=${1}
shift
findings=${1}
shift

if tmux has-session ${tmux_master}; then
  echo "Master session already start ${tmux_master}"
  exit 1
fi
if tmux has-session ${tmux_sec1}; then
  echo "Secondary1 session already start ${tmux_sec1}"
  exit 1
fi
if tmux has-session ${tmux_sec2}; then
  echo "Secondary2 session already start ${tmux_sec2}"
  exit 1
fi
if tmux has-session ${tmux_sec3}; then
  echo "Secondary3 session already start ${tmux_sec3}"
  exit 1
fi

echo tmux new -d -n ${tmux_master} afl-fuzz -m 20971588 -i ${testcases} -o ${findings} -M ${tmux_master} $*
if ! tmux new -d -n ${tmux_master} afl-fuzz -m 20971588 -i ${testcases} -o ${findings} -M ${tmux_master} $*; then
  echo "Error starting master $?"
fi

if ! tmux new -d -n ${tmux_sec1} afl-fuzz -m 20971588 -i ${testcases} -o ${findings} -S ${tmux_sec1} $*; then
  echo "Error starting sec1 $?"
fi

if ! tmux new -d -n ${tmux_sec2} afl-fuzz -m 20971588 -i ${testcases} -o ${findings} -S ${tmux_sec2} $*; then
  echo "Error starting sec2 $?"
fi

if ! tmux new -d -n ${tmux_sec3} afl-fuzz -m 20971588 -i ${testcases} -o ${findings} -S ${tmux_sec3} $*; then
  echo "Error starting sec3 $?"
fi
