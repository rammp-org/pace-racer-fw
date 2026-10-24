#!/bin/bash
# hvh_accept.sh [extra-setting]: hfi-higher-vh acceptance once the board boots this branch (UNTESTED on hardware with hrtb).
# Vh 0.20 V, cal /tmp/hfi/hvh/cal020.txt, hrtn x1.2 of the 0.17 V fit, zero-current windows every 10 ms (3 ms wide).
# Hold the bench lock; take the scope check first (scopecap.py, see report).
D=$(cd "$(dirname "$0")" && pwd)
export VH=0.20 CAL="$(cat /tmp/hfi/hvh/cal020.txt)" HN="hrtn -0.0054 0.0485 -0.0484 0.0493"
export EXTRA=${1:-"hrtb 10 3 1.5 3 0.5"}
for k in 1 2 3; do $D/hvh.sh ACC_f30_$k 'xrun 3 30 1' | grep RESULT; done
$D/hvh.sh ACC_f60 'xrun 3 60 1' | grep RESULT
$D/hvh.sh ACC_m30 'xrun 3 -30 1' | grep RESULT
$D/hvh.sh ACC_m60 'xrun 3 -60 1' | grep RESULT
