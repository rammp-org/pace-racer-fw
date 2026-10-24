#!/bin/bash
# xshd.sh <tag> <drive cmd> [run_s]: smooth-drive runs (cogging/Coulomb feed-forward + xsl position loop) with
# the breakaway-tracking observer settings (xaccept.sh). env XSL, CFF, I0, G, EV override.
D=$(cd "$(dirname "$0")" && pwd); cd /tmp/hfi; CAL=$(cat cal12_017.txt); P=/tmp/hfi/venv/bin/python
G=${G:-"hrtg 600 100000 0.01 0 0.2 20"}
SET=("sg 0.02 0.5" "id 0" "hguard 60" "$G" "${HM:-hrtm 4000 8 0 1300 ${I0:-1.5}}" "${HK:-hrtk -3000 -3100}" "hrtc 0 0 0 ${CQ:-0}"
     "hrtn -0.0046 0.0412 -0.0484 0.0493" "${CFF:-cff 1 1}" "${XSL:-xsl 0.6 0.015 4 0 5 1}" "hrt 0.17 $CAL@0.8")
RUN=${3:-10}
$P $D/xshd.py shd/$1.npy ${EV:-160} $RUN "${SET[@]}" 'ARM@0.05' "$2@$(echo "$RUN+0.3" | bc)" 2>&1 | grep -vE "^> (id|hguard|hrtc)" | cut -c1-330
