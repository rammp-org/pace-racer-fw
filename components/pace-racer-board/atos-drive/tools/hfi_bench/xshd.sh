#!/bin/bash
# xshd.sh <tag> <drive cmd> [run_s]: smooth-drive runs (cogging/Coulomb feed-forward + xsl position loop) with
# the best smooth-hfi-drive settings (2026-10-23). env HM, HK, CQ, CFF, XSL, I0, G, EV override
# (G replaces the hrtg line; G="hid 0.5" was used for d-bias tests).
D=$(cd "$(dirname "$0")" && pwd); cd /tmp/hfi; CAL=$(cat cal12_017.txt); P=/tmp/hfi/venv/bin/python
G=${G:-"hrtg 600 100000 0.01 0 0.2 20"}
SET=("sg 0.02 0.5" "id 0" "hguard 60" "$G" "${HM:-hrtm 4000 8 0 4000 ${I0:-1.5} 4000}" "${HK:-hrtk -3600 -3500}" "hrtc 0 0 0 ${CQ:-6}"
     "hrtn -0.0046 0.0412 -0.0484 0.0493" "${CFF:-cff 1 1}" "${XSL:-xsl 0.6 0.015 4 0 30 1}" "hrt 0.17 $CAL@0.8")
RUN=${3:-10}
$P $D/xshd.py shd/$1.npy ${EV:-160} $RUN "${SET[@]}" 'ARM@0.05' "$2@$(echo "$RUN+0.3" | bc)" 2>&1 | grep -vE "^> (id|hguard|hrtc)" | cut -c1-330
