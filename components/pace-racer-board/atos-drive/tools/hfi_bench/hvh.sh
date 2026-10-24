#!/bin/bash
# hvh.sh <tag> <drive cmd> [run_s]: xshd.sh (smooth-hfi-drive best set) with the injection level and its
# calibration as parameters (hfi-higher-vh, 2026-10-23). env VH (default 0.17), CAL ("ns_re ns_im off",
# default /tmp/hfi/cal12_017.txt), HN (hrtn line), plus xshd.sh's HM, HK, CQ, CFF, XSL, I0, G, EV, EXTRA.
D=$(cd "$(dirname "$0")" && pwd); cd /tmp/hfi; P=/tmp/hfi/venv/bin/python
VH=${VH:-0.17}; CAL=${CAL:-$(cat cal12_017.txt)}
G=${G:-"hrtg 600 100000 0.01 0 0.2 20"}
SET=("sg 0.02 0.5" "id 0" "hguard 60" "$G" "${HM:-hrtm 4000 8 0 4000 ${I0:-1.5} 4000}" "${HK:-hrtk -3600 -3500}" "hrtc 0 0 0 ${CQ:-6}"
     "${HN:-hrtn -0.0046 0.0412 -0.0484 0.0493}" "${CFF:-cff 1 1}" "${XSL:-xsl 0.6 0.015 4 0 30 1}" ${EXTRA:+"$EXTRA"} "hrt $VH $CAL@0.8")
RUN=${3:-10}
$P $D/xshd.py hvh/$1.npy ${EV:-160} $RUN "${SET[@]}" 'ARM@0.05' "$2@$(echo "$RUN+0.3" | bc)" 2>&1 | grep -vE "^> (id|hguard|hrtc)" | cut -c1-330
