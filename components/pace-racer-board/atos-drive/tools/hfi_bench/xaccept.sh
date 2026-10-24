#!/bin/bash
# xaccept.sh xiq|xrun <tag> <governor rpm | rpm> [s]: acceptance runs with the model observer; env I0, G, SG, COG, EV override
# acceptance runs; model + current-saliency correction on
cd /tmp/hfi; CAL=$(cat cal12_017.txt); P=/tmp/hfi/venv/bin/python
OT=/Users/alxchunlin/orca/workspaces/pace-racer-fw/hfi-breakaway-tracking/components/pace-racer-board/atos-drive/tools/hfi_bench/otest.py
G=${G:-"hrtg 300 30000 0.01 0 0.2 20"}
SET=("${SG:-sg 0.05 0.2}" "id 0" "hguard 60" "$G" "hrtm 4000 8 0 1300 ${I0:-0}" "${COG:-hrtk -3000 -3100}" "hrtc 0 0 0" "hrtn -0.0046 0.0412 -0.0484 0.0493" "hrt 0.17 $CAL@0.8")
case "$1" in
  xiq)  $P $OT bt/$2.npy 40 3000 9 "${SET[@]}" 'ARM@0.05' "xiq 1 ${3:-200}@0.4" "xiq 2.5 ${3:-200}@2.1" ;;
  xrun) $P $OT bt/$2.npy ${EV:-200} 3000 9 "${SET[@]}" 'ARM@0.05' "xrun 3 $3 1@${4:-10.5}" ;;
esac 2>&1 | grep -vE "^> (id|hguard|hrt)" | cut -c1-260
