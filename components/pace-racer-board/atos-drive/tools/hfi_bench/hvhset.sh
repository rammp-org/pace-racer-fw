# source hvhset.sh: SET array = hvh.sh's setup (same env overrides), for scopecap.py / other runners
VH=${VH:-0.17}; CAL=${CAL:-$(cat /tmp/hfi/cal12_017.txt)}
G=${G:-"hrtg 600 100000 0.01 0 0.2 20"}
SET=("sg 0.02 0.5" "id 0" "hguard 60" "$G" "${HM:-hrtm 4000 8 0 4000 ${I0:-1.5} 4000}" "${HK:-hrtk -3600 -3500}" "hrtc 0 0 0 ${CQ:-6}"
     "${HN:-hrtn -0.0046 0.0412 -0.0484 0.0493}" "${CFF:-cff 1 1}" "${XSL:-xsl 0.6 0.015 4 0 30 1}" ${EXTRA:+"$EXTRA"} "hrt $VH $CAL@0.8")
