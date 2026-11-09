source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
cd ~/mib_debug_claude/racer/tools
probe(){ for W in 192.168.4.4 192.168.4.3; do echo "$(date +%T) $1 $W | $(python3 eth_link.py $W info 2>&1 | grep -oE "uptime_s.:[0-9]+|timed out" | tr -d "\n") | $(python3 eth_link.py $W cmd atos 2>&1 | head -1 | grep -oE "rx=[0-9]+ tx|heap .*" | tr "\n" " ") | $(python3 eth_link.py $W cmd s 2>&1 | head -1 | grep -oE "\([0-9.]+%\)|cmax=[0-9]+us" | tr "\n" " ")"; done; }
probe before
python3 ../many.py ${1:-30} ${2:-60} 2>&1 | grep -vE "malformed|tev:" > /tmp/many.out &
P=$!; sleep 50; probe "loaded"
wait $P; cat /tmp/many.out; probe after
python3 ~/mib_debug_claude/mibstat.py 2>&1 | grep drive_
