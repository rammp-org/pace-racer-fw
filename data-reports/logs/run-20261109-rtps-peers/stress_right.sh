source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
cd ~/mib_debug_claude/racer/tools
W=192.168.4.3; N=${1:-30}; D=${2:-60}
probe(){ echo "$(date +%T) $1 | $(python3 eth_link.py $W info 2>&1 | grep -oE "uptime_s\":[0-9]+|timed out" | tr -d "\n") | $(python3 eth_link.py $W cmd atos 2>&1 | head -1 | grep -oE "rx=[0-9]+ tx|matched cmd=[0-9]+|heap .*" | tr "\n" " ") | $(python3 eth_link.py $W cmd s 2>&1 | head -1 | grep -oE "\([0-9.]+%\)|cmax=[0-9]+us" | tr "\n" " ") | census $(tail -1 ~/mib_debug_claude/watch.jsonl | python3 -c "import sys,json; d=json.loads(sys.stdin.read()); print(\"jetson_n=%d wheels=%d\"%(d[\"jetson_n\"],len(d[\"wheels\"])))")"; }
probe before
python3 ../many.py $N $D > /tmp/many.out 2>&1 &
P=$!; sleep 15; probe "+15s"; sleep 20; probe "+35s"; sleep 22; probe "+57s"
wait $P; cat /tmp/many.out | tail -2; sleep 12; probe "after"
python3 ~/mib_debug_claude/mibstat.py 2>&1 | grep drive_right
