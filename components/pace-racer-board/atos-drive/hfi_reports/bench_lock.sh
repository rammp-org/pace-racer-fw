#!/bin/bash
# Exclusive bench lock (board /dev/cu.usbmodem2101, Rigol PSU, MHO984 scope), FIFO-fair.
# usage: bench_lock.sh acquire <owner> [max_wait_s]   (blocks; exit 1 on timeout)
#        bench_lock.sh release <owner>
#        bench_lock.sh status
L=/tmp/hfi/bench.lock; Q=/tmp/hfi/bench.queue; mkdir -p "$Q"
case "$1" in
  acquire)
    owner="$2"; maxw="${3:-7200}"; t0=$(date +%s)
    tk="$Q/$(python3 -c 'import time;print(time.time_ns())')-$owner"; touch "$tk"
    trap 'rm -f "$tk"' EXIT
    while true; do
      # drop tickets of waiters that vanished (older than 3 h)
      find "$Q" -type f -mmin +180 -delete 2>/dev/null
      touch "$tk"
      head=$(ls "$Q" | sort | head -1)
      if [ "$Q/$head" = "$tk" ] && mkdir "$L" 2>/dev/null; then break; fi
      if [ $(( $(date +%s) - t0 )) -ge "$maxw" ]; then echo "TIMEOUT waiting; held by: $(cat $L/owner 2>/dev/null)"; exit 1; fi
      sleep 10
    done
    echo "$owner $(date '+%F %T')" > "$L/owner"; echo "ACQUIRED by $owner";;
  release)
    if [ "$(cut -d' ' -f1 $L/owner 2>/dev/null)" = "$2" ]; then rm -rf "$L"; echo "RELEASED by $2"; else echo "NOT OWNER (held by: $(cat $L/owner 2>/dev/null))"; exit 1; fi;;
  status) if [ -d "$L" ]; then echo "HELD by: $(cat $L/owner)"; else echo "FREE"; fi; echo "queue: $(ls $Q | sort | sed 's/^[0-9]*-//' | tr '\n' ' ')";;
  *) echo "usage: $0 acquire|release|status <owner>"; exit 2;;
esac
