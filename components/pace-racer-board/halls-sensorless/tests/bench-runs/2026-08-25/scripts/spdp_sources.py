import os
# --- repo-relative paths -------------------------------------------------
# Run from anywhere: everything resolves off this file's location. The two
# machine-specific bits are overridable by environment variable.
HERE  = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))   # .../halls-sensorless/tests
DATA  = os.path.abspath(os.path.join(HERE, "..", "data"))
TELEM_SUB = os.environ.get("TELEM_SUB", os.path.join(TESTS, "host_sub", "build", "telem_sub"))
BENCH_LIB = os.environ.get("BENCH_LIB", "")   # dyno bench_lib.py dir; only the spin/bringup scripts need it

import socket, struct, sys, time, collections
sys.path.insert(0, TESTS)
import prlib
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try: s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
except Exception: pass
s.bind(("", 7400))
s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
             socket.inet_aton("239.255.0.1")+socket.inet_aton("192.168.50.1"))
s.settimeout(1.0)
seen = collections.Counter()
end = time.time() + float(sys.argv[1])
while time.time() < end:
    try: d, a = s.recvfrom(4096)
    except socket.timeout: continue
    seen[(a[0], len(d))] += 1
s.close()
print("SPDP sources seen on 239.255.0.1:7400 ---")
for (ip, ln), n in sorted(seen.items()):
    who = "BOARD" if ip.endswith(".50") else ("HOST-SUB" if ip.endswith(".1") else "?")
    print("  %-16s %-9s %4d pkts  %d B" % (ip, who, n, ln))
if not seen: print("  (nothing)")
