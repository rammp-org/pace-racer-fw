import os
# --- repo-relative paths -------------------------------------------------
# Run from anywhere: everything resolves off this file's location. The two
# machine-specific bits are overridable by environment variable.
HERE  = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))   # .../halls-sensorless/tests
DATA  = os.path.abspath(os.path.join(HERE, "..", "data"))
TELEM_SUB = os.environ.get("TELEM_SUB", os.path.join(TESTS, "host_sub", "build", "telem_sub"))
BENCH_LIB = os.environ.get("BENCH_LIB", "")   # dyno bench_lib.py dir; only the spin/bringup scripts need it

import sys, time, socket, threading
sys.path.insert(0, TESTS)
import prlib

def flat_out(nbytes, secs=6):
    res = {}
    def rx():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        s.bind((prlib.host_addr(), prlib.UDP_PORT))
        s.settimeout(0.5)
        s.sendto(b"probe", (prlib.BOARD_IP, prlib.UDP_PORT))
        pkts = total = 0; t0 = None
        end = time.time() + secs + 3
        while time.time() < end:
            try: d,_ = s.recvfrom(2048)
            except socket.timeout: continue
            if t0 is None: t0 = time.time()
            pkts += 1; total += len(d)
        s.close()
        el = (time.time()-t0) if t0 else 0
        res['host'] = (pkts, total, el)
    t = threading.Thread(target=rx); t.start()
    time.sleep(1.0)
    fw = prlib.console("etx max %d %d" % (secs, nbytes), read_s=secs+4)
    t.join()
    pkts, total, el = res.get('host', (0,0,0))
    print("--- %d B payload ---" % nbytes)
    for l in fw:
        if l.startswith("#"): print("  FW  ", l)
    if el > 0:
        print("  HOST  %d pkts, %d B in %.2fs -> %.0f pps, %.1f kB/s"
              % (pkts, total, el, pkts/el, total/1024.0/el))
    else:
        print("  HOST  nothing arrived")

for n in (64, 1400):
    flat_out(n)
    time.sleep(1.0)
