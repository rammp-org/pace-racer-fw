import os
# --- repo-relative paths -------------------------------------------------
# Run from anywhere: everything resolves off this file's location. The two
# machine-specific bits are overridable by environment variable.
HERE  = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))   # .../halls-sensorless/tests
DATA  = os.path.abspath(os.path.join(HERE, "..", "data"))
TELEM_SUB = os.environ.get("TELEM_SUB", os.path.join(TESTS, "host_sub", "build", "telem_sub"))
BENCH_LIB = os.environ.get("BENCH_LIB", "")   # dyno bench_lib.py dir; only the spin/bringup scripts need it

import sys, time, serial
sys.path.insert(0, TESTS)
import prlib

def reset():
    p = serial.Serial(prlib.PORT, 115200, timeout=0.3)
    p.dtr=False; p.rts=True; time.sleep(0.2); p.rts=False
    t0=time.time()
    while time.time()-t0 < 5.0: p.read(16384)
    p.close()

for w in (4, 5, 6, 8):
    reset()
    prlib.console("eth", 5.0)
    r = prlib.console_first("rtps %d 0" % w, "#rtps", 8.0)
    if r is None:
        r = [l for l in prlib.console("", 1.0) if l.startswith("!")]
        r = r[0] if r else "(no reply)"
    st = prlib.console_first("rtpstat", "#rtpstat", 2.5)
    print("W=%d -> %s" % (w, r))
    print("        %s" % st)
