import os
# --- repo-relative paths -------------------------------------------------
# Run from anywhere: everything resolves off this file's location. The two
# machine-specific bits are overridable by environment variable.
HERE  = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))   # .../halls-sensorless/tests
DATA  = os.path.abspath(os.path.join(HERE, "..", "data"))
TELEM_SUB = os.environ.get("TELEM_SUB", os.path.join(TESTS, "host_sub", "build", "telem_sub"))
BENCH_LIB = os.environ.get("BENCH_LIB", "")   # dyno bench_lib.py dir; only the spin/bringup scripts need it

import sys, time, subprocess, threading
sys.path.insert(0, TESTS)
import prlib

def stats():
    l = prlib.console_first("s", "#stats", read_s=2.0)
    return l

print("baseline (quiet):")
prlib.console("s", 1.5)            # clear counters
time.sleep(4)
print("  ", stats())

for pps, size in ((500, 64), (2000, 64), (5000, 64), (2000, 1400)):
    prlib.console("s", 1.5)        # reset counters
    t = threading.Thread(target=lambda: subprocess.run(
        ["python3",os.path.join(TESTS, "udp_load.py"),str(pps),str(size),"6"], capture_output=True))
    t.start(); t.join()
    print("rx load %d pps x %d B:" % (pps, size))
    print("  ", stats())
    print("  ", prlib.console_first("nstat", "#nstat", read_s=2.0))
