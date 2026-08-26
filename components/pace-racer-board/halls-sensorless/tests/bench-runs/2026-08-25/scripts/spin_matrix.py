"""Control quality at 100 rpm across network conditions (motor free-shafted).

Always stops the motor on exit, including on exception/Ctrl-C.
"""
import os
# --- repo-relative paths -------------------------------------------------
# Run from anywhere: everything resolves off this file's location. The two
# machine-specific bits are overridable by environment variable.
HERE  = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))   # .../halls-sensorless/tests
DATA  = os.path.abspath(os.path.join(HERE, "..", "data"))
TELEM_SUB = os.environ.get("TELEM_SUB", os.path.join(TESTS, "host_sub", "build", "telem_sub"))
BENCH_LIB = os.environ.get("BENCH_LIB", "")   # dyno bench_lib.py dir; only the spin/bringup scripts need it

import sys, time, subprocess
sys.path.insert(0, TESTS)
import prlib

CAP = 15.0

def quality(label, dur=CAP):
    subprocess.run([sys.executable, os.path.join(TESTS, "foc_quality.py"), label, str(dur)])

def bg_udp(pps, size, secs):
    return subprocess.Popen([sys.executable, os.path.join(TESTS, "udp_load.py"), str(pps), str(size), str(secs)],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

try:
    prlib.console("rtpshz 0", 1.0)
    print("spin up: run 4 100 5")
    prlib.console("run 4 100 5", 10.0)
    time.sleep(8)                       # settle into closed loop
    prlib.console("s", 1.0)             # clear counters

    print("\n%-22s %s" % ("condition", "control quality over 15 s"))
    print("-"*118)

    # A: quiet -- no RTPS, no traffic
    quality("A quiet (no net)")
    print("   ", prlib.console_first("s", "#stats", 1.5))

    # B: RTPS publishing at the telemetry target rate
    prlib.console("rtpshz 10", 1.0); prlib.console("s", 1.0)
    quality("B rtps 10Hz")
    print("   ", prlib.console_first("s", "#stats", 1.5))

    # C: RTPS + inbound flood
    prlib.console("s", 1.0)
    p = bg_udp(2000, 64, int(CAP)+3)
    quality("C rtps + rx load")
    p.wait()
    print("   ", prlib.console_first("s", "#stats", 1.5))

    # D: bidirectional -- inbound flood + outbound burst
    prlib.console("etx max %d 512" % (int(CAP)+3), 0.8)
    prlib.console("s", 1.0)
    p = bg_udp(2000, 64, int(CAP)+3)
    quality("D bidirectional")
    p.wait()
    print("   ", prlib.console_first("s", "#stats", 1.5))
    print("   ", prlib.console_first("nstat", "#nstat", 1.5))

    # E: back to quiet -- confirms nothing drifted over the run
    prlib.console("rtpshz 0", 1.0); prlib.console("s", 1.0)
    quality("E quiet (repeat)")
    print("   ", prlib.console_first("s", "#stats", 1.5))

finally:
    print("\nSTOP")
    for l in prlib.console("stop", 4.0):
        print("  ", l)
