"""Control quality at 100 rpm under magnetic-brake load.

Brake is raised in steps with an abort check between each: aborts on thermal
rise, current near the clamp, or speed collapse. Motor is stopped and the brake
released on every exit path.
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
sys.path.insert(0, BENCH_LIB or sys.exit("set BENCH_LIB to the dyno bench/ directory"))
import prlib, bench_lib as bl

CAP = 15.0
brake = bl.Brake()

def row():
    rows = [r for r in prlib.console('', 1.6, keep_stream=True) if r and r[0].isdigit()]
    if not rows: return None
    p = [x.strip() for x in rows[-1].split(",")]
    return dict(mode=p[1], iq=float(p[3]), rpm=float(p[10]), tmax=max(float(x) for x in p[12:16]))

def check(tag):
    r = row()
    if r is None: raise RuntimeError("no stream")
    print("   %-14s mode=%s iq=%.2f A rpm=%.0f Tmax=%.1fC" % (tag, r["mode"], r["iq"], r["rpm"], r["tmax"]))
    if r["tmax"] > 60:  raise RuntimeError("ABORT: temp %.1f C" % r["tmax"])
    if r["iq"]  > 7.0:  raise RuntimeError("ABORT: iq %.2f A near clamp" % r["iq"])
    if r["rpm"] < 60:   raise RuntimeError("ABORT: speed collapse %.0f rpm" % r["rpm"])
    return r

try:
    prlib.console("rtpshz 0", 1.0)
    print("spin up: run 6 100 5")
    prlib.console("run 6 100 5", 10.0)
    time.sleep(8)
    check("free-shaft")

    for v in (0.6, 0.9, 1.2):   # stall edge measured between 1.0 and 1.8 V; stay under it
        brake.set(v, settle=1.5, ilim=3.0)
        print("brake -> %.1f V" % v)
        check("brake %.1fV" % v)

    prlib.console("s", 1.0)
    subprocess.run([sys.executable, os.path.join(TESTS, "foc_quality.py"), "L-A quiet loaded ~4A", str(CAP)])
    print("   ", prlib.console_first("s", "#stats", 1.5))
    check("post-quiet")

    prlib.console("rtpshz 10", 1.0)
    prlib.console("etx max %d 512" % (int(CAP)+3), 0.8)
    prlib.console("s", 1.0)
    p = subprocess.Popen([sys.executable, os.path.join(TESTS, "udp_load.py"), "2000", "64", str(int(CAP)+3)],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run([sys.executable, os.path.join(TESTS, "foc_quality.py"), "L-B bidir loaded ~4A", str(CAP)])
    p.wait()
    print("   ", prlib.console_first("s", "#stats", 1.5))
    check("post-bidir")

finally:
    print("\nRELEASE BRAKE + STOP")
    try: brake.off()
    except Exception as e: print("  brake off failed:", e)
    for l in prlib.console("stop", 4.0): print("  ", l)
    try: brake.close()
    except Exception: pass
