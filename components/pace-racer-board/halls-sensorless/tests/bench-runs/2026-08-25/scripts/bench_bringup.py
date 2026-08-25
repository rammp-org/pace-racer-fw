import os
# --- repo-relative paths -------------------------------------------------
# Run from anywhere: everything resolves off this file's location. The two
# machine-specific bits are overridable by environment variable.
HERE  = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))   # .../halls-sensorless/tests
DATA  = os.path.abspath(os.path.join(HERE, "..", "data"))
TELEM_SUB = os.environ.get("TELEM_SUB", os.path.join(TESTS, "host_sub", "build", "telem_sub"))
BENCH_LIB = os.environ.get("BENCH_LIB", "")   # dyno bench_lib.py dir; only the spin/bringup scripts need it

import sys, time
sys.path.insert(0, BENCH_LIB or sys.exit("set BENCH_LIB to the dyno bench/ directory"))
import bench_lib as bl

# 1. CH1 torque rail — read only, NEVER written.
br = bl.Brake()
tv, ti = br.torque_rail_meas()
print("1. CH1 torque rail: %.3f V / %.3f A" % (float(tv), float(ti)))
if abs(float(tv) - 24.0) > 1.0 or float(ti) < 0.05:
    print("   WARNING: expected 24.000 V / ~0.13 A (set by hand on the front panel)")

# 2. CH2 brake parked at zero, current limit per user spec (3 A, not the 1 A default).
br.dp.write(":SOUR2:CURR 3.000")
br.off()
print("2. CH2 brake: 0 V, output OFF, current limit 3.000 A")

# 3. VM up (Vset untouched).
p = bl.Psu()
print("3. BK output_on() before:", p.output_on())
if not p.output_on():
    p.output(True)
v = p.wait_vm()
print("3. BK VM rail: %.2f V (VM_MIN=%s)" % (float(v), bl.Psu.VM_MIN))

# 4. Board enumerates only once VM is up.
port = None
t0 = time.time()
while time.time() - t0 < 20:
    port = bl._find_board_port()
    if port: break
    time.sleep(1.0)
print("4. Board port:", port)
