"""Fig 6: >=500 W bus power — hall-commutated speed run under heavy brake.
Uses HALLS for commutation (fine at speed, immune to the encoder mount slip).
Requires: user go on stand torque + brake rating; BK Iset raised; possibly vl.
Sequence: hcal (brake off) -> ramp brake while at speed -> log bus V/I from BK.
"""
import sys, time, re, json
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
import serial
from bench_lib import Board, Brake, save, PSU

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig6"
TARGET_RPM = 300       # bemf ~12 V + IR: inside vl 20 up to ~16 A
AMPS = 20
BRAKE_RAMP = [0.5, 0.8, 1.0, 1.2, 1.4, 1.6]   # ramp until >=500 W or iq ceiling
HOLD_S = 6.0

psu = serial.Serial(PSU, 115200, timeout=1.0)
def bus():
    psu.reset_input_buffer(); psu.write(b"MEAS:VOLT?\n"); time.sleep(0.15)
    v = float(psu.read(100).decode().strip() or 0)
    psu.write(b"MEAS:CURR?\n"); time.sleep(0.15)
    i = float(psu.read(100).decode().strip() or 0)
    return v, i

# raise Iset to 15 A (Vset untouched)
psu.write(b"CURR 15\n"); time.sleep(0.3)

b = Board()
b.arm_limits()
brake = Brake(); brake.off()

# hall cal spin (I/f, both directions)
b.ack("ho 9999 45 1", "#handoff")
b.ack("hcal 8", "#")
b.ack("run 4 30 5", "#")
time.sleep(9); b.stop()
b.ack("run 4 -30 5", "#")
time.sleep(9); b.stop()
out_hs = b.cmd("hs", 0.5)
print(out_hs[:300], flush=True)
if "cal=1" not in out_hs:
    print("!! hcal incomplete — abort", flush=True); sys.exit(1)

srow = re.compile(r"^(\d+\.\d+), ([A-Z]), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?\d+), (-?\d+), (-?\d+), (-?\d+), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+)")
b.ack(f"hrun {AMPS} {TARGET_RPM} 6", "#hall speed")
time.sleep(8)
log = []
peak_w = 0
for bv in BRAKE_RAMP:
    brake.set(bv, settle=1.0)
    t0 = time.time(); buf = ""
    while time.time() - t0 < HOLD_S:
        time.sleep(0.3)
        buf += b.s.read(32000).decode(errors="replace")
        lines = buf.split("\n"); buf = lines[-1]
        row = None
        for ln in lines[:-1]:
            if ln.startswith("!"):
                print("FAULT:", ln, flush=True)
            m = srow.match(ln.strip())
            if m:
                row = m
        if row:
            v, i = bus()
            p = v * i
            peak_w = max(peak_w, p)
            rec = {"t": time.time(), "brake_v": bv, "bus_v": v, "bus_i": i, "bus_w": p,
                   "iq": float(row.group(4)), "rpm": float(row.group(11)),
                   "tmax": max(float(row.group(k)) for k in (13, 14, 15, 16))}
            log.append(rec)
            print(f"  brake {bv:.1f}V: {p:5.0f} W bus ({v:.1f}V {i:.2f}A) iq={rec['iq']:.1f} "
                  f"rpm={rec['rpm']:.0f} tmax={rec['tmax']:.0f}C", flush=True)
            if rec["tmax"] > 75:
                print("!! temp abort", flush=True); bv = 99; break
            if abs(rec["iq"]) > AMPS * 0.97 and rec["rpm"] < TARGET_RPM * 0.7:
                print("  iq ceiling + speed collapsing — stopping ramp", flush=True); bv = 99; break
    if bv == 99 or peak_w >= 550:
        break
brake.set(0.0, settle=0); brake.off()
b.stop()
save(DATA, "power", {"log": log, "peak_w": peak_w, "target_rpm": TARGET_RPM, "amps": AMPS})
print(f"\nPEAK BUS POWER: {peak_w:.0f} W", flush=True)
psu.write(b"CURR 8\n")  # restore
brake.close(); b.close(); psu.close()
