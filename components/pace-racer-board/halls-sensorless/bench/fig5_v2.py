"""Fig 5 retry: gentler engage — pre-current before brake locks, then step up."""
import sys, time, re
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from bench_lib import Board, Brake, save

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig5"
STEPS = [4, 6, 8, 10, 12, 14, 16, 18]
HOLD_S = 15.0

b = Board()
brake = Brake()
srow = re.compile(r"^(\d+\.\d+), ([A-Z]), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+)")
out = {"steps": []}
for amps in STEPS:
    brake.off(); time.sleep(1.5)
    ofs, cons = b.ecal()
    # engage torque mode GENTLY while still unloaded, then lock the brake,
    # then step the current up — no coast->full-current transient
    b.ack("eiq 1", "#enc iq")
    time.sleep(0.8)
    brake.set(2.5, settle=2.5)
    b.ack(f"eiq {amps}", "#enc iq")
    print(f"=== {amps} A HOLD ({HOLD_S:.0f}s) — ecal {ofs:.1f} deg (cons {cons:.3f}) ===", flush=True)
    t0 = time.time(); iqs = []; buf = ""; faults = []
    while time.time() - t0 < HOLD_S:
        time.sleep(0.2)
        buf += b.s.read(32000).decode(errors="replace")
        lines = buf.split("\n"); buf = lines[-1]
        for ln in lines[:-1]:
            if ln.startswith("!"): faults.append(ln); print("  FAULT:", ln, flush=True)
            m = srow.match(ln.strip())
            if m: iqs.append(float(m.group(4)))
    iq_meas = sum(iqs[-30:]) / max(1, len(iqs[-30:]))
    if not faults:
        b.ack("eiq 0", "#enc iq")
        b.stop()
    out["steps"].append({"amps": amps, "iq_meas": iq_meas, "ecal_ofs": ofs,
                         "ecal_cons": cons, "faults": faults})
    print(f"    iq measured {iq_meas:.2f} A, faults={len(faults)}", flush=True)
    save(DATA, "staircase", out)
    if faults:
        print("STAIRCASE STOPPED ON FAULT", flush=True)
        break
brake.off()
print("\nper-step ecal offsets:", ", ".join(f"{s['ecal_ofs']:.1f}" for s in out["steps"]), flush=True)
brake.close(); b.close()
