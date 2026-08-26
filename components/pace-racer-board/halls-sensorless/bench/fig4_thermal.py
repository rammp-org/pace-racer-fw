"""Fig 4: lock-rotor thermal staircase — d-axis current hold, LM75 temps from stream."""
import sys, time, re, json, os
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from bench_lib import Board, Brake, save

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig4"
os.makedirs(DATA, exist_ok=True)
TEMP_ABORT = 75.0
STEP_MAX_S = 300
PLATEAU_C_PER_MIN = 0.4

# stream row: t, st, id, iq, iqref, vd, vq, aerr, rpmd, rpme, rpmh, flux, t0..t3
srow = re.compile(
    r"^(\d+\.\d+), ([A-Z]), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), "
    r"(-?\d+), (-?\d+), (-?\d+), (-?\d+), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+)")

b = Board()
b.arm_limits()
brake = Brake(); brake.off(); brake.close()
b.ack("th 0", "#theta")

steps = [5.0, 10.0, 15.0, 20.0]
out = {"steps": []}
aborted = False
for amps in steps:
    if aborted:
        break
    b.ack(f"id {amps}", "#id*")
    series = []          # (t, tmax, t0..t3, id_meas)
    t0 = time.time()
    buf = ""
    plateau = False
    while time.time() - t0 < STEP_MAX_S and not plateau:
        time.sleep(0.4)
        buf += b.s.read(32000).decode(errors="replace")
        lines = buf.split("\n"); buf = lines[-1]
        for ln in lines[:-1]:
            m = srow.match(ln.strip())
            if m:
                temps = [float(m.group(k)) for k in (13, 14, 15, 16)]
                series.append((round(time.time()-t0, 2), max(temps), *temps, float(m.group(3))))
        if series and series[-1][1] > TEMP_ABORT:
            print(f"!! ABORT: tmax {series[-1][1]:.1f}C at {amps}A", flush=True)
            aborted = True
            break
        # plateau: after 90 s, compare last-60s slope
        if series and series[-1][0] > 90:
            recent = [s for s in series if s[0] > series[-1][0] - 60]
            if len(recent) > 10:
                dT = recent[-1][1] - recent[0][1]
                if dT / (recent[-1][0] - recent[0][0]) * 60 < PLATEAU_C_PER_MIN:
                    plateau = True
    final = series[-1] if series else None
    out["steps"].append({"amps": amps, "series": series, "plateau": plateau,
                         "final_tmax": final[1] if final else None,
                         "dur_s": final[0] if final else 0})
    print(f"step {amps}A done: tmax={final[1] if final else '?'}C after {final[0] if final else 0}s "
          f"plateau={plateau}", flush=True)
    with open(os.path.join(DATA, "thermal.json"), "w") as f:
        json.dump(out, f)

b.cmd("id 0", 0.3)
b.stop()
b.cmd("f", 0.3)
print("thermal staircase complete" + (" (ABORTED)" if aborted else ""), flush=True)
b.close()
