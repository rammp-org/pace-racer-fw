"""Fig 4 rerun WITH heatsinks: lock-rotor thermal staircase, same protocol as
the 2026-08-19 baseline, plus phase-C clamp current from the scope."""
import sys, time, re, json, os
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from bench_lib import Board, Brake, Scope, save

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig4"
TEMP_ABORT = 80.0
STEP_MAX_S = 300
PLATEAU_C_PER_MIN = 0.4
CLAMP_V_PER_A = 0.01

NUM = r"(-?(?:[\d.]+|nan))"
srow = re.compile(r"^(\d+\.\d+), ([A-Z]), " + ", ".join([NUM]*5) +
                  r", (-?\d+), (-?\d+), (-?\d+), (-?\d+), " + ", ".join([NUM]*5))
def f(x):
    try: return float(x)
    except ValueError: return float("nan")

# STALE (2026-08-26): CH2 was the phase-C current clamp; it is now the
# phase-B high-side source for the gate-drive session. Re-attach the clamp
# and pick a free channel before trusting the amps column below.
sc = Scope()
sc.w(":CHAN2:DISP ON"); sc.w(":MEAS:ITEM VAVG,CHAN2"); sc.w(":RUN")
time.sleep(1.0)
def phase_c_amps():
    try:
        v = float(sc.q(":MEAS:ITEM? VAVG,CHAN2"))
        return v / CLAMP_V_PER_A if abs(v) < 1e6 else None
    except (ValueError, OSError):
        return None
clamp_zero = phase_c_amps() or 0.0
print(f"clamp zero offset: {clamp_zero:.2f} A", flush=True)

b = Board()
b.arm_limits()
b.ack("odg 8", "#ocp")
brake = Brake(); brake.off(); brake.close()
b.ack("th 0", "#theta")
b.stream_on()

steps = [5.0, 10.0, 15.0, 20.0]
out = {"config": "heatsink", "clamp_zero_A": clamp_zero, "steps": []}
aborted = False
try:
    for amps in steps:
        if aborted: break
        b.ack(f"id {amps}", "#id*")
        series = []; t0 = time.time(); buf = ""; plateau = False
        next_clamp = t0 + 5
        clamp_a = None
        while time.time() - t0 < STEP_MAX_S and not plateau:
            time.sleep(0.4)
            buf += b.s.read(32000).decode(errors="replace")
            lines = buf.split("\n"); buf = lines[-1]
            for ln in lines[:-1]:
                if ln.startswith("!"):
                    print("FAULT:", ln[:90], flush=True); aborted = True
                m = srow.match(ln.strip())
                if m:
                    temps = [f(m.group(k)) for k in (13, 14, 15, 16)]
                    tmax = max((t for t in temps if t == t), default=float("nan"))
                    series.append((round(time.time()-t0, 2), tmax, *temps, f(m.group(3))))
            if aborted: break
            if time.time() > next_clamp:
                clamp_a = phase_c_amps()
                next_clamp = time.time() + 20
            if series and series[-1][1] == series[-1][1] and series[-1][1] > TEMP_ABORT:
                print(f"!! temp abort {series[-1][1]:.1f}C", flush=True); aborted = True; break
            if series and series[-1][0] > 90:
                recent = [s for s in series if s[0] > series[-1][0] - 60]
                if len(recent) > 10:
                    dT = recent[-1][1] - recent[0][1]
                    if dT / (recent[-1][0] - recent[0][0]) * 60 < PLATEAU_C_PER_MIN:
                        plateau = True
        final = series[-1] if series else None
        cc = (clamp_a - clamp_zero) if clamp_a is not None else None
        out["steps"].append({"amps": amps, "series": series, "plateau": plateau,
                             "final_tmax": final[1] if final else None,
                             "dur_s": final[0] if final else 0,
                             "phase_c_clamp_A": cc})
        print(f"step {amps}A: tmax={final[1] if final else '?'}C after "
              f"{final[0] if final else 0}s plateau={plateau} "
              f"phaseC_clamp={cc if cc is None else round(cc,2)}A (expect {-amps/2:.1f})", flush=True)
        with open(os.path.join(DATA, "thermal_heatsink.json"), "w") as fjs:
            json.dump(out, fjs)
finally:
    try:
        b.cmd("id 0", 0.3); b.stop(); b.cmd("f", 0.3); b.stream_off()
    except OSError: pass
    for o in (sc, b):
        try: o.close()
        except Exception: pass
print("heatsink staircase complete" + (" (ABORTED)" if aborted else ""), flush=True)
