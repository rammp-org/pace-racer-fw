"""Fig 5 staircase — single sweep-derived offset, torque via LAN scope."""
import sys, time, re, statistics
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from bench_lib import Board, Brake, Scope, save

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig5"
STEPS = [4, 6, 8, 10, 12, 14, 16, 18]
HOLD_S = 10.0

sc = Scope(); b = Board(); brake = Brake()
srow = re.compile(r"^(\d+\.\d+), ([A-Z]), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+)")

def torque_robust(retries=4):
    for _ in range(retries):
        t = sc.torque_nm(n=3, settle=0.15)
        if t is not None:
            return t
        time.sleep(0.4)
    return None

try:
    zero = torque_robust()
    print(f"torque zero: {zero:.3f} N·m-eq (scale TBD)", flush=True)
    out = {"zero_nm": zero, "ofs_deg": 352.5, "steps": []}
    b.ack("eiq 1", "#enc iq"); time.sleep(0.5)
    brake.set(2.5, settle=2.0)
    for amps in STEPS:
        b.stream_on()
        b.ack(f"eiq {amps}", "#enc iq")
        t0 = time.time(); iqs = []; buf = ""; faults = []; tq = None
        while time.time() - t0 < HOLD_S:
            time.sleep(0.15)
            buf += b.s.read(32000).decode(errors="replace")
            lines = buf.split("\n"); buf = lines[-1]
            for ln in lines[:-1]:
                if ln.startswith("!"): faults.append(ln)
                m = srow.match(ln.strip())
                if m: iqs.append(float(m.group(4)))
            if time.time() - t0 > 4.0 and tq is None:
                tq = torque_robust()
        b.stream_off()
        iq_meas = statistics.mean(iqs[-25:]) if iqs else 0.0
        rec = {"amps": amps, "iq_meas": iq_meas,
               "torque_raw": (tq - zero) if tq is not None else None,
               "faults": faults[:3]}
        out["steps"].append(rec)
        tpa = rec["torque_raw"] / iq_meas if rec["torque_raw"] and iq_meas > 0.5 else float("nan")
        print(f"{amps:3d} A: iq={iq_meas:5.2f}  T_raw={rec['torque_raw']}  T/A={tpa:6.4f}  "
              f"faults={len(faults)}", flush=True)
        save(DATA, "staircase_final", out)
        if faults:
            print("STOPPED ON FAULT:", faults[0][:80], flush=True)
            break
    # slip sentinel: repeat 8 A, compare to the sweep's 8 A point (~0.51)
    b.ack("eiq 8", "#enc iq"); time.sleep(2.5)
    t8 = torque_robust()
    out["recheck_8A"] = (t8 - zero) if t8 is not None else None
    print(f"slip sentinel 8 A recheck: {out['recheck_8A']} (sweep gave ~0.51)", flush=True)
    save(DATA, "staircase_final", out)
finally:
    try:
        b.cmd("eiq 0", 0.4); b.stop()
    except OSError: pass
    brake.off()
    for o in (sc, brake, b):
        try: o.close()
        except Exception: pass
