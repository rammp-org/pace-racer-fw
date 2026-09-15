"""Fig 5: encoder torque linearity — eiq staircase at stall.
Torque from the MHO984 over LAN (0-10 V = 0-200 N·m), smooth rotating ecal
per step, stream quieted except while parsing holds."""
import sys, time, re, statistics
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from bench_lib import Board, Brake, Scope, save

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig5"
STEPS = [4, 6, 8, 10, 12, 14, 16, 18]
HOLD_S = 12.0

sc = Scope()
b = Board()
brake = Brake()
srow = re.compile(r"^(\d+\.\d+), ([A-Z]), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+)")
try:
    zero_nm = sc.torque_nm()
    print(f"torque zero: {zero_nm:.3f} N·m", flush=True)
    out = {"zero_nm": zero_nm, "nm_per_v": Scope.NM_PER_V, "steps": []}
    for amps in STEPS:
        brake.off(); time.sleep(1.0)
        ofs, spread = b.ecal()
        b.ack("eiq 1", "#enc iq"); time.sleep(0.5)
        brake.set(2.5, settle=2.0)
        b.stream_on()
        b.ack(f"eiq {amps}", "#enc iq")
        t0 = time.time(); iqs = []; buf = ""; faults = []
        tq = None
        while time.time() - t0 < HOLD_S:
            time.sleep(0.15)
            buf += b.s.read(32000).decode(errors="replace")
            lines = buf.split("\n"); buf = lines[-1]
            for ln in lines[:-1]:
                if ln.startswith("!"): faults.append(ln)
                m = srow.match(ln.strip())
                if m: iqs.append(float(m.group(4)))
            if time.time() - t0 > 4.0 and tq is None:
                tq = sc.torque_nm()
        iq_meas = statistics.mean(iqs[-30:]) if iqs else 0.0
        b.ack("eiq 0", "#enc iq")
        b.stop()
        b.stream_off()
        rec = {"amps": amps, "iq_meas": iq_meas, "torque_nm": tq,
               "ecal_ofs": ofs, "ecal_spread": spread, "faults": faults[:3]}
        out["steps"].append(rec)
        tpa = (tq - zero_nm) / iq_meas if (tq is not None and iq_meas > 0.5) else float("nan")
        print(f"{amps:3d} A: iq={iq_meas:5.2f}  torque={tq:6.2f} N·m  T/A={tpa:5.3f}  "
              f"ecal={ofs:6.1f} (spread {spread:4.1f})  faults={len(faults)}", flush=True)
        save(DATA, "staircase_scope", out)
        if faults:
            print("STOPPED ON FAULT:", faults[0][:80], flush=True)
            break
    brake.off()
    ofs_end, spread_end = b.ecal()
    out["ecal_end"] = ofs_end
    save(DATA, "staircase_scope", out)
    print(f"final ecal: {ofs_end:.1f} (spread {spread_end:.1f})", flush=True)
finally:
    try:
        b.cmd("stop", 0.4)
    except OSError:
        pass
    for o in (sc, brake, b):
        try: o.close()
        except Exception: pass
