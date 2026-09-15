"""Fig 6: approach the max-power edge from below; on stall, release fully,
recover, re-approach. Time-boxed 120 s of loaded running."""
import sys, time, re, json, os, statistics
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
import serial as pyserial
from bench_lib import Board, Brake, Scope, PSU

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig6"
NUM = r"(-?(?:[\d.]+|nan))"
srow = re.compile(r"^(\d+\.\d+), ([A-Z]), " + ", ".join([NUM]*5) +
                  r", (-?\d+), (-?\d+), (-?\d+), (-?\d+), " + ", ".join([NUM]*5))
def f(x):
    try: return float(x)
    except ValueError: return float("nan")

psu = pyserial.Serial(PSU, 115200, timeout=1.0)
def bus():
    psu.reset_input_buffer(); psu.write(b"MEAS:VOLT?\n"); time.sleep(0.12)
    v = float(psu.read(100).decode().strip() or 0)
    psu.write(b"MEAS:CURR?\n"); time.sleep(0.12)
    i = float(psu.read(100).decode().strip() or 0)
    return v, i
psu.write(b"CURR 15\n"); time.sleep(0.3)

sc = Scope(); b = Board(); brake = Brake(); brake.off()
log = []
try:
    b.ack("e 1", "#edge")
    for cmd, tok in (("lim 30 35", "#lim"), ("vl 20", "#vl"), ("vds 5", "#vds"), ("odg 8", "#ocp")):
        b.ack(cmd, tok)
    hs = b.cmd("hs", 0.6)
    if "cal=1" not in hs:
        b.ack("ho 9999 45 1", "#handoff"); b.ack("hcal 8", "#")
        b.ack("run 4 30 5", "#"); time.sleep(9); b.stop()
        b.ack("run 4 -30 5", "#"); time.sleep(9); b.stop()
    print("cooling 60 s before the run...", flush=True)
    time.sleep(60)
    b.stream_on()
    b.ack("hrun 30 150 5", "#hall speed")
    time.sleep(7)
    bv = 2.0
    brake.set(bv, settle=0.5, ilim=3.0)
    t0 = time.time(); buf = ""; stop_reason = "time box"; last_row_t = time.time()
    recoveries = 0; edge_v = None
    while time.time() - t0 < 120:
        time.sleep(1.0)
        buf += b.s.read(32000).decode(errors="replace")
        lines = buf.split("\n"); buf = lines[-1]
        row = None; faults = []
        for ln in lines[:-1]:
            if ln.startswith("!"): faults.append(ln)
            m = srow.match(ln.strip())
            if m: row = m
        if faults: stop_reason = "FAULT " + faults[0][:60]; break
        if not row:
            if time.time() - last_row_t > 5: stop_reason = "telemetry silent"; break
            continue
        last_row_t = time.time()
        v, i = bus()
        tq = sc.torque_nm(n=2, settle=0.08)
        temps = [f(row.group(k)) for k in (13, 14, 15, 16)]
        tmax = max((t for t in temps if t == t), default=float("nan"))
        rec = {"t": round(time.time()-t0, 1), "brake_v": bv, "bus_w": v*i, "bus_v": v, "bus_i": i,
               "iq": f(row.group(4)), "rpm": f(row.group(11)), "torque_nm": tq, "tmax": tmax}
        log.append(rec)
        print(f"t={rec['t']:4.0f}s bv={bv:4.2f} {rec['bus_w']:5.0f} W  iq={rec['iq']:5.1f}  "
              f"rpm={rec['rpm']:3.0f}  T={tq if tq is None else round(tq,1)}  tmax={tmax:.0f}C", flush=True)
        if tmax == tmax and tmax > 70: stop_reason = "temp"; break
        if rec["rpm"] < 100:                       # stalled: full release + recover
            recoveries += 1
            if recoveries > 3: stop_reason = "3 stalls"; break
            edge_v = bv
            brake.set(0.0, settle=0, ilim=3.0)
            print(f"  STALL at {bv:.2f} V — full release, recovering", flush=True)
            time.sleep(4.0)
            b.s.read(65536)
            bv = max(2.0, edge_v - 0.25)
            brake.set(bv, settle=0.5, ilim=3.0)
        elif rec["rpm"] > 140 and rec["iq"] < 28.5:
            step = 0.15 if (edge_v is None or bv < edge_v - 0.3) else 0.04
            bv = min(6.0, bv + step)
            brake.set(bv, settle=0, ilim=3.0)
    ws = [r["bus_w"] for r in log if r["rpm"] > 120]
    if ws:
        peak = max(ws)
        best10 = max(statistics.mean(ws[i:i+10]) for i in range(max(1, len(ws)-9)))
        print(f"\n{stop_reason}: PEAK (spinning) {peak:.0f} W, best 10 s avg {best10:.0f} W", flush=True)
finally:
    try: brake.set(0.0, settle=0); brake.off()
    except Exception: pass
    try: b.cmd("stop", 0.5); time.sleep(1.5); b.stream_off()
    except OSError: pass
    try: psu.write(b"CURR 8\n"); psu.close()
    except OSError: pass
    os.makedirs(DATA, exist_ok=True)
    json.dump({"log": log}, open(os.path.join(DATA, "power150_edge.json"), "w"))
    print(f"saved {len(log)} rows", flush=True)
    for o in (sc, brake, b):
        try: o.close()
        except Exception: pass
