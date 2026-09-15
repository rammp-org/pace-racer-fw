"""1 kW chase, MCP-coexistent: brake is set by the operator loop (MCP holds the
DP2031); this script requests values via ladder_req.json and waits for
ladder_ack.json. Safety does NOT depend on the operator: any abort does
b.stop() (unload + Hi-Z) immediately, brake release can lag."""
import sys, time, re, json, pathlib
sys.path.insert(0, "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/components/pace-racer-board/halls-sensorless/bench")
from bench_lib import Board, Scope, Psu, save

HERE = pathlib.Path(__file__).resolve().parent
REQ, ACK = HERE / "ladder_req.json", HERE / "ladder_ack.json"
DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/run-20260826-1kw"
pathlib.Path(DATA).mkdir(parents=True, exist_ok=True)
TARGET_RPM, AMPS, HOLD_S = 500, 30, 5.0
BRAKE_RAMP = [1.8, 2.3, 2.7, 3.0, 3.2, 3.4, 3.6, 3.8]
CLAMP_ABORT_A = 40.0

srow = re.compile(r"^(\d+\.\d+), ([A-Z]), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?\d+), (-?\d+), (-?\d+), (-?\d+), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+)")
sstat = re.compile(r"#stats n=(\d+) en=(\d) late=(\d+) \(([\d.]+)%\).*cmax=(\d+)us")
seq = 0

def want_brake(v):
    global seq
    seq += 1
    REQ.write_text(json.dumps({"brake_v": v, "seq": seq}))
    print(f"[req] brake -> {v} V (seq {seq})", flush=True)

def brake_acked(timeout, tick):
    """Wait for ack of current seq; runs `tick()` while waiting so the motor
    stays watched. Returns False on timeout."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        tick()
        try:
            a = json.loads(ACK.read_text())
            if a.get("seq") == seq:
                return True
        except (OSError, ValueError):
            pass
        time.sleep(0.4)
    return False

psu = Psu(); scope = Scope(); b = Board()
log, peak_w, abort = [], 0.0, None
state = {"row": None, "fault": None, "buf": ""}

def tick():
    state["buf"] += b.s.read(32000).decode(errors="replace")
    lines = state["buf"].split("\n"); state["buf"] = lines[-1]
    for ln in lines[:-1]:
        if ln.startswith("!") and state["fault"] is None:
            state["fault"] = ln.strip()
        m = srow.match(ln.strip())
        if m: state["row"] = m

try:
    psu.iset(24.0)
    for c, tok in (("lim 30 45", "#lim"), ("vl 26", "#vlim"), ("vds 5", "#vds"), ("odg 8", "#ocp")):
        print(b.ack(c, tok).strip(), flush=True)
    b.cmd("sq", 0.4)
    b.stream_on()
    print(b.ack(f"hrun {AMPS} {TARGET_RPM} 8", "#hall speed").strip(), flush=True)
    time.sleep(12)
    for bv in BRAKE_RAMP:
        want_brake(bv)
        if not brake_acked(120, tick):
            abort = "brake ack timeout"; break
        if state["fault"]: abort = "FAULT: " + state["fault"]; break
        t0 = time.time(); clampA = 0.0
        while time.time() - t0 < HOLD_S:
            time.sleep(0.25); tick()
            if state["fault"]: abort = "FAULT: " + state["fault"]; break
            try:
                vmax = float(scope.q(":MEAS:ITEM? VMAX,CHAN3")); vmin = float(scope.q(":MEAS:ITEM? VMIN,CHAN3"))
                if abs(vmax) < 9e37: clampA = max(clampA, vmax * 100.0)
                if abs(vmin) < 9e37: clampA = max(clampA, -vmin * 100.0)
            except (ValueError, OSError): pass
            if clampA > CLAMP_ABORT_A:
                abort = f"CLAMP: {clampA:.1f} A real on phase C"; break
        v, i = psu.meas()[:2]; p = v * i
        tq = scope.torque_nm(n=3, settle=0.15)
        ms = sstat.search(b.cmd("sq", 0.5) or "")
        row = state["row"]
        rec = {"t": time.time(), "brake_v": bv, "bus_v": v, "bus_i": i, "bus_w": p,
               "torque_nm": tq, "clamp_peak_a": clampA,
               "iq": float(row.group(4)) if row else None,
               "rpm": float(row.group(11)) if row else None,
               "tmax": max(float(row.group(k)) for k in (13, 14, 15, 16)) if row else None,
               "late_pct": float(ms.group(4)) if ms else None,
               "cmax_us": int(ms.group(5)) if ms else None}
        log.append(rec); peak_w = max(peak_w, p)
        print(f"  brake {bv:.1f}V: {p:6.1f} W ({v:.2f}V {i:.2f}A) tq={tq if tq is None else round(tq,1)}N·m "
              f"iq={rec['iq']} rpm={rec['rpm']} tmax={rec['tmax']} clamp={clampA:.1f}A "
              f"late={rec['late_pct']}% cmax={rec['cmax_us']}us", flush=True)
        if abort: print("!! " + abort, flush=True); break
        if v < 45.0: abort = f"VM sag {v:.2f} V — Iset CC"; print("!! " + abort, flush=True); break
        if rec["tmax"] and rec["tmax"] > 75: abort = "temp > 75C"; print("!! " + abort, flush=True); break
        if p >= 1010: print("** 1 kW reached **", flush=True); break
        if rec["iq"] and abs(rec["iq"]) > AMPS * 0.97 and rec["rpm"] < TARGET_RPM * 0.7:
            abort = "iq ceiling + speed collapse"; print("!! " + abort, flush=True); break
finally:
    try: b.stop()
    except Exception: pass
    want_brake(0.0)   # operator releases; motor already unloaded+coasting
    try: psu.iset(8.0)
    except Exception: pass
    save(DATA, "ladder_1kw_f_500rpm", {"log": log, "peak_w": peak_w, "abort": abort,
                              "target_rpm": TARGET_RPM, "amps": AMPS, "trip_a": 45})
    print(f"\nPEAK BUS POWER: {peak_w:.1f} W  abort={abort}", flush=True)
    b.close(); psu.close(); scope.close()
