"""Load-step regulation at constant speed: hold 400 rpm, servo the brake to
measured-torque setpoints (N.m, from the CH4 sensor), log a continuous time
series. Brake volts are an internal actuator detail - the data is all N.m.
Brake writes go through the operator handshake (MCP owns the DP2031)."""
import sys, time, re, json, pathlib
sys.path.insert(0, "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/components/pace-racer-board/halls-sensorless/bench")
from bench_lib import Board, Scope, Psu, save

HERE = pathlib.Path(__file__).resolve().parent
REQ, ACK = HERE / "ladder_req.json", HERE / "ladder_ack.json"
DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/run-20260827-torque-hold"
pathlib.Path(DATA).mkdir(parents=True, exist_ok=True)
TARGET_RPM = 400
SETPOINTS = [2.0, 4.0, 6.0, 8.0, 10.0, 8.0, 6.0, 4.0, 2.0]   # N.m
TOL, HOLD_S, SLOPE0 = 0.4, 8.0, 3.5   # N.m tolerance, dwell, N.m per brake volt

srow = re.compile(r"^(\d+\.\d+), ([A-Z]), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?\d+), (-?\d+), (-?\d+), (-?\d+), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+)")
seq = 0

def want_brake(v):
    global seq
    seq += 1
    REQ.write_text(json.dumps({"brake_v": round(v, 3), "seq": seq}))
    print(f"[req] brake -> {v:.2f} V (seq {seq})", flush=True)

def brake_acked(timeout, tick):
    t0 = time.time()
    while time.time() - t0 < timeout:
        tick()
        try:
            if json.loads(ACK.read_text()).get("seq") == seq:
                return True
        except (OSError, ValueError):
            pass
        time.sleep(0.4)
    return False

psu = Psu(); scope = Scope(); b = Board()
series, marks, abort = [], [], None
state = {"row": None, "fault": None, "buf": ""}
t_start = time.time()

def tick():
    state["buf"] += b.s.read(32000).decode(errors="replace")
    lines = state["buf"].split("\n"); state["buf"] = lines[-1]
    for ln in lines[:-1]:
        if ln.startswith("!") and state["fault"] is None:
            state["fault"] = ln.strip()
        m = srow.match(ln.strip())
        if m: state["row"] = m

def sample(setp, phase):
    """One logged sample: torque (2 quick reads), stream row, bus."""
    tick()
    tq = scope.torque_nm(n=2, settle=0.12)
    v, i = psu.meas()[:2]
    row = state["row"]
    rec = {"t": round(time.time() - t_start, 2), "setpoint_nm": setp, "phase": phase,
           "torque_nm": tq, "bus_w": round(v * i, 1), "bus_v": v,
           "iq": float(row.group(4)) if row else None,
           "rpm": float(row.group(11)) if row else None,
           "tmax": max(float(row.group(k)) for k in (13, 14, 15, 16)) if row else None}
    series.append(rec)
    return rec

try:
    psu.iset(15.0)
    for c, tok in (("lim 30 45", "#lim"), ("vl 26", "#vlim")):
        print(b.ack(c, tok).strip(), flush=True)
    b.stream_on()
    b.cmd("s", 0.3)
    print(b.ack(f"hrun 30 {TARGET_RPM} 8", "#hall speed").strip(), flush=True)
    time.sleep(11)
    bv, slope = 0.0, SLOPE0
    for setp in SETPOINTS:
        marks.append({"t": round(time.time() - t_start, 2), "setpoint_nm": setp})
        # servo the brake onto the setpoint
        for attempt in range(6):
            rec = sample(setp, "servo")
            tq = rec["torque_nm"]
            if state["fault"]: abort = "FAULT: " + state["fault"]; break
            if rec["rpm"] is not None and rec["rpm"] < 250: abort = f"speed collapse {rec['rpm']}"; break
            if tq is None: continue
            err = setp - tq
            if abs(err) <= TOL: break
            step = max(-0.5, min(0.5, err / slope))
            prev_bv, prev_tq = bv, tq
            bv = max(0.0, min(4.5, bv + step))
            want_brake(bv)
            if not brake_acked(120, tick): abort = "brake ack timeout"; break
            time.sleep(1.2)
            r2 = sample(setp, "servo")
            if r2["torque_nm"] is not None and abs(bv - prev_bv) > 0.05:
                s_new = (r2["torque_nm"] - prev_tq) / (bv - prev_bv)
                if 1.0 < s_new < 12.0: slope = 0.5 * slope + 0.5 * s_new
        if abort: break
        # dwell: log the hold
        t0 = time.time()
        while time.time() - t0 < HOLD_S:
            rec = sample(setp, "hold")
            if state["fault"]: abort = "FAULT: " + state["fault"]; break
            if rec["tmax"] and rec["tmax"] > 75: abort = "temp > 75C"; break
            time.sleep(0.4)
        if abort: break
        got = [r["torque_nm"] for r in series if r["phase"] == "hold" and r["setpoint_nm"] == setp and r["torque_nm"]]
        print(f"  setpoint {setp:4.1f} N.m: held {sum(got)/len(got):5.2f} N.m mean, "
              f"bus {series[-1]['bus_w']:6.1f} W, rpm {series[-1]['rpm']}", flush=True)
finally:
    # gentle finish: unload the BRAKE first, then stop the motor (the 0x0622
    # after the 1 kW run came from stopping into a still-energized brake)
    want_brake(0.0)
    brake_acked(60, tick)
    time.sleep(1.0)
    try: b.stop()
    except Exception: pass
    try: psu.iset(8.0)
    except Exception: pass
    save(DATA, "torque_hold", {"series": series, "marks": marks, "abort": abort,
                               "target_rpm": TARGET_RPM, "setpoints": SETPOINTS, "tol": TOL})
    print(f"\nDONE abort={abort} samples={len(series)}", flush=True)
    b.close(); psu.close(); scope.close()
