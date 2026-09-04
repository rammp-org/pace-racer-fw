"""Fig 6: max power at 150 rpm — hall commutation, brake ramp until
stall/heat/fault. lim 30 35, vds 5. User is on the kill switch."""
import sys, time, re, json, os, statistics
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
import serial as pyserial
from bench_lib import Board, Brake, Scope, PSU, save

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig6"
RAMP_V = [1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0]
HOLD_S = 6.0
TEMP_ABORT = 70.0
STALL_RPM = 90.0

psu = pyserial.Serial(PSU, 115200, timeout=1.0)
def bus():
    psu.reset_input_buffer(); psu.write(b"MEAS:VOLT?\n"); time.sleep(0.15)
    v = float(psu.read(100).decode().strip() or 0)
    psu.write(b"MEAS:CURR?\n"); time.sleep(0.15)
    i = float(psu.read(100).decode().strip() or 0)
    return v, i
psu.write(b"CURR 15\n"); time.sleep(0.3)

sc = Scope(); b = Board(); brake = Brake(); brake.off()
srow = re.compile(r"^(\d+\.\d+), ([A-Z]), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), (-?[\d.]+), "
                  r"(-?\d+), (-?\d+), (-?\d+), (-?\d+), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+), ([\d.]+)")
log = []
try:
    b.ack("e 1", "#edge")
    for cmd, tok in (("lim 30 35", "#lim"), ("vl 20", "#vl"), ("vds 5", "#vds"), ("odg 8", "#ocp")):
        b.ack(cmd, tok)
    print("armed: lim 30 35, vl 20, vds 5, odg 8, BK Iset 15", flush=True)

    print("== hall cal (two gentle spins) ==", flush=True)
    b.ack("ho 9999 45 1", "#handoff")
    b.ack("hcal 8", "#")
    b.ack("run 4 30 5", "#"); time.sleep(9); b.stop()
    b.ack("run 4 -30 5", "#"); time.sleep(9); b.stop()
    hs = b.cmd("hs", 0.6)
    if "cal=1" not in hs:
        print("!! hcal incomplete:", hs[:200], flush=True); sys.exit(1)
    print("halls calibrated", flush=True)

    print("== spin up: hrun 30 150 5 ==", flush=True)
    b.stream_on()
    b.ack("hrun 30 150 5", "#hall speed")
    time.sleep(7)
    peak_w = 0.0
    stop_reason = "ramp complete"
    for bv in RAMP_V:
        brake.set(bv, settle=1.0, ilim=3.0)
        mv, mi = brake.meas()
        t0 = time.time(); buf = ""; row = None; faults = []
        while time.time() - t0 < HOLD_S:
            time.sleep(0.3)
            buf += b.s.read(32000).decode(errors="replace")
            lines = buf.split("\n"); buf = lines[-1]
            for ln in lines[:-1]:
                if ln.startswith("!"): faults.append(ln)
                m = srow.match(ln.strip())
                if m: row = m
        v, i = bus()
        tq = sc.torque_nm(n=3, settle=0.1)
        if row:
            rec = {"brake_v_cmd": bv, "brake_v": mv, "brake_i": mi,
                   "bus_v": v, "bus_i": i, "bus_w": v*i,
                   "iq": float(row.group(4)), "rpm": float(row.group(11)),
                   "torque_nm": tq,
                   "tmax": max(float(row.group(k)) for k in (13, 14, 15, 16))}
            log.append(rec)
            peak_w = max(peak_w, rec["bus_w"])
            print(f"brake {bv:4.1f}V({mi:.2f}A): {rec['bus_w']:5.0f} W bus  iq={rec['iq']:5.1f}A  "
                  f"rpm={rec['rpm']:3.0f}  T={tq if tq is None else round(tq,1)} N·m  "
                  f"tmax={rec['tmax']:.0f}C  faults={len(faults)}", flush=True)
            if faults:
                stop_reason = "FAULT: " + faults[0][:70]; break
            if rec["tmax"] > TEMP_ABORT:
                stop_reason = f"temp {rec['tmax']:.0f}C"; break
            if rec["rpm"] < STALL_RPM:
                stop_reason = f"stall approach (rpm {rec['rpm']:.0f})"; break
        else:
            stop_reason = "no telemetry"; break
    print(f"\nramp ended: {stop_reason}   PEAK BUS POWER = {peak_w:.0f} W", flush=True)
finally:
    try: brake.set(0.0, settle=0); brake.off()
    except Exception: pass
    try:
        b.cmd("stop", 0.5); time.sleep(1.5)
        b.stream_off()
    except OSError: pass
    try: psu.write(b"CURR 8\n"); psu.close()
    except OSError: pass
    os.makedirs(DATA, exist_ok=True)
    json.dump({"log": log}, open(os.path.join(DATA, "power150.json"), "w"))
    print(f"saved {len(log)} points", flush=True)
    for o in (sc, brake, b):
        try: o.close()
        except Exception: pass
