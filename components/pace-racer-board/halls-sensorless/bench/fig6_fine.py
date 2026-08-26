"""Fig 6 fine ramp: ride the 150 rpm max-power edge with 1 Hz logging."""
import sys, time, re, json, os
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
import serial as pyserial
from bench_lib import Board, Brake, Scope, PSU

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig6"
TEMP_ABORT = 70.0

psu = pyserial.Serial(PSU, 115200, timeout=1.0)
def bus():
    psu.reset_input_buffer(); psu.write(b"MEAS:VOLT?\n"); time.sleep(0.12)
    v = float(psu.read(100).decode().strip() or 0)
    psu.write(b"MEAS:CURR?\n"); time.sleep(0.12)
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
    # halls stay calibrated from the previous run (no reboot since)
    hs = b.cmd("hs", 0.6)
    if "cal=1" not in hs:
        print("hcal...", flush=True)
        b.ack("ho 9999 45 1", "#handoff")
        b.ack("hcal 8", "#")
        b.ack("run 4 30 5", "#"); time.sleep(9); b.stop()
        b.ack("run 4 -30 5", "#"); time.sleep(9); b.stop()
        hs = b.cmd("hs", 0.6)
        if "cal=1" not in hs:
            print("hcal failed:", hs[:150]); sys.exit(1)
    b.stream_on()
    b.ack("hrun 30 150 5", "#hall speed")
    time.sleep(7)
    bv = 2.0
    brake.set(bv, settle=1.0, ilim=3.0)
    peak_w = 0.0; stalls = 0; stop_reason = "plan complete"
    t_end = time.time() + 100
    buf = ""
    last_adj = time.time()
    while time.time() < t_end:
        time.sleep(1.0)
        buf += b.s.read(32000).decode(errors="replace")
        lines = buf.split("\n"); buf = lines[-1]
        row = None; faults = []
        for ln in lines[:-1]:
            if ln.startswith("!"): faults.append(ln)
            m = srow.match(ln.strip())
            if m: row = m
        if faults:
            stop_reason = "FAULT: " + faults[0][:70]; break
        if not row:
            continue
        v, i = bus()
        rec = {"t": time.time(), "brake_v": bv, "bus_w": v*i, "bus_v": v, "bus_i": i,
               "iq": float(row.group(4)), "rpm": float(row.group(11)),
               "tmax": max(float(row.group(k)) for k in (13, 14, 15, 16))}
        log.append(rec)
        peak_w = max(peak_w, rec["bus_w"])
        print(f"brake {bv:4.2f}V: {rec['bus_w']:5.0f} W  iq={rec['iq']:5.1f}A  rpm={rec['rpm']:3.0f}  "
              f"tmax={rec['tmax']:.0f}C", flush=True)
        if rec["tmax"] > TEMP_ABORT:
            stop_reason = f"temp {rec['tmax']:.0f}C"; break
        if rec["rpm"] < 110:          # sagging toward stall — back off
            stalls += 1
            if stalls > 2:
                stop_reason = "repeated stall"; break
            bv = max(1.5, bv - 0.3)
            brake.set(bv, settle=0, ilim=3.0)
            print(f"  sag — backing off to {bv:.2f} V", flush=True)
            last_adj = time.time()
        elif rec["rpm"] > 140 and rec["iq"] < 29 and time.time() - last_adj > 3.0:
            bv = min(6.0, bv + 0.15)
            brake.set(bv, settle=0, ilim=3.0)
            last_adj = time.time()
    print(f"\nend: {stop_reason}   PEAK BUS POWER = {peak_w:.0f} W", flush=True)
finally:
    try: brake.set(0.0, settle=0); brake.off()
    except Exception: pass
    try:
        b.cmd("stop", 0.5); time.sleep(1.5); b.stream_off()
    except OSError: pass
    try: psu.write(b"CURR 8\n"); psu.close()
    except OSError: pass
    os.makedirs(DATA, exist_ok=True)
    json.dump({"log": log}, open(os.path.join(DATA, "power150_fine.json"), "w"))
    print(f"saved {len(log)} rows", flush=True)
    for o in (sc, brake, b):
        try: o.close()
        except Exception: pass
