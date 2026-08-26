"""Tight RTPS delivered-rate sweep.

One subscriber for the whole run (a fresh participant per point leaves stale
ReaderProxies on the board, which then duplicates every sample). Board rebooted
first for the same reason. Windows are bounded by absolute wall-clock times
recorded around each dwell, and rtpstat is read at both edges so pub_ok covers
exactly the same window as the host-side count -- that separates "the board
never published that fast" from "packets were lost on the wire".
"""
import os
# --- repo-relative paths -------------------------------------------------
# Run from anywhere: everything resolves off this file's location. The two
# machine-specific bits are overridable by environment variable.
HERE  = os.path.dirname(os.path.abspath(__file__))
TESTS = os.path.abspath(os.path.join(HERE, "..", "..", ".."))   # .../halls-sensorless/tests
DATA  = os.path.abspath(os.path.join(HERE, "..", "data"))
TELEM_SUB = os.environ.get("TELEM_SUB", os.path.join(TESTS, "host_sub", "build", "telem_sub"))
BENCH_LIB = os.environ.get("BENCH_LIB", "")   # dyno bench_lib.py dir; only the spin/bringup scripts need it

import sys, time, subprocess, re, json
sys.path.insert(0, TESTS)
import prlib, serial

SUB = TELEM_SUB
LOG = os.path.join(DATA, "delivered_sweep_samples.log")
RATES = [10, 25, 50, 100, 200, 300, 500, 1000, 2000]
SETTLE, DWELL = 2.5, 12.0

def reboot():
    p = serial.Serial(prlib.PORT, 115200, timeout=0.3)
    p.dtr = False; p.rts = True; time.sleep(0.2); p.rts = False
    t0 = time.time()
    while time.time() - t0 < 6.0: p.read(16384)
    p.close()

print("rebooting board (clears stale reader proxies)...")
reboot()
prlib.console("eth", 5.0); prlib.console("eth", 2.0)
print("rtps:", prlib.console_first("rtps 1 0", "#rtps", 6.0))

total = (SETTLE + DWELL) * len(RATES) + 25
f = open(LOG, "w")
proc = subprocess.Popen([SUB, "192.168.50.1", str(total), "-1"],
                        stdout=f, stderr=subprocess.DEVNULL)
print("waiting for discovery...")
time.sleep(6.0)

marks = []
# prlib.console() opens the port, sleeps 0.3s, then writes -- so the firmware
# sees each command ~CMD_OFF after the call starts. Bracketing BOTH window edges
# with the same offset makes the pub_ok window and the host-side sample window
# identical by construction, so the offset cancels instead of showing up as
# phantom loss (or phantom gain, if you correct it in the wrong direction).
CMD_OFF = 0.35
for hz in RATES:
    prlib.console("rtpshz %d" % hz, 0.6)
    time.sleep(SETTLE)
    t_reset = time.time()
    prlib.console("rtpstat", 1.0)              # resets counters on read
    time.sleep(DWELL)
    t_read = time.time()
    st = prlib.console_first("rtpstat", "#rtpstat", 1.5) or ""
    s2 = prlib.console_first("s", "#stats", 1.5) or ""
    marks.append(dict(hz=hz, t0=t_reset + CMD_OFF, t1=t_read + CMD_OFF,
                      rtpstat=st, stats=s2))
    print("  %5d Hz done" % hz)

proc.wait(); f.close()

samples = []
for line in open(LOG):
    m = re.match(r"S ([\d.]+) ([\d.]+)", line)
    if m: samples.append(float(m.group(2)))
print("\nlogged %d samples" % len(samples))

print("\n%7s %11s %11s %8s %10s %8s %9s" %
      ("req Hz", "published", "delivered", "loss%", "pub_us", "late%", "coalesce"))
print("-" * 72)
rows = []
for mk in marks:
    dur = mk["t1"] - mk["t0"]
    got = sum(1 for t in samples if mk["t0"] <= t < mk["t1"])
    d = dict(p.split("=", 1) for p in mk["rtpstat"].replace("#rtpstat ", "").split() if "=" in p)
    pub_ok = int(d.get("pub_ok", 0))
    published = pub_ok / dur
    delivered = got / dur
    loss = (1 - delivered / published) * 100 if published > 0 else 0
    lm = re.search(r"late=\d+ \(([\d.]+)%\)", mk["stats"])
    cm = re.search(r"coalesce=([\d.]+)", mk["stats"])
    row = dict(hz=mk["hz"], published=published, delivered=delivered, loss=loss,
               pub_us=d.get("pub_us", "?"), overrun=int(d.get("overrun", 0)),
               late=lm.group(1) if lm else "?", coalesce=cm.group(1) if cm else "?",
               samples=got, dur=dur)
    rows.append(row)
    print("%7d %11.1f %11.1f %8.1f %10s %8s %9s" %
          (row["hz"], published, delivered, loss, row["pub_us"], row["late"], row["coalesce"]))
json.dump(rows, open(os.path.join(DATA, "delivered_sweep.json"), "w"), indent=1)
print("\npeak delivered: %.1f Hz" % max(r["delivered"] for r in rows))
