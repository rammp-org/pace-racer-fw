"""Fig 1b/1c: J estimate, then velocity + position autotune progressions."""
import sys, time, math
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from bench_lib import Board, Brake, save, KT

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig1"

b = Board()
b.arm_limits()
brake = Brake(); brake.off(); brake.close()

ofs, cons = b.ecal()
print(f"ecal: ofs={ofs:.2f} consistency={cons:.3f}")

# ---- J estimate: iq step from rest, unloaded --------------------------------
b.ack("cap 20 1500 0", "#cap armed")           # 1 kHz x 1.5 s
b.ack("eiq 3", "#enc iq")
time.sleep(1.6)
b.stop()
meta, rows = b.capdump()
# rows: pos_deg, rpm, iq. Fit rpm slope on the linear region after breakaway
# (stiction delays motion; the 50 ms rpm filter needs settling room too).
ts = [i * meta["dt"] for i in range(len(rows))]
onset = next((t for t, r in zip(ts, rows) if abs(r[1]) > 2), 0.0)
lo, hi = onset + 0.15, onset + 0.55
seg = [(t, r[1]) for t, r in zip(ts, rows) if lo <= t <= hi]
iqs = [r[2] for t, r in zip(ts, rows) if lo <= t <= hi]
n = len(seg)
mx = sum(p[0] for p in seg)/n; my = sum(p[1] for p in seg)/n
slope = sum((p[0]-mx)*(p[1]-my) for p in seg)/sum((p[0]-mx)**2 for p in seg)  # rpm/s
iq_mean = sum(iqs)/len(iqs)
alpha = slope * 2*math.pi/60.0                  # rad/s^2
J = KT * iq_mean / alpha                        # kg m^2
print(f"J est: iq={iq_mean:.2f}A slope={slope:.0f} rpm/s -> alpha={alpha:.1f} rad/s2 -> J={J*1000:.2f} mkg·m2")
save(DATA, "jest", {"dt": meta["dt"], "rows": rows, "iq": iq_mean, "slope_rpm_s": slope, "J": J})

# ---- velocity autotune: pole placement, 3 progressive runs ------------------
# plant: rpm_dot = K*iq with K = KT/J * 60/2pi  [rpm/s per A]
# PI (fw units: A per rpm, A per rpm*s): s^2 + K*kp*s + K*ki = 0
K = KT / J * 60/(2*math.pi)
ZETA = 1.0
runs = []
for tag, wn in (("A-soft", 2.0), ("B-computed", 5.0), ("C-tight", 9.0)):
    kp = 2*ZETA*wn/K
    ki = wn*wn/K
    b.ack(f"sg {kp:.5f} {ki:.5f}", "#speed gains")
    b.ack("cap 20 2500 0", "#cap armed")        # 1 kHz x 2.5 s
    b.ack("erun 8 50 0.15", "#enc speed")       # near-step to 50 rpm
    time.sleep(2.6)
    b.stop()
    meta, rows = b.capdump()
    rpm = [r[1] for r in rows]
    peak = max(rpm)
    tail = rpm[-400:]
    ss = sum(tail)/len(tail)
    ovs = (peak - 50) / 50 * 100
    t90 = next((i*meta["dt"] for i, x in enumerate(rpm) if x >= 45), None)
    runs.append({"tag": tag, "wn": wn, "kp": kp, "ki": ki, "dt": meta["dt"],
                 "rpm": rpm, "iq": [r[2] for r in rows],
                 "peak": peak, "ss": ss, "overshoot_pct": ovs, "t90": t90})
    print(f"  vel {tag}: wn={wn} kp={kp:.4f} ki={ki:.4f} -> peak={peak:.1f} ss={ss:.1f} "
          f"ovs={ovs:.0f}% t90={t90}")
    time.sleep(1.0)
save(DATA, "vel_tune", {"K": K, "J": J, "zeta": ZETA, "runs": runs})

# keep the computed gains for the rest of the campaign
kp = 2*ZETA*5.0/K; ki = 25.0/K
b.ack(f"sg {kp:.5f} {ki:.5f}", "#speed gains")

# ---- position autotune: P(D) cascade, 3 progressive runs --------------------
pruns = []
for tag, pkp, pkd in (("A-soft", 0.6, 0.0), ("B-medium", 2.0, 0.0), ("C-derivative", 4.0, 0.05)):
    b.ack(f"pg {pkp} {pkd}", "#pos gains")
    start = b.enc_acc_rotor_deg()
    target = start + 90.0
    b.ack("cap 20 2500 0", "#cap armed")
    b.ack(f"epos {target:.1f} 60 6", "#enc pos")
    time.sleep(2.6)
    b.stop()
    meta, rows = b.capdump()
    pos = [r[0] - start for r in rows]
    peak = max(pos)
    tail = pos[-400:]
    ss = sum(tail)/len(tail)
    ovs = (peak - 90) / 90 * 100
    t90 = next((i*meta["dt"] for i, x in enumerate(pos) if x >= 81), None)
    pruns.append({"tag": tag, "kp": pkp, "kd": pkd, "dt": meta["dt"],
                  "pos": pos, "rpm": [r[1] for r in rows],
                  "peak": peak, "ss": ss, "overshoot_pct": ovs, "t90": t90})
    print(f"  pos {tag}: kp={pkp} kd={pkd} -> peak={peak:.1f} ss={ss:.1f} ovs={ovs:.0f}% t90={t90}")
    time.sleep(1.0)
save(DATA, "pos_tune", {"runs": pruns})
b.close()
print("fig1 complete")
