"""Fig 1a: R/L identification — voltage steps, fit R from slope, L from tau."""
import sys, time, math
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from bench_lib import Board, Brake, save

DATA = "/Users/alxchunlin/code-stuff/atdev/pace-racer/pace-racer-fw/data-reports/logs/figs-20260819/fig1"

b = Board()
b.arm_limits()
brake = Brake(); brake.off(); brake.close()
print("limits armed, brake off")

levels = [0.6, 1.0, 1.5, 2.0]
results = []
for v in levels:
    b.ack(f"rl {v} 15", "#rl step")
    time.sleep(0.5)
    meta, rows = b.capdump()
    # rows: ialpha, ibeta, valpha at 50 us. id at th=0 == ialpha.
    ia = [r[0] for r in rows]
    va = [r[2] for r in rows]
    n = len(ia)
    # steady state = mean of last 30% where valpha is at the step level
    tailN = max(10, int(n * 0.3))
    iss = sum(ia[-tailN:]) / tailN
    vss = sum(va[-tailN:]) / tailN
    # tau: first index crossing 63.2% of iss (step starts at row 0 by design)
    i63 = 0.632 * iss
    tau_idx = next((k for k, x in enumerate(ia) if x >= i63), None)
    results.append({"v": v, "vss": vss, "iss": iss, "tau_us": (tau_idx * 50 if tau_idx else None),
                    "trace_ia": ia[:200], "trace_va": va[:200]})
    print(f"  rl {v:.1f}V: n={n} Iss={iss:.3f}A tau~{tau_idx*50 if tau_idx else '?'}us")
    time.sleep(0.5)

# R from slope of vss vs iss (least squares through the points; intercept = deadtime/Vf error)
xs = [r["iss"] for r in results]; ys = [r["vss"] for r in results]
n = len(xs); mx = sum(xs)/n; my = sum(ys)/n
R = sum((x-mx)*(y-my) for x, y in zip(xs, ys)) / sum((x-mx)**2 for x in xs)
V0 = my - R*mx
taus = [r["tau_us"] for r in results if r["tau_us"]]
tau_us = sum(taus)/len(taus)
L_uH = R * tau_us  # L = R*tau; tau in us -> L in uH
print(f"\nR = {R:.4f} ohm (slope), V0 offset = {V0:.3f} V (deadtime+Vf)")
print(f"tau = {tau_us:.0f} us mean -> L = {L_uH:.0f} uH")
print(f"(datasheet-era values: R=0.323 ohm, L=336 uH)")
save(DATA, "rl", {"levels": results, "R": R, "V0": V0, "tau_us": tau_us, "L_uH": L_uH})
b.close()
