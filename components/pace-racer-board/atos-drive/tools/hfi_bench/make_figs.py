"""Figures for the HFI draft PR. Data: /tmp/hfi (bench captures, 2026-10-23/24)."""
import json
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, "/Users/alxchunlin/.claude/skills/alex-style/matplotlib")
import alex_style as ax  # noqa: E402

OUT = sys.argv[1]
P = ax.use("light")
S = P["series"]
D = "/tmp/hfi"
wrap = lambda d: (d + 180) % 360 - 180  # noqa: E731
wrap180 = lambda d: (d + 90) % 180 - 90  # noqa: E731


# 1. Fake saliency from the measurement chain (simulation, motor with zero saliency)
def sim(phi_deg, div, median, stale):
    k = np.arange(8000)
    ph = 2 * np.pi * (k % div) / div
    i = np.exp(1j * (ph + np.radians(phi_deg)))
    ia, ib = i.real, (i * np.exp(-2j * np.pi / 3)).real
    if median:
        med3 = lambda x: np.r_[x[:2], np.median(np.c_[x[:-2], x[1:-1], x[2:]], axis=1)]  # noqa: E731
        ic = -ia - ib
        ia, ib, ic = med3(ia), med3(ib), med3(ic)
    if stale:  # one phase converted per period, the other held
        held = np.zeros((2, len(k)))
        last = [0.0, 0.0]
        for t in k:
            f = t % 2
            last[f] = (ia, ib)[f][t]
            held[:, t] = last
        ia, ib = held
    z = ia + 1j * (ia + 2 * ib) / np.sqrt(3)
    return abs(np.mean(z * np.exp(1j * ph))) / abs(np.mean(z * np.exp(-1j * ph)))


phis = np.arange(0, 91, 3)
fig, a = plt.subplots(figsize=(7.2, 3.6))
a.plot(phis, [sim(p, 4, True, False) for p in phis], color=S[1], label="median-of-3 filter, fs/4")
a.plot(phis, [sim(p, 4, False, True) for p in phis], color=S[3], label="stale phase sample, fs/4")
a.plot(phis, [sim(p, 8, False, True) for p in phis], color=S[0], label="fresh-phase demod, fs/8")
ax.ref_line(a, 0.106, "real rotor term, 10.6 %", status="good")
a.set_xlim(0, 90)
a.set_ylim(0, 1.05)
a.set_xlabel("Injection phase vs phase-A axis, deg")
a.set_ylabel("Apparent saliency |N|/|P|, ratio")
ax.label_lines(a)
ax.title(a, "What the sampler was measuring", "Simulated motor with ZERO saliency, demodulated through each sampling path")
fig.tight_layout()
ax.save(fig, f"{OUT}/fig1_measurement_chain.png", dpi=160)

# 2. HFI angle vs encoder at standstill (12 V, Vh 0.17 V, fs/8), 24 alignments
r = json.load(open(f"{D}/scan12_017.json"))
ns_re, ns_im, off = [float(x) for x in open(f"{D}/cal12_017.txt").read().split()]
th = np.array([x["e0"] for x in r])
N = np.array([complex(*x["N"]) for x in r])
Pp = np.array([complex(*x["P"]) for x in r])
est2 = np.degrees(np.angle((N - complex(ns_re, ns_im)) * Pp / abs(Pp))) + 180 - off
est = (est2 / 2) % 180
err = wrap180(est - th % 180)
fig, a = plt.subplots(figsize=(7.2, 3.6))
a.plot([0, 360], [0, 360], color=P["muted"], lw=1, ls=(0, (2, 4)), label="_ideal")
a.scatter(th, np.where(abs((est + 180) - th) < abs(est - th), est + 180, est), s=22, color=S[0], zorder=3)
a.set_xlim(0, 360)
a.set_ylim(0, 360)
a.set_xticks(range(0, 361, 60))
a.set_yticks(range(0, 361, 60))
a.set_xlabel("Rotor angle from encoder, deg elec")
a.set_ylabel("HFI angle estimate, deg elec")
ax.title(a, "HFI angle at standstill", f"24 releases into the 6 cogging detents; error rms {np.sqrt(np.mean(err**2)):.1f} deg (mod 180, polarity from encoder)")
fig.tight_layout()
ax.save(fig, f"{OUT}/fig2_standstill_angle.png", dpi=160)

# 3. The console stall on the scope (before the fix)
d = np.load(f"{D}/dump1.npz")
t, cur = d["t"], d["a"]
w = int(round(50 / (t[1] - t[0])))
n = len(cur) // w
tm = t[: n * w].reshape(n, w).mean(1)
cm = cur[: n * w].reshape(n, w).mean(1)
fig, a = plt.subplots(figsize=(7.2, 3.6))
step = max(1, len(t) // 20000)
a.plot(t[::step], cur[::step], color=S[0], lw=0.6, label="_raw")
a.plot(tm, cm, color=S[1], lw=2.0, label="mean per PWM period", marker="s", ms=4)
ax.ref_line(a, 10, "scope trigger 10 A", status="warning")
a.set_ylim(-1, 12)
a.set_xlabel("Time from trigger, us")
a.set_ylabel("Phase A current, A")
ax.label_lines(a)
ax.title(a, "Console dump froze the duties", "0.17 V injection held as DC across 3 uH: +2.5 A per period until the dump released the ISR (before the fix)")
fig.tight_layout()
ax.save(fig, f"{OUT}/fig3_console_stall_scope.png", dpi=160)

fig, a = plt.subplots(figsize=(7.2, 2.4))
ax.bars(a, ["Before fix, during capdump", "After fix, during capdump", "Idle, no console output"], [227, 156, 150],
        highlight="Before fix, during capdump", unit=" us", xlabel="Longest time the bridge held one duty set, us", ylabel="Condition")
ax.title(a, "Duty freeze during console output", "Max FOC wake-to-wake gap; red = the freeze that caused three DRV trips")
fig.tight_layout()
ax.save(fig, f"{OUT}/fig3b_console_stall_fix.png", dpi=160)


# 4. HFI-driven runs: reverse holds, forward trips at the stick spot
def load9(f):
    a9 = np.load(f)
    a9 = a9[: len(a9) // 2 * 2].reshape(-1, 6)
    ch = np.flatnonzero(np.abs(np.diff(a9[:, 0])) + np.abs(np.diff(a9[:, 2])) > 1e-6)
    a9 = a9[: ch[-1] + 2] if len(ch) else a9
    return np.arange(len(a9)) * 0.008, a9


SD = f"{D}/reports/smooth-hfi-drive-data"
fig, a = plt.subplots(figsize=(7.2, 3.6))
for f, lab, c, z in (("ACC30_3", "forward +30 rpm, guard trip", S[1], 2), ("ACCm30_1", "reverse -30 rpm, held 10 s", S[0], 3)):
    tt, a9 = load9(f"{SD}/{f}.npy")
    e = wrap(a9[:, 2] - a9[:, 0])
    big = np.flatnonzero(np.abs(e) > 90)
    if len(big):  # stop at the loss: after the trip the frame is frozen and wraps
        tt, e = tt[: big[0] + 1], e[: big[0] + 1]
    a.plot(tt, e, color=c, lw=1.2, label=lab, zorder=z)
ax.ref_line(a, 60, "encoder guard +60 deg", status="critical")
ax.ref_line(a, -60, "encoder guard -60 deg", status="critical")
a.set_ylim(-180, 180)
a.set_yticks(range(-180, 181, 60))
a.set_xlabel("Time since drive start, s")
a.set_ylabel("HFI angle minus encoder, deg elec")
ax.label_lines(a)
ax.title(a, "Closed loop on the HFI angle", "xrun 3 A ceiling, 12 V bus, Vh 0.17 V; forward losses start at the stick spot near 180 deg elec")
fig.tight_layout()
ax.save(fig, f"{OUT}/fig4_hfi_drive.png", dpi=160)

# 5. Polarity test accuracy per detent (<= 3 A d pulses)
det = ["350 deg", "51 deg", "111 deg", "167 deg", "232 deg", "300 deg"]
ok = [8 / 16, 6 / 6, 18 / 18, 15 / 16, 8 / 8, 11 / 14]
fig, a = plt.subplots(figsize=(7.2, 2.8))
ax.bars(a, det, [round(100 * v) for v in ok], highlight="350 deg", unit=" %", xlabel="Polarity decisions correct, %", ylabel="Cogging detent, enc deg elec")
a.set_xlim(0, 110)
ax.title(a, "Encoder-free polarity (hpol)", "66 of 78 correct (85 %); red = the detent where a stator-fixed term cancels the 2 % magnet signal")
fig.tight_layout()
ax.save(fig, f"{OUT}/fig5_polarity.png", dpi=160)

# 6. Angle error summary
lab = ["Standstill, no current", "Encoder FOC 30 rpm, HFI observing", "HFI drive, reverse 30 rpm", "HFI drive, forward 30 rpm (to trip)"]
val = [4.8, 15.0, 11.8, 19.3]
fig, a = plt.subplots(figsize=(7.2, 2.6))
ax.bars(a, lab, val, highlight="HFI drive, forward 30 rpm (to trip)", unit=" deg", xlabel="HFI angle error vs encoder, sd, deg elec", ylabel="Condition")
ax.title(a, "Tracking quality by condition", "Drive current corrupts the HFI angle; red = the open problem")
fig.tight_layout()
ax.save(fig, f"{OUT}/fig6_error_summary.png", dpi=160)
print("ok")
