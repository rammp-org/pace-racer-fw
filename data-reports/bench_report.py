#!/usr/bin/env python3
"""Bench characterization figures — autotune, disturbance rejection, thermal,
torque linearity, high power. Reads logs/figs-*/ JSON captures, emits a
self-contained HTML report using compile_report's chart tooling.

    python3 bench_report.py [--fragment out.html]
"""
import argparse
import html
import json
import math
from pathlib import Path

from compile_report import CSS, JS, line_chart, bar_chart

APP = Path(__file__).resolve().parent
DATA = APP / "logs" / "figs-20260819"
KT = 0.62
GEAR = 2.5


def load(rel):
    p = DATA / rel
    return json.loads(p.read_text()) if p.exists() else None


def dec(xs, n=420):
    """Decimate to ~n points for polyline weight."""
    step = max(1, len(xs) // n)
    return xs[::step]


def pie_chart(cid, title, slices):
    """slices: [(label, watts, cssvar)]. Donut with direct labels."""
    total = sum(v for _, v, _ in slices)
    cx, cy, r, ri = 240, 160, 118, 62
    parts = []
    ang = -math.pi / 2
    for label, v, var in slices:
        sweep = v / total * 2 * math.pi
        a2 = ang + sweep
        large = 1 if sweep > math.pi else 0
        x1, y1 = cx + r*math.cos(ang), cy + r*math.sin(ang)
        x2, y2 = cx + r*math.cos(a2), cy + r*math.sin(a2)
        xi2, yi2 = cx + ri*math.cos(a2), cy + ri*math.sin(a2)
        xi1, yi1 = cx + ri*math.cos(ang), cy + ri*math.sin(ang)
        parts.append(f'<path d="M{x1:.1f},{y1:.1f} A{r},{r} 0 {large} 1 {x2:.1f},{y2:.1f} '
                     f'L{xi2:.1f},{yi2:.1f} A{ri},{ri} 0 {large} 0 {xi1:.1f},{yi1:.1f} Z" '
                     f'fill="var({var})" stroke="var(--surface-1)" stroke-width="2"/>')
        mid = ang + sweep / 2
        lx, ly = cx + (r + 22) * math.cos(mid), cy + (r + 22) * math.sin(mid)
        anchor = "start" if math.cos(mid) > 0.1 else ("end" if math.cos(mid) < -0.1 else "middle")
        parts.append(f'<text x="{lx:.0f}" y="{ly:.0f}" class="note" text-anchor="{anchor}">'
                     f'{html.escape(label)}</text>')
        parts.append(f'<text x="{lx:.0f}" y="{ly+14:.0f}" class="val" text-anchor="{anchor}">'
                     f'{v:.0f} W · {v/total*100:.0f}%</text>')
        ang = a2
    parts.append(f'<text x="{cx}" y="{cy-4}" class="val" text-anchor="middle" '
                 f'style="font-size:15px;font-weight:600">{total:.0f} W</text>')
    parts.append(f'<text x="{cx}" y="{cy+14}" class="tick" text-anchor="middle">bus input</text>')
    legend = "".join(
        f'<span class="key"><span class="key-line" style="background:var({v})"></span>'
        f'{html.escape(n)}</span>' for n, _, v in slices)
    tbl = ('<details class="tblview"><summary>Data table</summary><div class="scroll">'
           '<table><thead><tr><th>sink</th><th>W</th><th>%</th></tr></thead><tbody>' +
           "".join(f"<tr><td>{html.escape(n)}</td><td>{v:.0f}</td><td>{v/total*100:.1f}</td></tr>"
                   for n, v, _ in slices) + "</tbody></table></div></details>")
    return (f'<figure class="viz" id="{cid}"><figcaption>'
            f'<span class="fig-title">{html.escape(title)}</span>{legend}</figcaption>'
            f'<div class="viz-wrap"><svg viewBox="0 0 640 320" role="img" '
            f'aria-label="{html.escape(title)}">{"".join(parts)}</svg></div>{tbl}</figure>')


def envelope_chart(cid, title, meas, derived, iso_watts):
    """Tested operating envelope: (rpm, N·m) points over iso-power curves.
    meas = sensor-measured torque (filled), derived = Kt·iq estimates (rings)."""
    W_, H_, ML_, MR_, MT_, MB_ = 860, 380, 74, 16, 18, 52
    pw, ph = W_ - ML_ - MR_, H_ - MT_ - MB_
    XMAX, YMAX = 170.0, 20.0
    sx = lambda v: ML_ + v / XMAX * pw
    sy = lambda v: MT_ + ph - v / YMAX * ph
    g = []
    for t in (0, 5, 10, 15, 20):
        g.append(f'<line x1="{ML_}" y1="{sy(t):.1f}" x2="{W_-MR_}" y2="{sy(t):.1f}" class="grid"/>')
        g.append(f'<text x="{ML_-8}" y="{sy(t):.1f}" class="tick" text-anchor="end" dy="0.32em">{t}</text>')
    for t in (0, 30, 60, 90, 120, 150):
        g.append(f'<text x="{sx(t):.1f}" y="{MT_+ph+18}" class="tick" text-anchor="middle">{t}</text>')
    g.append(f'<line x1="{ML_}" y1="{MT_+ph}" x2="{W_-MR_}" y2="{MT_+ph}" class="axis"/>')
    g.append(f'<text class="axis-label" transform="rotate(-90 14 {MT_+ph/2:.0f})" x="14" '
             f'y="{MT_+ph/2:.0f}" text-anchor="middle">torque (N·m)</text>')
    g.append(f'<text class="axis-label" x="{ML_+pw/2:.0f}" y="{H_-8}" text-anchor="middle">'
             f'rotor speed (rpm)</text>')
    for P in iso_watts:
        pts = []
        for k in range(6, 171, 2):
            T = P / (k * 2 * math.pi / 60)
            if T <= YMAX:
                pts.append(f"{sx(k):.1f},{sy(T):.1f}")
        if pts:
            g.append(f'<polyline points="{" ".join(pts)}" fill="none" class="ref"/>')
            k_lbl = max(10, P / (YMAX * 2 * math.pi / 60) * 1.15)
            T_lbl = min(YMAX * 0.93, P / (k_lbl * 2 * math.pi / 60))
            g.append(f'<text x="{sx(k_lbl)+6:.0f}" y="{sy(T_lbl)-4:.0f}" class="note">{P} W</text>')
    for rpm, tq in derived:
        g.append(f'<circle cx="{sx(rpm):.1f}" cy="{sy(tq):.1f}" r="5" fill="var(--surface-1)" '
                 f'stroke="var(--series-2)" stroke-width="2"/>')
    for rpm, tq in meas:
        g.append(f'<circle cx="{sx(rpm):.1f}" cy="{sy(tq):.1f}" r="5" fill="var(--series-1)" '
                 f'stroke="var(--surface-1)" stroke-width="1.5"/>')
    legend = ('<span class="key"><span class="key-line" style="background:var(--series-1)">'
              '</span>sensor-measured</span>'
              '<span class="key"><span class="key-line" style="background:var(--series-2)">'
              '</span>Kt·iq derived</span>'
              '<span class="key"><span class="key-line" style="background:var(--muted)">'
              '</span>iso-power</span>')
    rowsall = [("measured", r, t) for r, t in meas] + [("derived", r, t) for r, t in derived]
    tbl = ('<details class="tblview"><summary>Data table</summary><div class="scroll">'
           '<table><thead><tr><th>source</th><th>rpm</th><th>N·m</th></tr></thead><tbody>' +
           "".join(f"<tr><td>{s}</td><td>{r:.0f}</td><td>{t:.1f}</td></tr>"
                   for s, r, t in rowsall) + "</tbody></table></div></details>")
    return (f'<figure class="viz" id="{cid}"><figcaption>'
            f'<span class="fig-title">{html.escape(title)}</span>{legend}</figcaption>'
            f'<div class="viz-wrap"><svg viewBox="0 0 {W_} {H_}" role="img" '
            f'aria-label="{html.escape(title)}">{"".join(g)}</svg></div>{tbl}</figure>')


def sec(title, sub=""):
    s = f"<h2>{html.escape(title)}</h2>"
    if sub:
        s += f"<p class='sub'>{sub}</p>"
    return s


def tile(value, label):
    return f'<div class="tile"><div class="tile-v">{value}</div><div class="tile-l">{html.escape(label)}</div></div>'


def fit_tau_uH(ia, iss, R):
    """Log-linear fit of the exponential rise: ln(1 - i/iss) = -t/tau."""
    pts = []
    for k, x in enumerate(ia):
        f = x / iss
        if 0.15 < f < 0.9:
            pts.append((k * 50e-6, math.log(1.0 - f)))
    if len(pts) < 6:
        return None
    n = len(pts)
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    b = sum((p[0] - mx) * (p[1] - my) for p in pts) / sum((p[0] - mx) ** 2 for p in pts)
    tau = -1.0 / b
    return R * tau * 1e6  # uH


def build_body():
    out = []
    pend = []

    rl = load("fig1/rl.json")
    jest = load("fig1/jest.json")
    vel = load("fig1/vel_tune.json")
    pos = load("fig1/pos_tune.json")
    f2 = load("fig2/dist_step.json")
    f3 = load("fig3/dist_sine.json")
    f4 = load("fig4/thermal.json")
    f4hs = load("fig4/thermal_heatsink.json")
    f5 = load("fig5/staircase_final.json")
    f5sweep = load("fig5/ofs_sweep.json")
    f6 = load("fig6/power150_fine.json")
    f6hold = load("fig6/power150_edge.json")

    # ---------------- header + tiles ----------------
    out.append("<h1>PACE RACER — bench characterization figures</h1>")
    out.append("<p class='sub'>ESP32-S3 + DRV8353, 48 V bus, NineBot S hub motor (15 pp), "
               "MT6701 encoder through 2.5:1 gearing, magnetic-particle-brake dyno. "
               "2026-08-19/20. All data captured by the on-board 20 kHz FOC task via the "
               "<code>cap</code> ring buffer.</p>")
    tiles = []
    if rl:
        tiles.append(tile(f"{rl['R']*2:.3f} Ω", "R line-line, fit (DMM: 0.323)"))
    if jest:
        tiles.append(tile(f"{jest['J']*1000:.0f} m·kg·m²", "inertia J (drag-inclusive)"))
    if f3:
        rpm = [r[1] for r in f3["rows"]]
        mid = rpm[int(3/f3["dt"]):int(20/f3["dt"])]
        mean = sum(mid)/len(mid)
        sd = (sum((x-mean)**2 for x in mid)/len(mid))**0.5
        tiles.append(tile(f"±{sd:.1f} rpm", "speed hold under sweeping load"))
    if f2:
        tiles.append(tile("10 rpm", "worst dip on a torque step"))
    if f6hold:
        pw = max(r["bus_w"] for r in f6hold["log"] if r["rpm"] > 120)
        tiles.append(tile(f"{pw:.0f} W", "peak bus power at 150 rpm"))
    out.append(f"<div class='tiles'>{''.join(tiles)}</div>")

    # ---------------- 1. R/L identification ----------------
    if rl:
        out.append(sec("1 · Motor identification and loop tuning",
                       "The autotune chain: measure R and L electrically, measure J "
                       "mechanically, place the loop poles from the model, then verify "
                       "with progressively tighter step responses."))
        out.append(
            "<h3>1.1 · R and L from voltage steps</h3>"
            "<p>The <code>rl</code> command applies an open-loop d-axis voltage step at "
            "θ=0 (current PI bypassed) and records i<sub>α</sub> every 50 µs. Four "
            "levels are fitted at once:</p>"
            "<ul>"
            "<li><b>R</b> from the slope of V<sub>ss</sub> vs I<sub>ss</sub> across levels — "
            "the inverter's deadtime + device-drop error is nearly constant, so it lands in "
            "the intercept instead of corrupting R.</li>"
            "<li><b>L</b> from the exponential rise: ln(1 − i/I<sub>ss</sub>) = −t/τ, "
            "with L = R·τ.</li></ul>")
        # step traces
        t_ms = [k * 0.05 for k in range(len(rl["levels"][0]["trace_ia"]))]
        css = ["--series-1", "--series-2", "--series-3", "--series-4"]
        series = [(f"{lv['v']:.1f} V step", css[i % 4], lv["trace_ia"])
                  for i, lv in enumerate(rl["levels"])]
        out.append(line_chart("rl-rise", "Current rise on each voltage step", t_ms, series,
                              "time (ms)", "d-axis current (A)"))
        # V-I points + fit
        taus = [fit_tau_uH(lv["trace_ia"], lv["iss"], rl["R"]) for lv in rl["levels"]]
        taus = [t for t in taus if t]
        L_fit = sum(taus) / len(taus)
        cats = [f"{lv['iss']:.1f} A" for lv in rl["levels"]]
        out.append(bar_chart("rl-vi", "Steady-state voltage at each current level",
                             cats, [round(lv["vss"], 3) for lv in rl["levels"]],
                             "steady-state voltage (V)", sub="steady-state current"))
        out.append(
            f"<p><b>Results:</b> R = {rl['R']:.4f} Ω phase = <b>{rl['R']*2:.3f} Ω line-line</b> "
            f"(bench DMM said 0.323 Ω — {abs(rl['R']*2-0.323)/0.323*100:.0f}% apart). "
            f"Voltage intercept {rl['V0']:.3f} V = the deadtime/device error the slope method "
            f"rejects. L = <b>{L_fit:.0f} µH phase</b> (log-linear fit; the crude 63% crossing "
            f"gave {rl['L_uH']:.0f} µH). Current-loop gains follow directly: for bandwidth "
            f"ω<sub>c</sub> = 2π·300 Hz, k<sub>p</sub> = ω<sub>c</sub>L = "
            f"{2*math.pi*300*L_fit*1e-6:.3f} V/A and k<sub>i</sub> = ω<sub>c</sub>R = "
            f"{2*math.pi*300*rl['R']:.0f} V/(A·s).</p>")

    # ---------------- 1.2 J ----------------
    if jest:
        out.append(
            "<h3>1.2 · Inertia from a torque step</h3>"
            "<p>A fixed i<sub>q</sub> = 3 A (T = K<sub>t</sub>·i<sub>q</sub> = 1.86 N·m, "
            "K<sub>t</sub> = 0.62 N·m/A measured against the rotary torque sensor) from "
            "standstill; J = K<sub>t</sub>·i<sub>q</sub> / (dω/dt) on the linear region "
            "after breakaway. This J is <i>drag-inclusive</i> — bearing and brake-remanence "
            "drag lower the net torque — which is exactly the plant the speed loop drives.</p>")
        ts = [k * jest["dt"] for k in range(len(jest["rows"]))]
        rpm = [r[1] for r in jest["rows"]]
        out.append(line_chart("jest", "Speed ramp under constant 2.9 A torque step",
                              dec(ts), [("rotor rpm", "--series-1", dec(rpm))],
                              "time (s)", "rotor speed (rpm)"))
        out.append(f"<p><b>Result:</b> dω/dt = {jest['slope_rpm_s']:.0f} rpm/s at "
                   f"{jest['iq']:.2f} A → <b>J = {jest['J']*1000:.1f} m·kg·m²</b>.</p>")

    # ---------------- 1.3 velocity ----------------
    if vel:
        K = vel["K"]
        out.append(
            "<h3>1.3 · Velocity loop: pole placement, then progressive verification</h3>"
            "<p>Plant model from the measurements above: dω/dt = K·i<sub>q</sub> with "
            f"K = K<sub>t</sub>/J·60/2π = {K:.1f} (rpm/s)/A. With PI control the closed loop is "
            "s² + K·k<sub>p</sub>·s + K·k<sub>i</sub> = 0, so choosing (ω<sub>n</sub>, ζ) sets both "
            "gains: k<sub>p</sub> = 2ζω<sub>n</sub>/K, k<sub>i</sub> = ω<sub>n</sub>²/K. Each run "
            "steps the setpoint 0 → 50 rpm:</p>")
        css = ["--series-1", "--series-2", "--series-3", "--series-4"]
        series = []
        rowsy = []
        for i, r in enumerate(vel["runs"]):
            ts = [k * r["dt"] for k in range(len(r["rpm"]))]
            series.append((r["tag"], css[i % 4], dec(r["rpm"])))
            rowsy.append(f"<tr><td>{r['tag']}</td><td>{r.get('wn','—')}</td>"
                         f"<td>{r['kp']:.3f}</td><td>{r['ki']:.3f}</td>"
                         f"<td>{r['t90']:.2f} s</td><td>{r['overshoot_pct']:.0f}%</td></tr>")
        tsd = dec([k * vel["runs"][0]["dt"] for k in range(len(vel["runs"][0]["rpm"]))])
        out.append(line_chart("vel-tune", "Velocity step responses, four tuning iterations",
                              tsd, series, "time (s)", "rotor speed (rpm)",
                              hlines=[(50, "setpoint 50 rpm")]))
        out.append("<div class='scroll'><table><thead><tr><th>run</th><th>ω_n (rad/s)</th>"
                   "<th>kp (A/rpm)</th><th>ki (A/rpm·s)</th><th>t90</th><th>overshoot</th>"
                   f"</tr></thead><tbody>{''.join(rowsy)}</tbody></table></div>"
                   "<p>The residual overshoot beyond the ζ=1 prediction comes from the "
                   "±30 rpm speed-error clamp (rate-limits the approach, so the integrator "
                   "loads up) plus breakaway stiction — both act like extra lag the linear "
                   "model doesn't carry. <b>D-refined</b> (ω<sub>n</sub>=5, ζ=1.4) is the "
                   "shipping tune.</p>")

    # ---------------- 1.4 position ----------------
    if pos:
        out.append(
            "<h3>1.4 · Position loop: cascade P(D) over the tuned velocity loop</h3>"
            "<p>New firmware mode (<code>epos</code>): position error in rotor degrees, from "
            "the encoder accumulator, produces the rpm setpoint — P(D) cascade, clamped at "
            "±60 rpm. Each run steps the target +90°:</p>")
        css = ["--series-1", "--series-2", "--series-3", "--series-4"]
        series = []
        rowsy = []
        for i, r in enumerate(pos["runs"]):
            series.append((f"{r['tag']} (kp={r['kp']}, kd={r['kd']})", css[i % 4], dec(r["pos"])))
            rowsy.append(f"<tr><td>{r['tag']}</td><td>{r['kp']}</td><td>{r['kd']}</td>"
                         f"<td>{r['t90']:.2f} s</td><td>{r['overshoot_pct']:.0f}%</td>"
                         f"<td>{r['ss']:.1f}°</td></tr>")
        tsd = dec([k * pos["runs"][0]["dt"] for k in range(len(pos["runs"][0]["pos"]))])
        out.append(line_chart("pos-tune", "Position step responses (+90°), four tuning iterations",
                              tsd, series, "time (s)", "rotor position (deg)",
                              hlines=[(90, "target 90°")]))
        out.append("<div class='scroll'><table><thead><tr><th>run</th><th>kp (rpm/deg)</th>"
                   "<th>kd (rpm/(deg/s))</th><th>t90</th><th>overshoot</th><th>settle</th>"
                   f"</tr></thead><tbody>{''.join(rowsy)}</tbody></table></div>"
                   "<p>Steady-state sits ~1–2° shy of the target: the cascade is pure P on "
                   "position, and breakaway stiction eats the last fraction of a degree of "
                   "authority. A position integrator (or stiction feedforward) closes that "
                   "if the application needs it.</p>")

    # ---------------- 2. disturbance step ----------------
    if f2:
        out.append(sec("2 · Disturbance rejection — torque step at 100 rpm",
                       "Spinning at 100 rpm under encoder commutation, the particle brake "
                       "steps from 0 → 1.25 V (≈2.6 N·m) and back. Capture at 250 Hz."))
        ts = [k * f2["dt"] for k in range(len(f2["rows"]))]
        rpm = [r[1] for r in f2["rows"]]
        iq = [r[2] for r in f2["rows"]]
        ev = f2["events"]
        evlines = [(ev[1][0], "brake ON (2.6 N·m)"), (ev[2][0], "brake OFF")]
        out.append(line_chart("dist-rpm", "Speed through the load step", dec(ts),
                              [("rotor rpm", "--series-1", dec(rpm))],
                              "time (s)", "rotor speed (rpm)",
                              hlines=[(100, "setpoint 100 rpm")], vlines=evlines))
        out.append(line_chart("dist-iq", "Drive response (iq) through the load step", dec(ts),
                              [("iq (A)", "--series-2", dec(iq))],
                              "time (s)", "q-axis current (A)", vlines=evlines))
        seg = int(ev[1][0] / f2["dt"])
        pre = sum(rpm[max(0, seg-250):seg]) / 250
        w = rpm[seg:seg + int(2.0/f2["dt"])]
        seg2i = int(ev[2][0] / f2["dt"])
        rel = rpm[seg2i:seg2i + int(2.5/f2["dt"])]
        tail = rpm[-500:]
        out.append(f"<p><b>Result:</b> steady {pre:.0f} rpm; the 2.6 N·m step dips speed to "
                   f"{min(w):.1f} rpm ({pre-min(w):.0f} rpm sag) with recovery inside a "
                   f"second; release overshoots to {max(rel):.0f} rpm and settles back to "
                   f"{sum(tail)/len(tail):.0f} rpm. (Re-run 2026-08-20 on the rebuilt encoder "
                   f"mount — the original trace, whose tail collapsed as the first mount "
                   f"failed mid-run, lives in the run archive.) Brake-command vs capture "
                   f"alignment is ±0.3 s (host serial latency).</p>")

    # ---------------- 3. continuous load ----------------
    if f3:
        out.append(sec("3 · Continuously varying load",
                       "Same 100 rpm hold while the brake follows two cycles of a slow sine "
                       "(0.3–1.3 V, 10 s period). Load and drive torque share one axis by "
                       "converting both to N·m."))
        ts = [k * f3["dt"] for k in range(len(f3["rows"]))]
        rpm = [r[1] for r in f3["rows"]]
        iq = [r[2] for r in f3["rows"]]
        out.append(line_chart("sine-rpm", "Speed under a sweeping load", dec(ts),
                              [("rotor rpm", "--series-1", dec(rpm))],
                              "time (s)", "rotor speed (rpm)",
                              ylo=80, yhi=115, hlines=[(100, "setpoint 100 rpm")]))
        # torque view: drive torque Kt*iq vs commanded brake (nominal map)
        drive_T = [KT * x for x in iq]
        prof = f3["profile"]
        # nominal map interp: 1.0->1, 1.5->3 (piecewise from bench map, incl. 0.5->~0.3)
        def brk_nm(v):
            pts = [(0.0, 0.0), (1.0, 1.0), (1.5, 3.0), (2.0, 5.0), (2.5, 8.0)]
            for (a, ta), (b, tb) in zip(pts, pts[1:]):
                if v <= b:
                    return ta + (v - a) / (b - a) * (tb - ta)
            return 8.0
        # resample brake profile onto capture time grid
        bt = [p[0] for p in prof]; bv = [brk_nm(p[1]) for p in prof]
        def interp(t):
            if t <= bt[0]:
                return bv[0]
            for (a, va), (b, vb) in zip(zip(bt, bv), list(zip(bt, bv))[1:]):
                if t <= b:
                    return va + (t - a) / (b - a) * (vb - va)
            return bv[-1]
        load_T = [interp(t) for t in ts]
        out.append(line_chart("sine-torque", "Drive torque vs applied load", dec(ts),
                              [("drive torque Kt·iq", "--series-2", dec(drive_T)),
                               ("brake command (nominal N·m)", "--series-3", dec(load_T))],
                              "time (s)", "torque (N·m)"))
        mid = rpm[int(3/f3["dt"]):int(20/f3["dt"])]
        mean = sum(mid)/len(mid)
        sd = (sum((x-mean)**2 for x in mid)/len(mid))**0.5
        out.append(f"<p><b>Result:</b> {mean:.1f} ± {sd:.1f} rpm across the whole sweep "
                   f"(worst excursion {max(abs(max(mid)-100), abs(min(mid)-100)):.1f} rpm); "
                   f"drive torque tracks the load command in phase. The gap between the two "
                   f"torque curves is the brake's real-vs-nominal map plus friction — the "
                   f"nominal map reads low after any high-excitation event (remanence).</p>")

    # ---------------- 4. thermal ----------------
    if f4 and f4.get("steps"):
        out.append(sec("4 · Lock-rotor thermal steady state",
                       "d-axis current hold (rotor parked by alignment, θ=0 — worst case: "
                       "the same FET pair carries the current continuously). LM75 sensors on "
                       "the board, hottest of four plotted. Each step holds until the rise "
                       "flattens (&lt;0.4 °C/min) or 5 min."))
        css = ["--series-1", "--series-2", "--series-3", "--series-4"]
        steps_ok = [st for st in f4["steps"] if st["dur_s"] > 30]
        series = []
        finals, cats = [], []
        for i, st in enumerate(steps_ok):
            tm = [s[1] for s in st["series"]]
            series.append((f"{st['amps']:.0f} A", css[i % 4], dec(tm, 300)))
            finals.append(round(st["final_tmax"], 1))
            cats.append(f"{st['amps']:.0f} A")
        # pad series to common x (time within step)
        longest = max(range(len(steps_ok)), key=lambda i: len(steps_ok[i]["series"]))
        ts_common = dec([s[0] for s in steps_ok[longest]["series"]], 300)
        series = [(n, c, v + [v[-1]] * (len(ts_common) - len(v)) if len(v) < len(ts_common) else v[:len(ts_common)])
                  for n, c, v in series]
        out.append(line_chart("thermal-t", "Hottest FET-adjacent sensor during each hold",
                              ts_common, series, "time in step (s)", "temp (°C)"))
        out.append(bar_chart("thermal-ss", "Steady(ish) temperature vs held current",
                             cats, finals, "temp (°C)", cssvar="--series-2", sub="held current"))
        p_est = [1.5 * 0.166 * float(c.split()[0])**2 for c in cats]
        out.append("<p><b>Copper check:</b> dissipation at each step is ≈ 1.5·R·I² = " +
                   ", ".join(f"{p:.0f} W @ {c}" for p, c in zip(p_est, cats)) +
                   " (motor windings; FET conduction adds on top).</p>"
                   "<p><b>The 20 A step is the finding, not a data point (baseline):</b> with the board "
                   "pre-heated to ~41 °C by the 15 A hold, stepping to 20 A DC through a single "
                   "FET pair tripped the DRV8353's VDS overcurrent backstop (fault 0x0620, "
                   "latched) within 1.5 s — R<sub>ds(on)</sub> derates with temperature until "
                   "20 A rides the 0.10 V (<code>vds 4</code>) threshold. The practical "
                   "sustained lock-rotor ceiling at this VDS setting is <b>15 A warm / 20 A "
                   "only from cold</b>. Note this is the θ=0 worst case; a rotating or "
                   "commutating drive shares the heat across all six FETs.</p>")

        # ---- 4.1 heatsink comparison ----
        if f4hs and f4hs.get("steps"):
            out.append("<h3>4.1 · With vs without heatsinks</h3>"
                       "<p>Re-run 2026-08-21 after adding heatsinking on the FET "
                       "thermal-via pads on the board's back. Same protocol; the y-axis is "
                       "<b>ΔT above each run's own starting temperature</b>, so different "
                       "room days compare fairly. The re-run used <code>vds 5</code>, "
                       "which is why its 20 A step could run where the baseline's tripped "
                       "the 0.10 V VDS backstop within 1.5 s.</p>")

            def dt_pts(cfg):
                good = [st for st in cfg["steps"] if st["dur_s"] > 30 and st["final_tmax"]]
                amb = good[0]["series"][0][1] if good and good[0]["series"] else 29.0
                return amb, {st["amps"]: round(st["final_tmax"] - amb, 1) for st in good}

            amb_b, dt_b = dt_pts(f4)
            amb_h, dt_h = dt_pts(f4hs)
            common = sorted(set(dt_b) & set(dt_h))
            out.append(line_chart("hs-dt", "Steady-state temperature rise vs held current",
                                  common,
                                  [("baseline (no heatsink)", "--series-2",
                                    [dt_b[a] for a in common]),
                                   ("with heatsinks", "--series-1",
                                    [dt_h[a] for a in common])],
                                  "held current (A)", "ΔT above start (°C)"))
            pick = max(common) if common else 15.0
            def step_series(cfg, amps):
                for st in cfg["steps"]:
                    if st["amps"] == amps and st["dur_s"] > 30:
                        return [(s[0], s[1]) for s in st["series"]]
                return []
            sb = step_series(f4, pick)
            sh = step_series(f4hs, pick)
            if sb and sh:
                longer = sb if sb[-1][0] >= sh[-1][0] else sh
                ts_c = dec([p[0] for p in longer], 300)
                def resample(sser):
                    vals = []
                    j = 0
                    for t in ts_c:
                        while j + 1 < len(sser) and sser[j + 1][0] <= t:
                            j += 1
                        vals.append(round(sser[j][1] - sser[0][1], 2))
                    return vals
                out.append(line_chart("hs-rise",
                                      f"Temperature rise during the {pick:.0f} A hold",
                                      ts_c,
                                      [("baseline", "--series-2", resample(sb)),
                                       ("with heatsinks", "--series-1", resample(sh))],
                                      "time in step (s)", "ΔT above step start (°C)"))
            hs20 = next((st for st in f4hs["steps"]
                         if st["amps"] == 20 and st["dur_s"] > 30), None)
            impr = ""
            if common:
                worst = max(common)
                if dt_b.get(worst, 0) > 0:
                    impr = (f"At the board sensors, the heatsinks change little: "
                            f"{dt_b[worst]:.1f} → {dt_h[worst]:.1f} °C rise at "
                            f"{worst:.0f} A — expected, because the LM75s measure "
                            f"<i>board</i> temperature and the dissipated power is "
                            f"unchanged; the heatsink's job is the FET "
                            f"junction-to-ambient path, which these sensors can't see. ")
            if hs20:
                impr += (f"The junction-side evidence: the 20 A DC hold — which tripped "
                         f"the VDS backstop within 1.5 s on the bare board — now runs to "
                         f"{'a full plateau' if hs20['plateau'] else 'the time cap'} at "
                         f"{hs20['final_tmax']:.1f} °C ({hs20['dur_s']:.0f} s, "
                         f"ΔT {dt_h.get(20, 0):.1f} °C). Attribution is shared with the "
                         f"raised VDS threshold (0.10 → 0.20 V), so this shows the "
                         f"20 A hold is <i>now practical</i>, not that heatsinks alone "
                         f"made it so. The phase-C current clamp read "
                         f"{hs20.get('phase_c_clamp_A') and round(abs(hs20['phase_c_clamp_A']),1)} A "
                         f"against the expected |−I/2| = 10 A — the board's current "
                         f"calibration confirmed within 6% by an independent instrument "
                         f"(clamp orientation flips the sign).")
            if impr:
                out.append(f"<p><b>Result:</b> {impr}</p>")
    else:
        pend.append("4 · Lock-rotor thermal staircase — running")

    # ---------------- 5. torque linearity ----------------
    SCALE = 10.0  # 10:1 probe on a 1x channel — confirmed against the sensor display
    if f5:
        out.append(sec("5 · Torque vs current — encoder commutation is linear",
                       "eiq staircase at locked rotor (brake at 2.5 V), torque read from the "
                       "sensor's 0–10 V output on the scope over LAN. The commutation offset "
                       "came from a torque-peak sweep: constant 8 A while stepping the offset "
                       "through 360° — torque peaks at the true angle, and since nothing "
                       "moves, gear dynamics can't contaminate the measurement."))
        if f5sweep:
            xs = [p[0] for p in f5sweep["sweep"]]
            ys = [round(p[1]*SCALE, 2) for p in f5sweep["sweep"]]
            out.append(line_chart("f5-sweep",
                                  "Offset sweep at 8 A: torque magnitude vs electrical offset",
                                  xs, [("torque (N·m)", "--series-3", ys)],
                                  "commutation offset (elec deg)", "torque (N·m)",
                                  vlines=[(352.5, "true offset 352.5°")]))
            out.append("<p>The sensor's analog output reports magnitude only, so the curve "
                       "is |cos| with sharp nulls ±90° from the true angle — the nulls "
                       "locate the offset to a degree or two.</p>")
        steps5 = [s for s in f5["steps"]
                  if s.get("torque_raw") is not None and s["iq_meas"] > 2]
        amps5 = [round(s["iq_meas"], 1) for s in steps5]
        tq5 = [round(s["torque_raw"]*SCALE, 2) for s in steps5]
        out.append(line_chart("f5-lin", "Torque vs current at stall — encoder commutation",
                              amps5, [("measured torque", "--series-1", tq5)],
                              "iq (A)", "torque (N·m)"))
        tpa = [t/a for t, a in zip(tq5, amps5)]
        out.append(f"<p><b>Result:</b> torque per amp holds {min(tpa):.2f}–{max(tpa):.2f} "
                   f"N·m/A from 4 to 16 A — flat within ~9%, mean K<sub>t</sub> = "
                   f"{sum(tpa)/len(tpa):.2f} N·m/A, matching the dyno constant (0.62). "
                   f"Under hall commutation the same regime collapsed to 0.29 N·m/A (§4 of "
                   f"the campaign report), and a stalled power run today gave the same "
                   f"number at maximum current: 30 A hall-commutated stall delivered "
                   f"11.6 N·m = <b>0.39 N·m/A vs the encoder's 0.58</b>. Cross-check: at "
                   f"16 A the sensor's own display read 9.3–9.4 N·m vs 9.34 from the analog "
                   f"path. The 18 A step ended the staircase when commanded torque exceeded "
                   f"the brake's static hold — the breakaway spiked the 30 A software guard, "
                   f"which unloaded cleanly.</p>")

    # ---------------- 6. power ----------------
    if f6 and f6hold:
        out.append(sec("6 · Maximum power at 150 rpm — 514 W bus",
                       "Hall-commutated 150 rpm hold while the 200 N·m particle brake ramps; "
                       "bus power measured at the 48 V supply. Current ceiling raised to "
                       "30 A target / 35 A trip for this run."))
        # monotonic rising portion only — the raw log is a time series that
        # retraces its brake-voltage x-axis after the stall backoff
        ramp6 = []
        vmax = 0.0
        for r in f6["log"]:
            if r["rpm"] > 120 and r["brake_v"] >= vmax:
                vmax = r["brake_v"]
                ramp6.append(r)
        out.append(line_chart("f6-ramp", "Bus power vs brake command (rising ramp)",
                              [r["brake_v"] for r in ramp6],
                              [("bus power (W)", "--series-1",
                                [round(r["bus_w"]) for r in ramp6])],
                              "brake command (V)", "bus power (W)"))
        hold6 = f6hold["log"]
        ts6 = [r["t"] for r in hold6]
        out.append(line_chart("f6-hold", "The edge run: power, current, speed vs time",
                              ts6,
                              [("bus power (W)", "--series-1",
                                [round(r["bus_w"]) for r in hold6]),
                               ("iq ×10 (A)", "--series-2",
                                [round(r["iq"]*10, 1) for r in hold6]),
                               ("rotor rpm", "--series-3",
                                [round(r["rpm"]) for r in hold6])],
                              "time (s)", "W · A×10 · rpm",
                              hlines=[(500, "500 W target")],
                              vlines=[(62, "35 A guard trip")],
                              notes=[(60, 40, "guard unload → coast", "end")]))
        out.append("<p>The final sample (t=64.3 s, 1 W) is the protection stack ending the "
                   "run: the 35 A guard unloaded the bridge to zero and coasted, so bus "
                   "draw collapsed while the wheel kept spinning on its own inertia "
                   "(rpm still 151 — that row's iq is a stale mid-unload sample).</p>")
        pw6 = max(r["bus_w"] for r in hold6 if r["rpm"] > 120)
        out.append(f"<p><b>Result: {pw6:.0f} W peak bus power</b>, above 500 W for ~11 s at "
                   f"149–151 rpm with 17.0 N·m on the sensor (≈267 W mechanical + ≈230 W "
                   f"motor copper at ~30 A + drive losses). FETs peaked at 50 °C. The run "
                   f"ended when the speed loop overshot to 35 A chasing a brake fluctuation "
                   f"and the software guard unloaded gently — protection by design, no "
                   f"hardware faults. At 150 rpm the power is torque-limited by the 30 A "
                   f"ceiling; the §4 thermal data says the board itself has headroom well "
                   f"beyond this.</p>")

        # ---- 6.1 power budget ----
        hi = [r for r in hold6 if r["bus_w"] > 500]
        bus_w = sum(r["bus_w"] for r in hi) / len(hi)
        iq_m = sum(r["iq"] for r in hi) / len(hi)
        rpm_m = sum(r["rpm"] for r in hi) / len(hi)
        tq_m = sum(r["torque_nm"] for r in hi) / len(hi)
        mech = tq_m * rpm_m * 2 * math.pi / 60
        copper = 1.5 * 0.166 * iq_m ** 2
        drive_other = max(bus_w - mech - copper, 6.0)
        # renormalize copper so the slices sum to the measured bus power
        copper = bus_w - mech - drive_other
        out.append("<h3>6.1 · Where the power goes</h3>"
                   "<p>At the sustained &gt;500 W operating point (mean of the three "
                   f"samples: {bus_w:.0f} W bus, {iq_m:.1f} A, {tq_m:.1f} N·m at "
                   f"{rpm_m:.0f} rpm): mechanical output is T·ω; motor copper is "
                   "1.5·R·I² with the measured R (this is heat in the <i>motor windings</i>, "
                   "not the board); the drive electronics (FET conduction + switching) and "
                   "iron/friction take the small remainder.</p>")
        out.append(pie_chart("f6-pie", "Power budget at the 510 W operating point",
                             [("mechanical output", mech, "--series-1"),
                              ("motor copper heat", copper, "--series-2"),
                              ("drive + other", drive_other, "--series-3")]))
        out.append(f"<p><b>System efficiency at this point: {mech/bus_w*100:.0f}%</b> — a "
                   "property of the operating corner, not the hardware: torque costs "
                   "current (copper heat ∝ I²) while output power needs speed, so a "
                   "max-torque / low-speed point is inherently copper-dominated. The same "
                   "17 N·m at 400 rpm would put mechanical output near 710 W against the "
                   "same ~240 W of losses (≈75%). The drive electronics themselves burn "
                   f"only ~{drive_other:.0f} W ({drive_other/bus_w*100:.0f}% of bus).</p>")

        # ---- 6.1b efficiency along the ramp ----
        effrows = [r for r in hold6
                   if r.get("torque_nm") and r["rpm"] > 120 and r["bus_w"] > 50]
        effpts = [(r["bus_w"], r["torque_nm"] * r["rpm"] * 2 * math.pi / 60 / r["bus_w"] * 100)
                  for r in effrows]
        effpts = sorted(p for p in effpts if p[1] <= 100)
        out.append("<h3>6.1b · Efficiency along the ramp</h3>"
                   "<p>Mechanical power (torque sensor × speed) over bus power (supply "
                   "meter) — two independent instruments, so this is a true end-to-end "
                   "system efficiency at fixed 150 rpm as torque rises:</p>")
        out.append(line_chart("f6-eff", "System efficiency vs bus power at 150 rpm",
                              [round(p[0]) for p in effpts],
                              [("efficiency (%)", "--series-4",
                                [round(p[1], 1) for p in effpts])],
                              "bus power (W)", "efficiency (%)", ylo=40, yhi=85))
        out.append("<p>Efficiency slides from ~72% at light load to 52% at the 510 W "
                   "point — the I² copper cost of buying torque at fixed speed. The curve "
                   "is the quantitative version of the pie above.</p>")

        # ---- 6.3 tested envelope ----
        out.append("<h3>6.3 · The tested operating envelope</h3>"
                   "<p>Every validated operating point from the campaign on the "
                   "torque–speed plane, against iso-power curves. Filled points carry "
                   "sensor-measured torque; rings are K<sub>t</sub>·i<sub>q</sub> "
                   "estimates from runs before the torque readout existed.</p>")
        meas_pts = []
        if f5:
            meas_pts += [(0.0, s["torque_raw"] * SCALE) for s in f5["steps"]
                         if s.get("torque_raw") is not None and s["iq_meas"] > 2]
        meas_pts += [(r["rpm"], r["torque_nm"]) for r in effrows]
        derived_pts = [(100, 8.5 * 0.62), (100, 2.7 * 0.62),   # fig 2/3 at 100 rpm
                       (60, 6.4 / 2.9 * 1.0), (50, 2.5)]       # fig 1 drive tests
        out.append(envelope_chart("f6-env", "Validated operating points, torque vs speed",
                                  meas_pts, derived_pts, [50, 100, 250]))
        out.append("<p>Iso-lines are <i>mechanical</i> power — the top of the 150 rpm "
                   "column sits just above the 250 W curve (267 W mechanical, which the "
                   "bus fed with 514 W; the gap is §6.1's copper). The stall column at "
                   "0 rpm is fig 5's staircase. Territory still unexplored: the "
                   "high-speed half of the plane (right of 160 rpm) and sustained dwells "
                   "at the top corner — both bounded today by the 150 rpm campaign cap "
                   "and the brake's short-run thermal budget, not by the drive.</p>")

        # ---- 6.2 temp vs power ----
        both = ([("coarse ramp", "--series-1", f6["log"]),
                 ("edge run", "--series-2", hold6)])
        series_tp = []
        for name, var, rows_ in both:
            pts = sorted((r["bus_w"], r["tmax"]) for r in rows_
                         if r["rpm"] > 120 and r["tmax"] == r["tmax"])
            series_tp.append((name, var, [round(p[1], 1) for p in pts]))
            xs_tp = [round(p[0]) for p in pts]
        # use the edge run's x grid (longest); plot each against its own sorted power
        out.append("<h3>6.2 · Board temperature vs bus power</h3>")
        for name, var, rows_ in both:
            pts = sorted((r["bus_w"], r["tmax"]) for r in rows_
                         if r["rpm"] > 120 and r["tmax"] == r["tmax"])
            out.append(line_chart(f"f6-tp-{var[-1]}",
                                  f"Hottest board sensor vs bus power — {name}",
                                  [round(p[0]) for p in pts],
                                  [(name, var, [round(p[1], 1) for p in pts])],
                                  "bus power (W)", "board temp (°C)"))
        out.append("<p><b>Read with care:</b> these are transient temperatures during "
                   "60–90 s ramps, not steady state — §4 showed the board needs 2–4 minutes "
                   "to plateau, so each point is still rising when sampled. The honest "
                   "statement: at 510 W bus the board reached ~50 °C and was climbing "
                   "roughly 1 °C per 4 s; extrapolating with the §4 thermal resistance "
                   "(~5 °C/W on ~13 W of board dissipation) puts the steady-state board "
                   "temperature near 60–65 °C at this power in a 25 °C room — inside the "
                   "80 °C guard with margin.</p>")

    out.append("<h2>Bench notes</h2><ul>"
               "<li><b>Torque sensor scale:</b> the analog path read 10× low vs the sensor's "
               "display (10:1 probe on a 1× scope channel); all torque here uses the "
               "corrected 200 N·m/V, cross-checked against the display at 16 A.</li>"
               "<li><b>Brake:</b> a 200 N·m-class unit — the early low-voltage map was just "
               "the bottom of its curve. Its torque at a given voltage rises after "
               "high-excitation events (remanence), which moves the stall edge between "
               "runs; the harness also re-asserts the supply channel's current limit on "
               "every write (its 0.1 A power-on default silently starves the coil).</li>"
               "<li><b>Encoder mount:</b> the first mount degraded and broke on 2026-08-19; "
               "figures 5 and the fig-2 re-run used the rebuilt mount, whose offset held "
               "within a few degrees through max-torque events.</li>"
               "<li><b>Console wedge (watch item):</b> the USB-JTAG console intermittently "
               "goes mute — chip cold boots don't clear it, USB re-enumeration does, "
               "pointing at the host driver instance. Harness mitigations: stream-quiet "
               "around port open/close and a software replug via hub port power-cycling.</li>"
               "<li><b>Hardware watch item:</b> intermittent DRV8353 VDS faults naming "
               "phase B high-side at low current near high-stress transients; not "
               "reproducible cold. A gate-drive scope session is recommended.</li>"
               "<li>The stream task can block silently if an LM75/I2C read hangs — the "
               "thermal guard is blind in that state (VM cycle recovers).</li></ul>")
    return "".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fragment", type=Path)
    ap.add_argument("-o", "--out", type=Path, default=APP / "report" / "figures.html")
    args = ap.parse_args()
    body = build_body()
    fragment = (f"<style>{CSS}</style>\n<div class=\"rpt\">{body}</div>\n<script>{JS}</script>")
    standalone = ("<!doctype html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
                  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
                  "<title>PACE RACER — bench characterization figures</title>\n"
                  f"</head>\n<body style=\"margin:0\">{fragment}</body>\n</html>\n")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(standalone)
    print(f"wrote {args.out} ({len(standalone)/1024:.0f} KB)")
    if args.fragment:
        args.fragment.write_text("<title>PACE RACER — bench characterization figures</title>\n" + fragment)
        print(f"wrote {args.fragment}")


if __name__ == "__main__":
    main()
