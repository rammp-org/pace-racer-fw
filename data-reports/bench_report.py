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


def envelope_chart(cid, title, meas, derived, iso_watts, xmax=170.0):
    """Tested operating envelope: (rpm, N·m) points over iso-power curves.
    meas = sensor-measured torque (filled), derived = Kt·iq estimates (rings)."""
    W_, H_, ML_, MR_, MT_, MB_ = 860, 380, 74, 16, 18, 52
    pw, ph = W_ - ML_ - MR_, H_ - MT_ - MB_
    XMAX, YMAX = xmax, 20.0
    sx = lambda v: ML_ + v / XMAX * pw
    sy = lambda v: MT_ + ph - v / YMAX * ph
    g = []
    for t in (0, 5, 10, 15, 20):
        g.append(f'<line x1="{ML_}" y1="{sy(t):.1f}" x2="{W_-MR_}" y2="{sy(t):.1f}" class="grid"/>')
        g.append(f'<text x="{ML_-8}" y="{sy(t):.1f}" class="tick" text-anchor="end" dy="0.32em">{t}</text>')
    xticks = (0, 30, 60, 90, 120, 150) if XMAX <= 200 else tuple(range(0, int(XMAX), 100))
    for t in xticks:
        g.append(f'<text x="{sx(t):.1f}" y="{MT_+ph+18}" class="tick" text-anchor="middle">{t}</text>')
    g.append(f'<line x1="{ML_}" y1="{MT_+ph}" x2="{W_-MR_}" y2="{MT_+ph}" class="axis"/>')
    g.append(f'<text class="axis-label" transform="rotate(-90 14 {MT_+ph/2:.0f})" x="14" '
             f'y="{MT_+ph/2:.0f}" text-anchor="middle">torque (N·m)</text>')
    g.append(f'<text class="axis-label" x="{ML_+pw/2:.0f}" y="{H_-8}" text-anchor="middle">'
             f'rotor speed (rpm)</text>')
    for P in iso_watts:
        pts = []
        for k in range(6, int(XMAX) + 1, 2):
            T = P / (k * 2 * math.pi / 60)
            if T <= YMAX:
                pts.append(f"{sx(k):.1f},{sy(T):.1f}")
        if pts:
            g.append(f'<polyline points="{" ".join(pts)}" fill="none" class="ref"/>')
            k_lbl = min(XMAX * 0.86, max(10, P / (YMAX * 2 * math.pi / 60) * 1.15))
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


def multi_xy(cid, title, series, xlabel, ylabel, hlines=(), ylo=0.0, yhi=None):
    """Multiple polylines, each with its OWN x grid — for comparing curves
    measured on different grids (e.g. power vs torque at three speeds).
    series: [(name, cssvar, [(x, y), ...])]."""
    W_, H_, ML_, MR_, MT_, MB_ = 860, 380, 74, 16, 18, 52
    pw, ph = W_ - ML_ - MR_, H_ - MT_ - MB_
    xs = [p[0] for _, _, pts in series for p in pts]
    ys = [p[1] for _, _, pts in series for p in pts] + [v for v, _ in hlines]
    xmax = max(xs) * 1.05
    ymax = yhi if yhi is not None else max(ys) * 1.07
    ymin = ylo
    def _nice(lo, hi, n=6):
        span = hi - lo
        step = 10 ** math.floor(math.log10(span / n))
        for m in (1, 2, 2.5, 5, 10):
            if span / (step * m) <= n:
                step *= m
                break
        v = math.ceil(lo / step) * step
        out_ = []
        while v <= hi + 1e-9:
            out_.append(v)
            v += step
        return out_
    sx = lambda v: ML_ + v / xmax * pw
    sy = lambda v: MT_ + ph - (v - ymin) / (ymax - ymin) * ph
    g = []
    for tk in _nice(ymin, ymax):
        g.append(f'<line x1="{ML_}" y1="{sy(tk):.1f}" x2="{W_-MR_}" y2="{sy(tk):.1f}" class="grid"/>')
        g.append(f'<text x="{ML_-8}" y="{sy(tk):.1f}" class="tick" text-anchor="end" dy="0.32em">{tk:g}</text>')
    for tk in _nice(0, xmax, 8):
        g.append(f'<text x="{sx(tk):.1f}" y="{MT_+ph+18}" class="tick" text-anchor="middle">{tk:g}</text>')
    g.append(f'<line x1="{ML_}" y1="{MT_+ph}" x2="{W_-MR_}" y2="{MT_+ph}" class="axis"/>')
    g.append(f'<text class="axis-label" transform="rotate(-90 14 {MT_+ph/2:.0f})" x="14" '
             f'y="{MT_+ph/2:.0f}" text-anchor="middle">{html.escape(ylabel)}</text>')
    g.append(f'<text class="axis-label" x="{ML_+pw/2:.0f}" y="{H_-8}" text-anchor="middle">'
             f'{html.escape(xlabel)}</text>')
    for v, lbl in hlines:
        g.append(f'<line x1="{ML_}" y1="{sy(v):.1f}" x2="{W_-MR_}" y2="{sy(v):.1f}" class="ref"/>')
        g.append(f'<text x="{W_-MR_}" y="{sy(v)-5:.1f}" class="note" text-anchor="end">{html.escape(lbl)}</text>')
    for name, var, pts in series:
        pl = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in pts)
        g.append(f'<polyline points="{pl}" fill="none" stroke="var({var})" '
                 f'stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>')
        ex, ey = sx(pts[-1][0]), sy(pts[-1][1])
        g.append(f'<circle cx="{ex:.1f}" cy="{ey:.1f}" r="4" fill="var({var})" '
                 f'stroke="var(--surface-1)" stroke-width="2"/>')
    legend = "".join(
        f'<span class="key"><span class="key-line" style="background:var({v})"></span>'
        f'{html.escape(n)}</span>' for n, v, _ in series)
    tbl = ('<details class="tblview"><summary>Data table</summary><div class="scroll">'
           f'<table><thead><tr><th>series</th><th>{html.escape(xlabel)}</th>'
           f'<th>{html.escape(ylabel)}</th></tr></thead><tbody>' +
           "".join(f"<tr><td>{html.escape(n)}</td><td>{a:.1f}</td><td>{b:.1f}</td></tr>"
                   for n, _, pts in series for a, b in pts) +
           "</tbody></table></div></details>")
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

    def load_run(rel):
        q = APP / "logs" / rel
        return json.loads(q.read_text()) if q.exists() else None
    la7 = load_run("run-20260826-1kw/ladder_1kw.json")
    lb7 = load_run("run-20260826-1kw/ladder_1kw_b.json")
    lc7 = load_run("run-20260826-1kw/ladder_1kw_c.json")
    lf7 = load_run("run-20260826-1kw/ladder_1kw_f_500rpm.json")
    th8 = load_run("run-20260827-torque-hold/torque_hold.json")

    # ---------------- header + tiles ----------------
    out.append("<h1>PACE RACER — bench characterization figures</h1>")
    out.append("<p class='sub'>ESP32-S3 + DRV8353, 48 V bus, NineBot S hub motor (15 pp), "
               "MT6701 encoder through 2.5:1 gearing, magnetic-particle-brake dyno. "
               "Campaign 2026-08-19/20, extended 2026-08-26/27 with the high-power chase "
               "(§5) and the constant-speed load-step run (§2.1). Figs 1-4 are captured "
               "by the on-board 20 kHz FOC task via the <code>cap</code> ring buffer; "
               "the 08-26/27 runs are host-logged. All loads are reported as measured shaft torque "
               "(N·m) from the inline rotary sensor.</p>")
    tiles = []
    if lf7:
        pk = max(lf7["log"], key=lambda r: r["bus_w"])
        eff1kw = pk["torque_nm"] * pk["rpm"] * 2 * math.pi / 60 / pk["bus_w"] * 100
        tiles.append(tile(f"{lf7['peak_w']:.0f} W", "peak bus power, 483 rpm (08-27)"))
        tiles.append(tile(f"{eff1kw:.0f}%", "system efficiency at the 1 kW point"))
    if th8:
        import statistics as _st
        _hr = [r["rpm"] for r in th8["series"] if r["phase"] == "hold" and r["rpm"]]
        tiles.append(tile(f"±{_st.stdev(_hr):.1f} rpm", "speed hold across 2-10 N·m load steps"))
    if rl:
        tiles.append(tile(f"{rl['R']*2:.3f} Ω", "R line-line, fit (DMM: 0.323)"))
    if jest:
        tiles.append(tile(f"{jest['J']*1000:.0f} m·kg·m²", "inertia J (drag-inclusive)"))
    if f2:
        tiles.append(tile("10 rpm", "worst dip on a 2.6 N·m torque step"))
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

    # ---------------- 2. speed regulation under load ----------------
    if th8:
        out.append(sec("2 · Speed regulation under load",
                       "One question, asked three ways across the campaign: does the wheel "
                       "hold its speed when the load changes? Headline run 2026-08-27: a "
                       "400 rpm hall-commutated hold while the load staircases through "
                       "2 → 4 → 6 → 8 → 10 → 8 → 6 → 4 → 2 N·m. Each setpoint is "
                       "closed-loop on the torque sensor itself (±0.4 N·m acceptance) — "
                       "the brake is only the actuator, so the data is entirely in "
                       "measured N·m."))
        out.append("<h3>2.1 · The load staircase at 400 rpm</h3>")
        ser = th8["series"]
        ts8 = [r["t"] for r in ser]
        out.append(line_chart("f8-tq", "Measured shaft torque vs setpoint",
                              ts8,
                              [("setpoint (N·m)", "--series-3",
                                [r["setpoint_nm"] for r in ser]),
                               ("measured torque (N·m)", "--series-1",
                                [round(r["torque_nm"], 2) if r["torque_nm"] is not None else 0.0
                                 for r in ser])],
                              "time (s)", "torque (N·m)"))
        vl8 = [(m["t"], f"{m['setpoint_nm']:.0f}") for m in th8["marks"]]
        out.append(line_chart("f8-rpm", "Speed through every load step (labels: N·m setpoints)",
                              ts8,
                              [("rotor rpm", "--series-2", [r["rpm"] for r in ser])],
                              "time (s)", "rotor speed (rpm)",
                              ylo=390, yhi=410, hlines=[(400, "setpoint 400 rpm")],
                              vlines=vl8))
        visits, seen = [], None
        for r in ser:
            if r["phase"] != "hold":
                seen = None
                continue
            if seen != r["setpoint_nm"]:
                visits.append({"sp": r["setpoint_nm"], "tq": [], "w": [], "rpm": []})
                seen = r["setpoint_nm"]
            v = visits[-1]
            if r["torque_nm"] is not None: v["tq"].append(r["torque_nm"])
            v["w"].append(r["bus_w"]); v["rpm"].append(r["rpm"])
        rows8 = "".join(
            f"<tr><td>{v['sp']:.0f}</td><td>{sum(v['tq'])/len(v['tq']):.2f}</td>"
            f"<td>{sum(v['w'])/len(v['w']):.0f}</td>"
            f"<td>{sum(v['rpm'])/len(v['rpm']):.1f}</td></tr>" for v in visits if v["tq"])
        out.append("<div class='scroll'><table><thead><tr><th>setpoint (N·m)</th>"
                   "<th>held mean (N·m)</th><th>bus (W)</th><th>rpm mean</th></tr></thead>"
                   f"<tbody>{rows8}</tbody></table></div>")
        import statistics as _st
        hr = [r["rpm"] for r in ser if r["phase"] == "hold" and r["rpm"]]
        out.append(f"<p><b>Result: {_st.mean(hr):.1f} ± {_st.stdev(hr):.2f} rpm across the "
                   "entire 2→10→2 N·m staircase</b> (worst single sample 1 rpm off "
                   "setpoint), bus power tracking 76→536→93 W. Setpoints are reached "
                   "from both directions — the down-leg exercises the brake's remanence "
                   "hysteresis, which the torque servo absorbs invisibly. Held means sit "
                   "within ±0.5 N·m of target; the residual is remanence creep during "
                   "each dwell, visible as the slow rise inside each torque step.</p>")
        # ---- 2.2 transient detail (250 Hz on-board capture) ----
    if f2:
        out.append("<h3>2.2 · Transient detail — a 2.6 N·m step at 250 Hz</h3>"
                   "<p class='sub'>The staircase above is host-sampled at ~2 Hz, which "
                   "hides the transient. This on-board 250 Hz capture (100 rpm, encoder "
                   "commutation) resolves what a step actually does to the loop.</p>")
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

    if f3:
        _r3 = [r[1] for r in f3["rows"]]
        _mid = _r3[int(3/f3["dt"]):int(20/f3["dt"])]
        _m3 = sum(_mid)/len(_mid)
        _sd3 = (sum((x-_m3)**2 for x in _mid)/len(_mid))**0.5
        out.append(f"<p>A third variant — a continuously sweeping sinusoidal load "
                   f"(≈0.3-2 N·m, 10 s period) at 100 rpm — held {_m3:.0f} ± {_sd3:.1f} rpm "
                   "and adds nothing the staircase and the step don't already show; its "
                   "traces live in the run archive.</p>")

    # ---------------- 3. torque linearity ----------------
    SCALE = 10.0  # 10:1 probe on a 1x channel — confirmed against the sensor display
    if f5:
        out.append(sec("3 · Torque vs current — encoder commutation is linear",
                       "eiq staircase with the rotor held by the particle brake, torque read from the "
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

    # ---------------- 5. power and efficiency ----------------
    if f6hold and lc7 and lf7:
        out.append(sec("5 · Power and efficiency — three speeds, one hardware",
                       "The high-power campaign in one frame: brake ladders at 150 rpm "
                       "(2026-08-19), 400 rpm (08-26) and 500 rpm (08-27), each holding "
                       "speed while the load rises. Bus power from the 48 V supply meter, "
                       "shaft torque from the inline sensor — two independent instruments, "
                       "so every efficiency here is a true end-to-end measurement."))
        EF = lambda tq, rpm, w: tq * rpm * 2 * math.pi / 60 / w * 100
        p150 = sorted((r["torque_nm"], r["bus_w"], r["rpm"], r["tmax"]) for r in f6hold["log"]
                      if r.get("torque_nm") is not None and r["rpm"] > 120 and r["bus_w"] > 50)
        p400 = sorted((r["torque_nm"], r["bus_w"], r["rpm"], r["tmax"]) for r in lc7["log"]
                      if r.get("torque_nm") is not None and r["bus_w"] > 50)
        p500 = sorted((r["torque_nm"], r["bus_w"], r["rpm"], r["tmax"]) for r in lf7["log"]
                      if r.get("torque_nm") is not None and r["bus_w"] > 50)
        out.append("<h3>5.1 · What torque buys at each speed</h3>"
                   "<p>Same x-axis, three speeds. Power is T·ω, so the same shaft torque "
                   "buys 3.3× the power at 500 rpm that it buys at 150 — and the drive "
                   "reaches 1 kW at 16 N·m instead of needing an impossible 60+.</p>")
        out.append(multi_xy("p-tq", "Bus power vs shaft torque at 150 / 400 / 500 rpm",
                            [("150 rpm", "--series-3", [(a, b) for a, b, _, _ in p150]),
                             ("400 rpm", "--series-2", [(a, b) for a, b, _, _ in p400]),
                             ("500 rpm", "--series-1", [(a, b) for a, b, _, _ in p500])],
                            "shaft torque (N·m)", "bus power (W)",
                            hlines=[(1000, "1 kW target")]))
        out.append(multi_xy("eff-tq", "System efficiency vs shaft torque at 150 / 400 / 500 rpm",
                            [("150 rpm", "--series-3", [(a, EF(a, c, b)) for a, b, c, _ in p150]),
                             ("400 rpm", "--series-2", [(a, EF(a, c, b)) for a, b, c, _ in p400]),
                             ("500 rpm", "--series-1", [(a, EF(a, c, b)) for a, b, c, _ in p500])],
                            "shaft torque (N·m)", "efficiency (%)", ylo=30, yhi=90))
        out.append("<p>The efficiency chart is the same physics from the loss side: "
                   "copper heat is R·I² and torque costs current, so at fixed speed "
                   "efficiency falls as torque rises — but raising speed lifts the whole "
                   "curve, because the numerator (T·ω) grows while the copper bill "
                   "stays put. At 150 rpm the drive peaked at 52% efficiency; the same "
                   "16-17 N·m delivered 73% at 400 rpm and <b>78% at the 1 kW point</b>.</p>")

        out.append("<h3>5.2 · Where the power goes at 1 kW</h3>")
        pk = max(lf7["log"], key=lambda r: r["bus_w"])
        mech_pk = pk["torque_nm"] * pk["rpm"] * 2 * math.pi / 60
        copper_pk = 1.5 * 0.166 * pk["iq"] ** 2
        drive_pk = pk["bus_w"] - mech_pk - copper_pk
        out.append(pie_chart("pie-1kw", "Power budget at the 1049 W operating point",
                             [("mechanical output", mech_pk, "--series-1"),
                              ("motor copper heat", copper_pk, "--series-2"),
                              ("drive + iron/friction", drive_pk, "--series-3")]))
        corners = [("150 rpm (08-19)", p150[-1], "30 A torque ceiling"),
                   ("400 rpm (08-26)", p400[-1], "VDS-OCP derating (§5.3)"),
                   ("500 rpm (08-27)", p500[-1], "none — target reached")]
        rowsc = "".join(
            f"<tr><td>{n}</td><td>{b:.0f}</td><td>{a*c*2*math.pi/60:.0f}</td>"
            f"<td>{b - a*c*2*math.pi/60:.0f}</td><td>{EF(a, c, b):.0f}%</td><td>{lim}</td></tr>"
            for n, (a, b, c, _), lim in corners)
        out.append("<div class='scroll'><table><thead><tr><th>operating corner</th>"
                   "<th>bus (W)</th><th>mechanical (W)</th><th>losses (W)</th>"
                   "<th>efficiency</th><th>what limited it</th></tr></thead>"
                   f"<tbody>{rowsc}</tbody></table></div>")
        out.append("<p>Motor copper is 1.5·R·i<sub>q</sub>² with the measured R — heat in "
                   "the <i>windings</i>, not the board; the drive electronics burn only "
                   "~10-20 W throughout, with iron and friction growing with speed. The "
                   "150 rpm corner fed 514 W in for 267 W out because torque-at-low-speed "
                   "is inherently copper-dominated; it also ended benignly, with the 35 A "
                   "software guard unloading cleanly when the speed loop chased a brake "
                   "fluctuation — protection by design, no hardware fault.</p>")

        out.append("<h3>5.3 · The wall at 400 rpm, and how it fell</h3>")
        out.append("<p><b>First, the guard had to be trustworthy.</b> The 08-20 campaign's "
                   "intermittent over-current aborts reported 49-52 A on phase C, but a "
                   "100 A current clamp on that phase, armed to trigger at 35 A, never "
                   "fired — reproduced three times. The spikes are conversion artifacts, "
                   "not current (readable past the ADC's +51/−39 A rail only because the "
                   "reconstructed phase can register 2× rail). The software trip moved "
                   "35→45 A; every ladder afterwards logged zero phantom trips while the "
                   "clamp confirmed real peaks ≤29 A. (At 45 A the trip is blind to "
                   "negative excursions on the measured pair — the DRV VDS OCP remains "
                   "the hardware layer.)</p>")
        wall = sorted((r["bus_w"], r["clamp_peak_a"], r["tmax"]) for r in lc7["log"])
        out.append(line_chart("f7-wall", "The derating wall at 400 rpm: real peaks and board temp vs power",
                              [round(w) for w, _, _ in wall],
                              [("phase-C peak, clamp (A)", "--series-2",
                                [round(c, 1) for _, c, _ in wall]),
                               ("hottest board sensor (°C)", "--series-1",
                                [round(tm, 1) for _, _, tm in wall])],
                              "bus power (W)", "A · °C"))
        out.append("<p>Three attempts at 400 rpm ended between 790 and 950 W with real "
                   "DRV8353 VDS-OCP faults — different phase pairs, board temperatures "
                   "from 44 to 60 °C, one on a coarse ~2 N·m load step and one on a "
                   "gentle ~0.7 N·m increment from a steady 847.7 W. The pattern is the "
                   "0.20 V VDS threshold (≈58-74 A cold) derating with hot R<sub>dson</sub> "
                   "into legitimate ~29 A peaks. A cold start did not clear it, which "
                   "killed the thermal-headroom theory and left two levers: raise the "
                   "threshold, or need less current.</p>"
                   "<p><b>The speed route won.</b> The firmware's speed-command clamp went "
                   "±400 → ±520 rpm (bemf ~21 V + IR ~3 V fits the 26 V SVPWM ceiling), "
                   "and a 500 rpm ladder walked through the fault band at ~21 A peaks "
                   "where 400 rpm needed 29. The 0.30 V VDS level was exposed on the "
                   "console as a fallback and never used.</p>")
        rowsf = "".join(
            f"<tr><td>{r['torque_nm']:.1f}</td><td>{r['bus_w']:.0f}</td>"
            f"<td>{r['rpm']:.0f}</td><td>{EF(r['torque_nm'], r['rpm'], r['bus_w']):.0f}%</td>"
            f"<td>{r['clamp_peak_a'] or '—'}</td><td>{r['tmax']:.0f}</td></tr>"
            for r in lf7["log"])
        out.append("<div class='scroll'><table><thead><tr><th>N·m</th><th>bus W</th>"
                   "<th>rpm</th><th>system eff</th><th>clamp peak (A)</th><th>board °C</th>"
                   f"</tr></thead><tbody>{rowsf}</tbody></table></div>")
        out.append(f"<p><b>Result: {pk['bus_w']:.0f} W bus at {pk['rpm']:.0f} rpm, "
                   f"{pk['torque_nm']:.1f} N·m — {EF(pk['torque_nm'], pk['rpm'], pk['bus_w']):.0f}% "
                   "system efficiency, no fault during the run, stock protection "
                   "throughout.</b> Two engineering footnotes. A VDS fault did fire "
                   "<i>after</i> the run while stopping the motor into the still-energized "
                   "brake — shutdown order is now brake-release first, motor stop second. "
                   "And the sampler's <code>late</code> fraction rises from 0.1% at "
                   "≤400 rpm to a steady 4.9% at 500 rpm (max latency still ≤46 µs, no "
                   "overruns): fine today, but it soft-bounds the next speed increment. "
                   "Control-loop timing is otherwise exonerated — every loaded rung at "
                   "every speed logged late ≤0.1% (at ≤400 rpm) and cmax ≤57 µs against "
                   "the 50 µs PWM period; an earlier \"late stuck at 5%\" scare was a "
                   "counter-semantics misread (the stats reset on read).</p>")

        out.append("<h3>5.4 · Board temperature vs power</h3>"
                   "<p>Transient temperatures during 1-2 min ladders, not steady state "
                   "(§4 shows the board needs 2-4 min to plateau). The picture is "
                   "consistent across speeds: ~30 °C/kW of bus power transiently, with "
                   "the §4 thermal resistance putting steady state near 60-65 °C at "
                   "500 W in a 25 °C room — inside the 80 °C guard.</p>")
        out.append(multi_xy("tmp-p", "Hottest board sensor vs bus power at 150 / 400 / 500 rpm",
                            [("150 rpm", "--series-3", [(b, d) for _, b, _, d in p150]),
                             ("400 rpm", "--series-2", [(b, d) for _, b, _, d in p400]),
                             ("500 rpm", "--series-1", [(b, d) for _, b, _, d in p500])],
                            "bus power (W)", "board temp (°C)", ylo=25))

        out.append("<h3>5.5 · The tested operating envelope</h3>"
                   "<p>Every validated operating point of the campaign on the "
                   "torque-speed plane, against mechanical iso-power curves. Filled "
                   "points carry sensor-measured torque; rings are K<sub>t</sub>·i<sub>q</sub> "
                   "estimates from runs before the torque readout existed.</p>")
        meas_pts = []
        if f5:
            meas_pts += [(0.0, s["torque_raw"] * SCALE) for s in f5["steps"]
                         if s.get("torque_raw") is not None and s["iq_meas"] > 2]
        meas_pts += [(c, a) for a, _, c, _ in p150 + p400 + p500]
        if th8:
            meas_pts += [(r["rpm"], r["torque_nm"]) for r in th8["series"]
                         if r["phase"] == "hold" and r.get("torque_nm") and r.get("rpm")]
        derived_pts = [(100, 8.5 * 0.62), (100, 2.7 * 0.62),   # §2 disturbance runs
                       (60, 6.4 / 2.9 * 1.0), (50, 2.5)]       # §1 drive tests
        out.append(envelope_chart("f6-env", "Validated operating points, torque vs speed",
                                  meas_pts, derived_pts, [50, 250, 500, 1000], xmax=560))
        out.append("<p>Iso-lines are <i>mechanical</i> power. The stall column at 0 rpm "
                   "is §3's staircase; the dense 400 rpm column at 2-10 N·m is §2's "
                   "load-step staircase; the 150 rpm column tops out against the 30 A "
                   "torque ceiling; the 400 rpm column against VDS derating (§5.3); and "
                   "the 500 rpm column crosses the 1000 W-bus operating point at 817 W "
                   "mechanical. Unexplored: speeds past 520 rpm (the present command "
                   "clamp, soft-bounded by sampler timing) and sustained dwells at the "
                   "top corner.</p>")

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
               "<li><b>Phantom over-current, closed (08-26):</b> the intermittent 49-52 A "
               "phase-C trip reports were proven phantom against a 100 A clamp (never "
               "above 29 A real, 3\u00d7 reproduced); soft trip raised 35\u219245 A, zero phantom "
               "trips since. The gate-drive scope session happened: ~322 ns dead-time, "
               "zero overlap samples, no shoot-through.</li>"
               "<li><b>VDS OCP derating is the current power wall (08-26):</b> two real "
               "0x0628 faults (high-side A+B) at 850-950 W attempts with the board at "
               "57-60 \u00b0C \u2014 the 0.20 V threshold\u2019s amp value falls with hot Rdson into "
               "real ~29 A peaks. A cold start did NOT clear it (third fault at 44 \u00b0C) \u2014 resolved by the 500 rpm speed route instead; see \u00a75.3.</li>"
               "<li><b>Shutdown order matters:</b> stopping the motor while the brake is "
               "still energized fired a VDS fault after an otherwise clean 1 kW run "
               "(hard decel + back-EMF). The harness now releases the brake, waits, "
               "then stops the motor.</li>"
               "<li><b>Sampler timing at speed (watch item):</b> <code>late</code> is "
               "0.1% at \u2264400 rpm but a steady 4.9% at 500 rpm (cmax \u226446 \u00b5s, no "
               "overruns). Fine today; bounds the next speed increment.</li>"
               "<li><b>Firmware 2026-08-27:</b> speed-command clamp \u00b1400 \u2192 \u00b1520 rpm; "
               "DRV VDS level 6 (0.30 V) exposed on the console but unused \u2014 the 1 kW "
               "point was reached with the stock 0.20 V threshold.</li>"
               "<li><b>Stats counters reset on read:</b> <code>s</code>/<code>sq</code> "
               "print-and-zero; a single read after boot mixes cal/arm work into the "
               "late percentage. Take a throwaway read, wait, read again.</li>"
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
