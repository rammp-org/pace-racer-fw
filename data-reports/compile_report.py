#!/usr/bin/env python3
"""Compile the halls-sensorless test-campaign logs into a shareable HTML report.

Zero dependencies: reads logs/run-*/{data.csv,events.log}, renders inline-SVG
charts (2px lines, hairline grid, crosshair tooltips), and writes a single
self-contained file. Print it from the browser for a PDF.

  python3 compile_report.py                  # -> report/index.html (standalone)
  python3 compile_report.py --fragment PATH  # body-only fragment (for hosting
                                             # wrappers that supply <head>)

Narrative sections are curated for the 2026-08 low-speed/high-torque campaign;
the extraction and chart helpers are generic. Manual bench readings (torque
display) are constants below with provenance comments.
"""
import argparse
import csv
import html
import json
from datetime import datetime
from pathlib import Path

APP = Path(__file__).resolve().parent
LOGS = APP / "logs"

# ---------------------------------------------------------------- bench data
# Torque display readings called out by the operator during the 2026-08-06
# 14:56 stall staircase (run-20260806-145606-agent), brake CH2 = 2.50 V.
TORQUE_DISPLAY = [(10, 6.4), (12, 7.5), (14, 8.6), (16, 7.0), (18, 5.2), (20, 6.0)]
KT_ALIGNED = 0.62  # Nm/A measured on the aligned steps (10-14 A)

# Control-loop timing across configs (ETHERNET_FOC_COEXISTENCE.md §14 + the
# 2026-08-06 12:23 session with MCPWM intr_priority=2).
TIMING = [
    ("16 KB icache",            4.9, (207, 239), (19.0, 22.0)),
    ("32 KB icache",            4.9, (59, 73),   (12.5, 16.3)),
    ("+ FOC task in IRAM",      4.9, (36, 39),   (11.3, 13.9)),
    ("+ MCPWM intr_priority 2", 0.0, (29, 38),   (7.2, 12.9)),
]

FAULTS = [
    ("0x0622 / 0x0628", "VDS OCP, high-side A+C / A+B",
     "Sensorless I/f pole-slip and observer glitches under load put the "
     "voltage vector at a wrong angle; phase A saw 48 A", "2026-08-06 morning"),
    ("0x0604 / 0x0608", "VDS OCP, low-side B",
     "Sustained 24-25 A stalled DC in heat-soaked FETs; VDS threshold derates "
     "with Rdson — the protection working at its designed boundary", "2026-08-06 / 08-17"),
    ("chip wedge", "USB-serial-JTAG dead, esptool cannot connect",
     "Correlates with the corrupted-angle OCP transient; only a VM power-cycle "
     "recovers it. Open board-level item since 07-30", "2026-08-06"),
]


# ------------------------------------------------------------- log ingestion
def read_run(name):
    """Rows of data.csv for a run dir, host_time parsed, numerics floated."""
    p = LOGS / name / "data.csv"
    if not p.exists():
        return []
    out = []
    with open(p) as f:
        for r in csv.DictReader(f):
            try:
                row = {"host": r["host_time"], "mode": r["mode"]}
                for k, v in r.items():
                    if k not in ("host_time", "mode"):
                        try:
                            row[k] = float(v)
                        except (ValueError, TypeError):
                            row[k] = float("nan")
                out.append(row)
            except KeyError:
                continue
    return out


def window(rows, h0, h1):
    return [r for r in rows if h0 <= r["host"][11:22] <= h1]


def run_index():
    """Appendix table: every run dir with first event + row count."""
    items = []
    for d in sorted(LOGS.iterdir()) if LOGS.exists() else []:
        if not d.is_dir() or not d.name.startswith("run-"):
            continue
        n = 0
        p = d / "data.csv"
        if p.exists():
            with open(p) as f:
                n = max(sum(1 for _ in f) - 1, 0)
        first = ""
        ev = d / "events.log"
        if ev.exists():
            with open(ev) as f:
                for line in f:
                    if " " in line:
                        first = line.split(" ", 1)[1].strip()
                        first = first.split(", logs=")[0]  # drop the repeated path
                        break
        items.append((d.name, n, first[:90]))
    return items


# ------------------------------------------------------------- SVG charting
W, H, ML, MR, MT, MB = 860, 316, 74, 16, 14, 52


def _scale(vals, lo=None, hi=None):
    lo = min(vals) if lo is None else lo
    hi = max(vals) if hi is None else hi
    if hi - lo < 1e-9:
        hi = lo + 1.0
    return lo, hi


def _ticks(lo, hi, n=5):
    import math
    raw = (hi - lo) / n
    mag = 10 ** math.floor(math.log10(raw)) if raw > 0 else 1
    step = min((s for s in (1, 2, 2.5, 5, 10) if s * mag >= raw), default=10) * mag
    t0 = math.ceil(lo / step) * step
    out = []
    t = t0
    while t <= hi + 1e-9:
        out.append(round(t, 6))
        t += step
    return out


def _fmt(v):
    if abs(v) >= 1000:
        return f"{v:,.0f}"
    if abs(v - round(v)) < 1e-6:
        return f"{int(round(v))}"
    return f"{v:.2f}".rstrip("0").rstrip(".")


def line_chart(cid, title, x, series, xlabel, ylabel, notes=(), ylo=None, yhi=None,
               hlines=(), vlines=()):
    """series: [(name, cssvar, values)]. Crosshair tooltip via shared JS.
    hlines/vlines: [(value, label)] dashed reference lines — setpoints on y,
    event times on x."""
    xlo, xhi = _scale(x)
    allv = [v for _, _, vals in series for v in vals] + [v for v, _ in hlines]
    ylo_, yhi_ = _scale(allv, ylo, yhi)
    pad = (yhi_ - ylo_) * 0.08
    ylo_, yhi_ = ylo_ - pad, yhi_ + pad
    pw, ph = W - ML - MR, H - MT - MB
    sx = lambda v: ML + (v - xlo) / (xhi - xlo) * pw
    sy = lambda v: MT + ph - (v - ylo_) / (yhi_ - ylo_) * ph

    g = []
    for t in _ticks(ylo_, yhi_):
        g.append(f'<line x1="{ML}" y1="{sy(t):.1f}" x2="{W-MR}" y2="{sy(t):.1f}" class="grid"/>')
        g.append(f'<text x="{ML-8}" y="{sy(t):.1f}" class="tick" text-anchor="end" dy="0.32em">{_fmt(t)}</text>')
    for t in _ticks(xlo, xhi, 7):
        g.append(f'<text x="{sx(t):.1f}" y="{MT+ph+18}" class="tick" text-anchor="middle">{_fmt(t)}</text>')
    g.append(f'<line x1="{ML}" y1="{MT+ph}" x2="{W-MR}" y2="{MT+ph}" class="axis"/>')
    # axis titles: y rotated along the left edge, x centered below the ticks
    g.append(f'<text class="axis-label" transform="rotate(-90 14 {MT+ph/2:.0f})" x="14" '
             f'y="{MT+ph/2:.0f}" text-anchor="middle">{html.escape(ylabel)}</text>')
    g.append(f'<text class="axis-label" x="{ML+pw/2:.0f}" y="{H-8}" text-anchor="middle">'
             f'{html.escape(xlabel)}</text>')
    # dashed reference lines
    for v, lbl in hlines:
        g.append(f'<line x1="{ML}" y1="{sy(v):.1f}" x2="{W-MR}" y2="{sy(v):.1f}" class="ref"/>')
        g.append(f'<text x="{W-MR}" y="{sy(v)-5:.1f}" class="note" text-anchor="end">{html.escape(lbl)}</text>')
    for v, lbl in vlines:
        g.append(f'<line x1="{sx(v):.1f}" y1="{MT}" x2="{sx(v):.1f}" y2="{MT+ph}" class="ref"/>')
        g.append(f'<text x="{sx(v)+5:.1f}" y="{MT+12}" class="note" text-anchor="start">{html.escape(lbl)}</text>')

    for name, var, vals in series:
        pts = " ".join(f"{sx(a):.1f},{sy(b):.1f}" for a, b in zip(x, vals))
        g.append(f'<polyline points="{pts}" fill="none" stroke="var({var})" '
                 f'stroke-width="2" stroke-linejoin="round" stroke-linecap="round"/>')
        # end marker + direct label
        ex, ey = sx(x[-1]), sy(vals[-1])
        g.append(f'<circle cx="{ex:.1f}" cy="{ey:.1f}" r="4" fill="var({var})" '
                 f'stroke="var(--surface-1)" stroke-width="2"/>')
    for nx, ny, txt, anchor in notes:
        g.append(f'<text x="{sx(nx):.1f}" y="{sy(ny):.1f}" class="note" text-anchor="{anchor}">{html.escape(txt)}</text>')
    g.append(f'<line class="xh" x1="0" y1="{MT}" x2="0" y2="{MT+ph}" style="display:none"/>')

    data = {"type": "line", "x": x, "xlo": xlo, "xhi": xhi, "ml": ML, "pw": pw,
            "series": [{"n": n, "v": v, "c": c} for n, c, v in series],
            "xlabel": xlabel, "ylabel": ylabel}
    legend = "".join(
        f'<span class="key"><span class="key-line" style="background:var({c})"></span>{html.escape(n)}</span>'
        for n, c, _ in series) if len(series) > 1 else ""
    tbl = table_view([xlabel] + [n for n, _, _ in series],
                     [[_fmt(a)] + [_fmt(s[2][i]) for s in series] for i, a in enumerate(x)][::max(1, len(x)//40)])
    return figure(cid, title, ylabel, legend, g, data, tbl)


def bar_chart(cid, title, cats, vals, ylabel, cssvar="--series-1", refline=None, sub=None):
    xlo, xhi = 0, len(cats)
    ylo_, yhi_ = _scale(vals + ([refline[0]] if refline else []), 0)
    yhi_ *= 1.15
    pw, ph = W - ML - MR, H - MT - MB
    bw = min(24, pw / len(cats) * 0.55)
    sy = lambda v: MT + ph - (v - ylo_) / (yhi_ - ylo_) * ph
    g = []
    for t in _ticks(ylo_, yhi_):
        g.append(f'<line x1="{ML}" y1="{sy(t):.1f}" x2="{W-MR}" y2="{sy(t):.1f}" class="grid"/>')
        g.append(f'<text x="{ML-8}" y="{sy(t):.1f}" class="tick" text-anchor="end" dy="0.32em">{_fmt(t)}</text>')
    for i, (c, v) in enumerate(zip(cats, vals)):
        cx = ML + (i + 0.5) / len(cats) * pw
        y = sy(v)
        hgt = MT + ph - y
        g.append(f'<path class="bar" data-i="{i}" d="M{cx-bw/2:.1f},{MT+ph:.1f} v-{max(hgt-4,0):.1f} '
                 f'q0,-4 4,-4 h{bw-8:.1f} q4,0 4,4 v{max(hgt-4,0):.1f} z" fill="var({cssvar})"/>')
        g.append(f'<text x="{cx:.1f}" y="{y-7:.1f}" class="val" text-anchor="middle">{_fmt(v)}</text>')
        g.append(f'<text x="{cx:.1f}" y="{MT+ph+18:.1f}" class="tick" text-anchor="middle">{html.escape(str(c))}</text>')
    if refline:
        rv, rl = refline
        g.append(f'<line x1="{ML}" y1="{sy(rv):.1f}" x2="{W-MR}" y2="{sy(rv):.1f}" class="ref"/>')
        g.append(f'<text x="{W-MR}" y="{sy(rv)-6:.1f}" class="note" text-anchor="end">{html.escape(rl)}</text>')
    g.append(f'<line x1="{ML}" y1="{MT+ph}" x2="{W-MR}" y2="{MT+ph}" class="axis"/>')
    g.append(f'<text class="axis-label" transform="rotate(-90 14 {MT+ph/2:.0f})" x="14" '
             f'y="{MT+ph/2:.0f}" text-anchor="middle">{html.escape(ylabel)}</text>')
    if sub:
        g.append(f'<text class="axis-label" x="{ML+pw/2:.0f}" y="{H-8}" text-anchor="middle">'
                 f'{html.escape(str(sub))}</text>')
    data = {"type": "bar", "cats": [str(c) for c in cats], "vals": vals, "ylabel": ylabel}
    tbl = table_view([sub or "step", ylabel], [[str(c), _fmt(v)] for c, v in zip(cats, vals)])
    return figure(cid, title, ylabel, "", g, data, tbl)


def figure(cid, title, ylabel, legend, body, data, tbl):
    return (
        f'<figure class="viz" id="{cid}">'
        f'<figcaption><span class="fig-title">{html.escape(title)}</span>{legend}</figcaption>'
        f'<div class="viz-wrap"><svg viewBox="0 0 {W} {H}" role="img" '
        f'aria-label="{html.escape(title)}">{"".join(body)}</svg>'
        f'<div class="tip" hidden></div></div>'
        f'<script type="application/json" class="viz-data">{json.dumps(data)}</script>'
        f"{tbl}</figure>")


def table_view(head, rows):
    h = "".join(f"<th>{html.escape(str(c))}</th>" for c in head)
    b = "".join("<tr>" + "".join(f"<td>{html.escape(str(c))}</td>" for c in r) + "</tr>" for r in rows)
    return (f'<details class="tblview"><summary>Data table</summary>'
            f'<div class="scroll"><table><thead><tr>{h}</tr></thead><tbody>{b}</tbody></table></div></details>')


# ------------------------------------------------------------------ assembly
def build_body():
    s = []
    now = datetime.now().strftime("%Y-%m-%d")

    s.append(f"""
<header class="rpt-head">
  <p class="eyebrow">PACE RACER · halls-sensorless · dyno campaign 2026-08-06 → 08-17</p>
  <h1>Low-speed / high-torque drive: what the board can do, and why we're moving to an encoder</h1>
  <p class="lede">Goal: confidence that this controller can start a wheelchair's main wheel from
  standstill under high load and drive it smoothly at low speed. Bench: NineBot&nbsp;S hub motor,
  magnetic-particle brake, rotary torque sensor, 48&nbsp;V bus. Compiled {now} from the
  <code>logs/</code> run archive.</p>
</header>

<section class="tiles">
  <div class="tile"><span class="t-label">Sustained drive under load</span><span class="t-value">126 W</span><span class="t-delta up">previous practical wall ~100 W</span></div>
  <div class="tile"><span class="t-label">Stall current, clean tracking</span><span class="t-value">22 A</span><span class="t-delta">iq error &lt; 0.1 A; VDS ceiling ~24 A</span></div>
  <div class="tile"><span class="t-label">Sampler late entries</span><span class="t-value">0.0%</span><span class="t-delta up">was 4.9% before MCPWM priority fix</span></div>
  <div class="tile"><span class="t-label">Measured torque constant</span><span class="t-value">0.62 <span class="unit">N·m/A</span></span><span class="t-delta">matches λ-derived 0.59</span></div>
</section>

<section>
  <h2><span class="secno">1</span>Control-loop timing is solved</h2>
  <p>Four configurations, measured on the same firmware. Cache and IRAM cut how long the loop
  <em>takes</em>; only the MCPWM interrupt priority (a local espp patch, <code>intr_priority=2</code> —
  priority&nbsp;3 starved the console) fixed how late it <em>starts</em>. The control loop now fits
  comfortably inside its 50&nbsp;µs period with interrupt entry latency eliminated as a variable.</p>
""")
    s.append(bar_chart("c-cmax", "Worst-case FOC compute time by configuration",
                       [t[0] for t in TIMING], [t[2][1] for t in TIMING],
                       "cmax, µs (worst of range)", "--series-1",
                       refline=(50, "50 µs control period"), sub="configuration"))
    s.append(bar_chart("c-late", "Sampler late entries by configuration",
                       [t[0] for t in TIMING], [t[1] for t in TIMING],
                       "late, %", "--series-2", sub="configuration"))
    s.append("""
  <div class="scroll"><table><thead><tr><th>configuration</th><th>late %</th><th>cmax µs</th><th>isr_max µs</th></tr></thead><tbody>""")
    for name, late, cmax, isr in TIMING:
        s.append(f"<tr><td>{html.escape(name)}</td><td>{late}</td><td>{cmax[0]}–{cmax[1]}</td><td>{isr[0]}–{isr[1]}</td></tr>")
    s.append("</tbody></table></div></section>")

    # --- why the wall existed
    r1908 = read_run("run-20260806-121908")
    s.append("""
<section>
  <h2><span class="secno">2</span>Why the ~100 W wall existed: sensorless can't do this regime</h2>
  <p>Every high-power attempt in the sensorless modes died the same two deaths. In I/f startup the
  load angle walks to ~177° under brake torque and the rotor pole-slips (<code>run 20 50 4</code>:
  slip at 18.8&nbsp;A → current spike → VDS OCP). In closed-loop observer mode at 80&nbsp;rpm the
  back-EMF is ~3&nbsp;V — one observer glitch put the voltage vector at a wrong angle and phase&nbsp;A
  saw 48&nbsp;A. Below is the moment it happens: the observer's speed estimate diverges from the
  halls' ground truth in ~100&nbsp;ms while the d-axis current spikes to −8.9&nbsp;A.</p>
""")
    if r1908:
        win = window(r1908, "12:20:43", "12:20:49")
        if win:
            t0 = win[0]["t"]
            x = [round(r["t"] - t0, 2) for r in win]
            s.append(line_chart(
                "c-obs", "Observer glitch under load — run 10 80 10, 2026-08-06 12:20",
                x, [("rpm (observer)", "--series-2", [r["rpm_est"] for r in win]),
                    ("rpm (halls)", "--series-1", [r["rpm_hall"] for r in win])],
                "seconds", "mechanical rpm",
                notes=[(3.2, 215, "estimate diverges; id spikes −8.9 A; DRV VDS OCP", "start")]))
    s.append("""
  <div class="scroll"><table><thead><tr><th>fault signature</th><th>decode</th><th>root cause</th><th>when</th></tr></thead><tbody>""")
    for code, decode, cause, when in FAULTS:
        s.append(f"<tr><td><code>{html.escape(code)}</code></td><td>{html.escape(decode)}</td>"
                 f"<td>{html.escape(cause)}</td><td>{html.escape(when)}</td></tr>")
    s.append("""</tbody></table></div>
  <p>Verdict: at wheelchair speeds (0–60 rpm) the observer has almost no signal and I/f has no
  angle feedback. This is physics, not tuning — the hall/encoder path is the correct architecture
  for this regime, which is why the campaign moved there.</p>
</section>""")

    # --- fixes
    r2355 = read_run("run-20260806-122355")
    s.append("""
<section>
  <h2><span class="secno">3</span>Firmware fixes shipped during the campaign</h2>
  <p><strong>Current-loop integrator windup at coast.</strong> After any stop, the loop kept
  integrating ADC noise against a Hi-Z bridge with no anti-windup: both integrators railed at
  −20&nbsp;V within ~8&nbsp;s (the mysterious “345% saturated while parked” readings), and a
  <code>id/iq</code> re-engage would have applied that railed vector as a current bang. Fixed by
  holding the PIs reset while coasting; verified ±0.02&nbsp;V over 16&nbsp;s of coast.</p>
""")
    if r2355:
        win = [r for r in r2355 if 74 <= r["t"] <= 92]
        if win:
            x = [round(r["t"] - 75.5, 2) for r in win]
            s.append(line_chart(
                "c-windup", "Integrator windup after `stop` — before the fix",
                x, [("vd", "--series-1", [r["vd"] for r in win]),
                    ("vq", "--series-2", [r["vq"] for r in win])],
                "seconds after stop", "volts",
                notes=[(9.0, -18.5, "railed at −vlim; after fix: ±0.02 V", "start")]))
    s.append("""
  <p><strong>Hall-table restore (<code>hset</code>).</strong> The calibration table was RAM-only;
  every VM power-cycle lost it, and re-calibrating needs a spin a brake-loaded wheel can't do.
  <code>hs</code> prints the live table; <code>hset</code> re-injects it. The test harness re-arms
  automatically after fault recovery.</p>
  <p><strong>Speed-aware hall resync gate.</strong> At ~22 A of stalled DC, phase-current noise
  biased a hall line long enough to pass debounce; the poller “resynced” to a sector ~3 away, the
  drive angle flipped ~176°, and the transient fired VDS OCP (and wedged the chip). Non-adjacent
  sector jumps are now accepted only within 0.25 s of real motion — while parked, the last
  validated sector is held (it's where the rotor demonstrably is). A <code>rej</code> counter in
  <code>hs</code> makes rejected corruption visible. Board-rev item stands: real 1–4.7 kΩ hall
  pull-ups.</p>
</section>""")

    # --- torque characterization
    s.append("""
<section>
  <h2><span class="secno">4</span>Stall-torque characterization: the hall blind spot, measured</h2>
  <p>Slow <code>hiq</code> staircase at standstill against the brake (CH2 = 2.5&nbsp;V), operator
  reading the torque display. The aligned steps give <strong>K<sub>t</sub> ≈ 0.62 N·m/A</strong>.
  During the 16&nbsp;A step the rotor micro-slipped across a hall boundary; the drive angle can
  only be known to one sector (60° electrical), the rotor kept creeping through the sector's blind
  spot, and torque-per-amp collapsed to ~0.30 — <em>half the torque for the same amps and heat</em>.
  It is self-limiting: degraded torque drops below the brake's grip, the creep stops just short of
  the edge that would correct the angle.</p>
""")
    s.append(bar_chart("c-kt", "Torque per amp across the stall staircase (operator torque display)",
                       [f"{a} A" for a, _ in TORQUE_DISPLAY],
                       [round(t / a, 3) for a, t in TORQUE_DISPLAY],
                       "N·m per A", "--series-1",
                       refline=(KT_ALIGNED, "aligned Kt 0.62"), sub="iq step"))
    s.append("""
  <p>Current control itself was flawless throughout — iq tracked its reference within 0.1&nbsp;A at
  every step to 22&nbsp;A, and the FETs (60&nbsp;°C) were never the limit below the VDS thermal
  ceiling at ~24&nbsp;A. The brake also measured stronger than its nominal map: static hold
  ≈ 9–10&nbsp;N·m at 2.5&nbsp;V (nominal 8), with noticeable remanent drag after de-energizing.</p>
</section>""")

    # --- drive test
    r_drive = read_run("run-20260817-155947-agent")
    s.append("""
<section>
  <h2><span class="secno">5</span>The wheelchair-start test: it drives — the start is the problem</h2>
  <p>Target: from standstill under load, ramp to 60&nbsp;rpm and hold. Light load
  (1.25&nbsp;V ≈ 2.6&nbsp;N·m): <strong>clean pass</strong> — 59.8&nbsp;±&nbsp;1.0&nbsp;rpm at
  4.4&nbsp;A. Heavy load (2.5&nbsp;V, ≈ 9.6&nbsp;N·m): 15&nbsp;A can't break away (marginal by
  design), 25&nbsp;A hit the VDS thermal ceiling during stalled windup, and 20&nbsp;A — plotted
  below — told the whole story in one trace.</p>
""")
    if r_drive:
        win = window(r_drive, "16:00:40.4", "16:00:55.1")
        if win:
            def hsec(hh):
                h_, m_, rest = hh.split(":")
                return int(h_) * 3600 + int(m_) * 60 + float(rest)
            t0 = hsec(win[0]["host"][11:23])
            x = [round(hsec(r["host"][11:23]) - t0, 2) for r in win]
            s.append(line_chart(
                "c-rpm", "hrun 20 A → 60 rpm under 9.6 N·m — wheel speed",
                x, [("rpm (halls)", "--series-1", [r["rpm_hall"] for r in win])],
                "seconds since hrun", "mechanical rpm",
                notes=[(6.9, 232, "breakaway lurch to 228 rpm", "start"),
                       (8.3, 18, "re-stall", "middle"),
                       (13.2, 78, "converges: 48→58 rpm, 126 W", "middle")]))
            s.append(line_chart(
                "c-iq", "hrun 20 A → 60 rpm under 9.6 N·m — torque current",
                x, [("iq measured", "--series-1", [r["iq"] for r in win]),
                    ("iq reference", "--series-2", [r["iqref"] for r in win])],
                "seconds since hrun", "amps"))
    s.append("""
  <p>Sequence: the speed-loop integrator winds up inside the hall blind spot, breakaway arrives
  with ~18&nbsp;A of pent-up torque → overshoot to 228&nbsp;rpm; the loop sheds torque, the
  mag-particle brake (whose drag is speed-independent) re-grips to a dead stop; second breakaway;
  then <strong>stable convergence at ~15.5&nbsp;A / 126&nbsp;W electrical under 9.6&nbsp;N·m</strong>.
  Steady-state drive under load is demonstrated; the start transient is dominated by the hall
  blind spot — and note the bench is <em>harsher</em> than the vehicle: real wheelchair loads
  (ramps, friction) don't re-grip at full torque at speed the way a particle brake does.</p>
</section>

<section>
  <h2><span class="secno">6</span>Conclusion and what's next</h2>
  <p class="conclusion">The power stage, current control, and timing are wheelchair-ready:
  0.1&nbsp;A-accurate torque control to 22&nbsp;A at stall, 126&nbsp;W sustained under load,
  0.0% control-loop jitter. The single remaining obstacle is <strong>rotor-angle resolution at
  standstill</strong> — 60°-electrical hall sectors cost up to half the torque during stall creep
  and cause the breakaway lurch. That is an architecture limit, not a tuning problem.</p>
  <ul>
    <li><strong>Decision: move to an absolute encoder (MT6701) for the drive angle</strong>, halls
    retained as a sanity/redundancy channel. Mount is being fabricated; BSP driver already exists.</li>
    <li>Board rev: 1–4.7 kΩ external hall pull-ups (noise), and investigate the VM-domain chip
    wedge (recoverable by automated VM power-cycle, but shouldn't happen).</li>
    <li>Known envelope until then: ≤ 22 A sustained stall at <code>vds 4</code>; the harness's
    watchdogs + verified VM-cycle recovery make unattended runs safe.</li>
  </ul>
</section>

<section class="appendix">
  <h2><span class="secno">A</span>Appendix: bench &amp; run archive</h2>
  <p>Bench control is fully scripted (<code>hall_test.py</code>): board console over USB,
  BK&nbsp;MR3K160120 (VM 48&nbsp;V, SCPI) for fault/wedge recovery, Rigol&nbsp;DP2031 CH2
  (mag brake, USBTMC; CH1 reserved for the 24&nbsp;V torque sensor). Brake map nominal
  1.0/1.5/2.0/2.5&nbsp;V → 1/3/5/8&nbsp;N·m; measured static hold at 2.5&nbsp;V ≈ 9–10&nbsp;N·m.
  Every run below is archived with raw serial, parsed CSV, and an event log.</p>
  <div class="scroll"><table><thead><tr><th>run</th><th>rows</th><th>first event</th></tr></thead><tbody>""")
    for name, n, first in run_index():
        s.append(f"<tr><td><code>{html.escape(name)}</code></td><td>{n}</td><td>{html.escape(first)}</td></tr>")
    s.append("""</tbody></table></div>
</section>
<footer class="rpt-foot">Compiled by <code>compile_report.py</code> from
<code>data-reports/logs/</code> · branch
<code>feat/bench-campaign</code></footer>
""")
    return "".join(s)


CSS = """
.rpt { color-scheme: light;
  --page:#f9f9f7; --surface-1:#fcfcfb; --ink:#0b0b0b; --ink-2:#52514e; --muted:#898781;
  --grid:#e1e0d9; --axis:#c3c2b7; --border:rgba(11,11,11,.10);
  --series-1:#2a78d6; --series-2:#eb6834; --series-3:#1baf7a; --series-4:#eda100;
  --good:#006300;
  font-family:system-ui,-apple-system,"Segoe UI",sans-serif; color:var(--ink);
  background:var(--page); margin:0 auto; max-width:920px; padding:40px 28px 64px;
  line-height:1.55; font-size:16px; }
@media (prefers-color-scheme: dark) { :root:where(:not([data-theme="light"])) .rpt {
  color-scheme:dark; --page:#0d0d0d; --surface-1:#1a1a19; --ink:#fff; --ink-2:#c3c2b7;
  --muted:#898781; --grid:#2c2c2a; --axis:#383835; --border:rgba(255,255,255,.10);
  --series-1:#3987e5; --series-2:#d95926; --series-3:#199e70; --series-4:#c98500;
  --good:#0ca30c; } }
:root[data-theme="dark"] .rpt { color-scheme:dark; --page:#0d0d0d; --surface-1:#1a1a19;
  --ink:#fff; --ink-2:#c3c2b7; --muted:#898781; --grid:#2c2c2a; --axis:#383835;
  --border:rgba(255,255,255,.10); --series-1:#3987e5; --series-2:#d95926;
  --series-3:#199e70; --series-4:#c98500; --good:#0ca30c; }
.rpt code { font-family:ui-monospace,"SF Mono",Menlo,monospace; font-size:.86em;
  background:var(--surface-1); border:1px solid var(--border); border-radius:4px; padding:1px 5px; }
.rpt h1 { font-size:1.9rem; line-height:1.2; text-wrap:balance; margin:.35em 0 .4em; letter-spacing:-.015em; }
.rpt h2 { font-size:1.22rem; margin:2.4em 0 .6em; letter-spacing:-.01em; display:flex; align-items:baseline; gap:.6em; }
.rpt .secno { font-family:ui-monospace,"SF Mono",Menlo,monospace; font-size:.8em; color:var(--muted);
  border:1px solid var(--border); border-radius:4px; padding:1px 7px; }
.rpt .eyebrow { text-transform:uppercase; letter-spacing:.09em; font-size:.72rem; color:var(--muted); margin:0; }
.rpt .lede { color:var(--ink-2); max-width:68ch; }
.rpt p { max-width:72ch; }
.rpt .tiles { display:grid; grid-template-columns:repeat(auto-fit,minmax(180px,1fr)); gap:12px; margin:1.6em 0; }
.rpt .tile { background:var(--surface-1); border:1px solid var(--border); border-radius:8px;
  padding:14px 16px; display:flex; flex-direction:column; gap:3px; }
.rpt .t-label { font-size:.74rem; color:var(--ink-2); }
.rpt .t-value { font-size:1.9rem; font-weight:600; letter-spacing:-.01em; }
.rpt .t-value .unit { font-size:.55em; font-weight:500; color:var(--ink-2); }
.rpt .t-delta { font-size:.72rem; color:var(--muted); }
.rpt .t-delta.up { color:var(--good); }
.rpt table { border-collapse:collapse; font-size:.85rem; width:100%; font-variant-numeric:tabular-nums; }
.rpt th { text-align:left; color:var(--ink-2); font-weight:600; border-bottom:1px solid var(--axis); padding:6px 12px 6px 0; }
.rpt td { border-bottom:1px solid var(--grid); padding:6px 12px 6px 0; vertical-align:top; }
.rpt .scroll { overflow-x:auto; margin:.8em 0 1.4em; }
.rpt .viz { margin:1.6em 0 2em; background:var(--surface-1); border:1px solid var(--border);
  border-radius:8px; padding:14px 14px 8px; }
.rpt figcaption { display:flex; flex-wrap:wrap; align-items:baseline; gap:6px 16px; margin-bottom:6px; }
.rpt .fig-title { font-weight:600; font-size:.92rem; }
.rpt .fig-y { color:var(--muted); font-size:.74rem; }
.rpt .key { font-size:.76rem; color:var(--ink-2); display:inline-flex; align-items:center; gap:6px; }
.rpt .key-line { width:16px; height:2px; border-radius:1px; display:inline-block; }
.rpt .viz-wrap { position:relative; }
.rpt svg { width:100%; height:auto; display:block; }
.rpt .grid { stroke:var(--grid); stroke-width:1; }
.rpt .axis { stroke:var(--axis); stroke-width:1; }
.rpt .ref { stroke:var(--muted); stroke-width:1; stroke-dasharray:5 4; }
.rpt .axis-label { fill:var(--ink-2); font-size:11.5px; font-family:system-ui,sans-serif; }
.rpt .tick { fill:var(--muted); font-size:11px; font-family:system-ui,sans-serif; font-variant-numeric:tabular-nums; }
.rpt .val { fill:var(--ink-2); font-size:11px; font-variant-numeric:tabular-nums; }
.rpt .note { fill:var(--ink-2); font-size:11px; }
.rpt .xh { stroke:var(--axis); stroke-width:1; }
.rpt .bar:hover, .rpt .bar:focus { filter:brightness(1.12); outline:none; }
.rpt .tip { position:absolute; pointer-events:none; background:var(--surface-1);
  border:1px solid var(--border); border-radius:6px; padding:7px 10px; font-size:.78rem;
  box-shadow:0 2px 10px rgba(0,0,0,.12); min-width:120px; z-index:2; }
.rpt .tip .tv { font-weight:600; font-variant-numeric:tabular-nums; }
.rpt .tip .tn { color:var(--ink-2); }
.rpt .tip .tk { display:inline-block; width:12px; height:2px; border-radius:1px; margin-right:6px; vertical-align:middle; }
.rpt .tblview { margin:4px 0 6px; }
.rpt .tblview summary { font-size:.76rem; color:var(--muted); cursor:pointer; }
.rpt .tblview table { font-size:.76rem; margin-top:6px; }
.rpt .conclusion { font-size:1.05rem; }
.rpt .appendix td, .rpt .appendix th { font-size:.78rem; }
.rpt .rpt-foot { margin-top:3em; color:var(--muted); font-size:.76rem; border-top:1px solid var(--grid); padding-top:1em; }
@media print { .rpt { max-width:none; } .rpt .tblview { display:none; } }
"""

JS = """
document.querySelectorAll('.rpt .viz').forEach(function(fig){
  var dataEl = fig.querySelector('.viz-data'); if(!dataEl) return;
  var d = JSON.parse(dataEl.textContent);
  var svg = fig.querySelector('svg'), tip = fig.querySelector('.tip'), wrap = fig.querySelector('.viz-wrap');
  function show(x, y, rows){
    tip.hidden = false;
    while (tip.firstChild) tip.removeChild(tip.firstChild);
    rows.forEach(function(r){
      var line = document.createElement('div');
      if (r.color){ var k = document.createElement('span'); k.className='tk';
        k.style.background = 'var(' + r.color + ')'; line.appendChild(k); }
      var v = document.createElement('span'); v.className='tv'; v.textContent = r.value;
      var n = document.createElement('span'); n.className='tn'; n.textContent = ' ' + r.name;
      line.appendChild(v); line.appendChild(n); tip.appendChild(line);
    });
    var bw = wrap.getBoundingClientRect(), tw = tip.getBoundingClientRect();
    var lx = Math.min(x + 14, bw.width - tw.width - 4);
    tip.style.left = Math.max(lx, 4) + 'px';
    tip.style.top = Math.max(y - tw.height - 10, 4) + 'px';
  }
  function hide(){ tip.hidden = true; var xh = svg.querySelector('.xh'); if (xh) xh.style.display='none'; }
  if (d.type === 'line'){
    var vb = svg.viewBox.baseVal;
    svg.addEventListener('pointermove', function(ev){
      var r = svg.getBoundingClientRect();
      var px = (ev.clientX - r.left) / r.width * vb.width;
      var xv = d.xlo + (px - d.ml) / d.pw * (d.xhi - d.xlo);
      var best = 0, bd = 1e9;
      d.x.forEach(function(v,i){ var e = Math.abs(v - xv); if (e < bd){ bd = e; best = i; } });
      var sx = d.ml + (d.x[best] - d.xlo) / (d.xhi - d.xlo) * d.pw;
      var xh = svg.querySelector('.xh');
      if (xh){ xh.style.display=''; xh.setAttribute('x1', sx); xh.setAttribute('x2', sx); }
      var rows = [{value: d.x[best], name: d.xlabel}];
      d.series.forEach(function(sr){ rows.push({value: sr.v[best], name: sr.n, color: sr.c}); });
      show((ev.clientX - r.left), (ev.clientY - r.top), rows);
    });
    svg.addEventListener('pointerleave', hide);
  } else if (d.type === 'bar'){
    svg.querySelectorAll('.bar').forEach(function(b){
      b.addEventListener('pointermove', function(ev){
        var i = +b.dataset.i, r = svg.getBoundingClientRect();
        show(ev.clientX - r.left, ev.clientY - r.top,
             [{value: d.vals[i], name: d.ylabel}, {value: '', name: d.cats[i]}]);
      });
      b.addEventListener('pointerleave', hide);
    });
  }
});
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fragment", type=Path,
                    help="also write a body-only fragment (no doctype/head) to this path")
    ap.add_argument("-o", "--out", type=Path, default=APP / "report" / "index.html")
    args = ap.parse_args()

    body = build_body()
    fragment = (f"<style>{CSS}</style>\n<div class=\"rpt\">{body}</div>\n"
                f"<script>{JS}</script>")
    standalone = ("<!doctype html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n"
                  "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
                  "<title>PACE RACER — low-speed/high-torque test campaign</title>\n"
                  f"</head>\n<body style=\"margin:0\">{fragment}</body>\n</html>\n")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(standalone)
    print(f"wrote {args.out} ({len(standalone)/1024:.0f} KB)")
    if args.fragment:
        args.fragment.parent.mkdir(parents=True, exist_ok=True)
        args.fragment.write_text(f"<title>PACE RACER — motor drive test campaign</title>\n{fragment}")
        print(f"wrote {args.fragment}")


if __name__ == "__main__":
    main()
