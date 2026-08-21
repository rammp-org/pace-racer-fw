#!/usr/bin/env python3
"""
PACE RACER halls-sensorless test monitor + recorder (hall-FOC variant).

Hall workflow: `ho 9999 45 1` + `run 2 30 5` + `hcal` (once per session), then
`sine <A> <rpm_amp> <period_s>` for the sine sweep — crank the brake torque up
between periods and watch the rpm-tracking plot + temps. `hrun <A> <rpm>` for
constant speed, `hiq <A>` for stall torque, `hs` for hall diagnostics.

Reads the 10 Hz CSV the sensorless firmware streams over the USB console:
    %t, mode, id, iq, iqref, vd, vq, aerr, rpm_drive, rpm_est, rpm_hall, flux,
    t0, t1, t2, t3
and derives electrical power P = 1.5*(vd*id + vq*iq), estimated bus current,
loop-voltage saturation, and per-sensor dT/dt. Every session is recorded to
logs/run-<timestamp>/: raw.log (every serial line), data.csv (parsed + derived
columns), events.log ('#'/'!' firmware lines and local annotations), and a
report.md summary written on exit (or with ctrl+e).

Type firmware commands in the input box (run/stop/lim/vl/tl/vds/...).
Local commands in the same box:
    t= <Nm>    record a manual torque-display reading (used for mech power/eff
               until the next entry — the serial torque source replaces this)
    p= <W>     record the PSU's displayed watts (for the board-loss delta)
Escape (or the STOP button) sends `stop` immediately.

Torque-sensor forward compatibility: implement SerialTorqueSource.poll() when
the capture cable arrives and pass --torque-port; the CSV/report columns
(torque_nm, torque_src, p_mech_w, eff) already exist and carry 'm' (manual) or
's' (sensor) provenance.

Usage:
    python monitor.py [port [baud]] [--torque-port DEV]
    python monitor.py /dev/cu.usbmodem101
"""

import math
import re
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

from telemetry import VBUS, Row, parse

import serial
from textual import work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.message import Message
from textual.widgets import Button, Footer, Header, Input, Label, RichLog, Rule, Static
from textual_plotext import PlotextPlot

_args = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = _args[0] if _args else "/dev/cu.usbmodem101"
BAUD = int(_args[1]) if len(_args) > 1 else 115200
TORQUE_PORT = None
for _i, _a in enumerate(sys.argv):
    if _a == "--torque-port" and _i + 1 < len(sys.argv):
        TORQUE_PORT = sys.argv[_i + 1]

# VBUS / NCOLS / Row / parse live in telemetry.py (stdlib-only, testable).
HIST = 300  # fast plots: 10 Hz -> 30 s window
TEMP_HIST = 1800  # temp plot: 3 min window
DTDT_WINDOW_S = 10.0  # dT/dt measured across this span
# Host-side watchdog (redundant with the firmware 'tl' guard, which is
# authoritative — USB can drop). Sends `stop` once, then a cooldown.
DTDT_STOP_C_PER_S = 2.0
WATCHDOG_COOLDOWN_S = 10.0
STALL_ALERT_S = 3.0
STATS_POLL_S = 10.0  # periodic `s` so filter-health counters land in events.log

DRIVING_MODES = {"S", "V", "C", "L", "W", "Q"}
MODE_NAMES = {"H": "HOLD", "S": "I/f", "V": "CONVERGE", "C": "CLOSED", "X": "STOPPING",
              "L": "HALL-SPEED", "W": "HALL-SINE", "Q": "HALL-IQ"}


class TorqueSource:
    """Reads the dyno torque sensor. poll() is called on each stream row and
    returns (torque_nm, source_char) or None while no fresh reading exists."""

    def poll(self) -> Optional[tuple]:
        return None

    def close(self) -> None:
        pass


class SerialTorqueSource(TorqueSource):
    """TODO(torque cable): open the sensor's port here and parse its protocol;
    return (nm, 's') from poll(). Everything downstream already handles it."""

    def __init__(self, port: str) -> None:
        raise NotImplementedError(
            f"torque-sensor serial protocol not implemented yet (port {port})")


class RowReceived(Message):
    def __init__(self, row: Row) -> None:
        super().__init__()
        self.row = row


class LineReceived(Message):
    def __init__(self, line: str) -> None:
        super().__init__()
        self.line = line


class SerialStatus(Message):
    def __init__(self, connected: bool, detail: str = "") -> None:
        super().__init__()
        self.connected = connected
        self.detail = detail


class SeriesPlot(PlotextPlot):
    def __init__(self, ylabel: str, series_names, colors, **kw) -> None:
        super().__init__(**kw)
        self._ylabel = ylabel
        self._names = series_names
        self._colors = colors

    def on_mount(self) -> None:
        self.plt.theme("dark")
        self.plt.ylabel(self._ylabel)

    def feed(self, series) -> None:
        self.plt.clear_data()
        for data, name, color in zip(series, self._names, self._colors):
            self.plt.plot(list(data), label=name, color=color)
        self.refresh()


class Monitor(App):
    BINDINGS = [
        Binding("escape", "estop", "STOP", priority=True),
        Binding("ctrl+e", "export", "Write report", priority=True),
    ]

    CSS = """
    Screen { background: $surface; }
    #status { height: 1; background: $primary; color: $text; padding: 0 1; }
    #status.err { background: $error; }
    Horizontal { height: 1fr; }
    #left  { width: 46; padding: 0 1; border-right: solid $primary; }
    #right { width: 1fr; padding: 0 1; }
    .head { color: $accent; text-style: bold; }
    .val  { height: 1; }
    SeriesPlot { height: 1fr; }
    Rule { margin: 0 0; color: $primary; }
    #stop { width: 100%; }
    #console { height: 8; border-top: solid $primary; }
    """

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Static(f"connecting  {PORT}  {BAUD} baud", id="status")
        with Horizontal():
            with Vertical(id="left"):
                yield Static("CONTROL", classes="head")
                yield Label("mode     ---", id="mode", classes="val")
                yield Label("id/iq    ---", id="idq", classes="val")
                yield Label("iqref    ---", id="iqref", classes="val")
                yield Label("vd/vq    ---", id="vdq", classes="val")
                yield Label("|v|/vlim ---", id="sat", classes="val")
                yield Rule()
                yield Static("POWER", classes="head")
                yield Label("P elec   ---", id="pw", classes="val")
                yield Label("I bus    ---", id="ibus", classes="val")
                yield Label("P mech   ---", id="pmech", classes="val")
                yield Rule()
                yield Static("SPEED  drive/est/hall", classes="head")
                yield Label("rpm      ---", id="rpms", classes="val")
                yield Label("flux     ---", id="flux", classes="val")
                yield Label("aerr     ---", id="aerr", classes="val")
                yield Rule()
                yield Static("TEMPS (°C, dT/dt °C/s)", classes="head")
                yield Label("T0  ---", id="t0", classes="val")
                yield Label("T1  ---", id="t1", classes="val")
                yield Label("T2  ---", id="t2", classes="val")
                yield Label("T3  ---", id="t3", classes="val")
                yield Rule()
                yield Static("LIMITS (from fw echoes)", classes="head")
                yield Label("lim  ---", id="lims", classes="val")
                yield Label("vds  ---", id="vds", classes="val")
                yield Rule()
                yield Label("torque   --- (t= <Nm>)", id="torque", classes="val")
                yield Label("psu      --- (p= <W>)", id="psu", classes="val")
                yield Button("STOP (esc)", id="stop", variant="error")
                yield Input(placeholder="fw cmd | t= Nm | p= W", id="cmd")
            with Vertical(id="right"):
                yield SeriesPlot("rpm", ("est", "hall"), ("magenta", "green"), id="plot-rpm")
                yield SeriesPlot("°C", ("T0", "T1", "T2", "T3"),
                                 ("red", "orange", "green", "cyan"), id="plot-temp")
                yield SeriesPlot("A", ("id", "iq"), ("cyan", "magenta"), id="plot-i")
                yield SeriesPlot("W", ("P",), ("yellow",), id="plot-p")
                yield RichLog(id="console", markup=False, wrap=False, max_lines=300)
        yield Footer()

    # ---------------- lifecycle ----------------

    def on_mount(self) -> None:
        self._closing = False
        self._ser: Optional[serial.Serial] = None
        self._temps = [deque(maxlen=TEMP_HIST) for _ in range(4)]
        self._id = deque(maxlen=HIST)
        self._iq = deque(maxlen=HIST)
        self._p = deque(maxlen=HIST)
        self._rpm_est = deque(maxlen=HIST)
        self._rpm_hall = deque(maxlen=HIST)
        self._temp_hist = deque()  # (host_t, temps) for dT/dt
        self._dtdt = [float("nan")] * 4
        self._vlim = 8.0
        self._lim_str = "8.0/15.0 A (boot)"
        self._vds_str = "0.06 V (boot)"
        self._torque = None  # (nm, src, host_t)
        self._psu_w = None
        self._last_row_t = None
        self._last_line_t = None
        self._last_mode = "H"
        self._stalled_shown = False
        self._watchdog_at = 0.0
        self._stats_at = time.monotonic()
        self._torque_src = SerialTorqueSource(TORQUE_PORT) if TORQUE_PORT else TorqueSource()

        run_dir = Path(__file__).parent / "logs" / \
            datetime.now().strftime("run-%Y%m%d-%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
        self._run_dir = run_dir
        self._raw = open(run_dir / "raw.log", "w")
        self._events = open(run_dir / "events.log", "w")
        self._csv = open(run_dir / "data.csv", "w")
        self._csv.write(
            "host_time,t,mode,id,iq,iqref,vd,vq,vmag,sat,p_w,ibus,aerr,"
            "rpm_drive,rpm_est,rpm_hall,flux,t0,t1,t2,t3,"
            "dtdt0,dtdt1,dtdt2,dtdt3,torque_nm,torque_src,p_mech_w,eff,psu_w\n")
        # Running aggregates for the report.
        self._agg = {}
        self._mode_s = {}
        self._n_rows = 0
        self._started = datetime.now()
        self._event(f"session start, port={PORT}, logs={run_dir}")
        self.set_interval(0.5, self._tick)
        self.query_one("#cmd", Input).focus()  # typing works immediately
        self._read()

    def on_unmount(self) -> None:
        self._closing = True
        self._write_report()
        for f in (self._raw, self._events, self._csv):
            try:
                f.close()
            except Exception:
                pass
        self._torque_src.close()

    # ---------------- serial ----------------

    @work(thread=True)
    def _read(self) -> None:
        while not self._closing:
            try:
                with serial.Serial(PORT, BAUD, timeout=1.0) as ser:
                    self._ser = ser
                    self.post_message(SerialStatus(connected=True))
                    while not self._closing:
                        line = ser.readline().decode("utf-8", errors="replace")
                        if not line:
                            continue
                        self.post_message(LineReceived(line))
                        row = parse(line)
                        if row:
                            self.post_message(RowReceived(row))
            except serial.SerialException as e:
                self._ser = None
                self.post_message(SerialStatus(connected=False, detail=str(e)))
                time.sleep(2.0)  # auto-retry: bench resets are routine
        self._ser = None

    def _send(self, cmd: str) -> None:
        if self._ser is None:
            self._event(f"SEND FAILED (disconnected): {cmd}")
            self.notify(f"NOT CONNECTED — `{cmd}` not sent (is another terminal "
                        "holding the port?)", severity="error", timeout=10)
            return
        try:
            self._ser.write((cmd.strip() + "\n").encode())
            self._event(f"> {cmd.strip()}")
            self.query_one("#console", RichLog).write(f"> {cmd.strip()}")
        except serial.SerialException as e:
            self.post_message(SerialStatus(connected=False, detail=str(e)))

    # ---------------- recording ----------------

    def _event(self, text: str) -> None:
        stamp = datetime.now().isoformat(timespec="milliseconds")
        self._events.write(f"{stamp} {text}\n")
        self._events.flush()

    def _acc(self, key: str, v: float) -> None:
        if math.isnan(v):
            return
        lo, hi = self._agg.get(key, (v, v))
        self._agg[key] = (min(lo, v), max(hi, v))

    # ---------------- events from the worker ----------------

    def on_line_received(self, m: LineReceived) -> None:
        line = m.line.rstrip("\n")
        self._last_line_t = time.monotonic()
        self._raw.write(
            f"{datetime.now().isoformat(timespec='milliseconds')} {line}\n")
        self._raw.flush()
        s = line.strip()
        if not (s.startswith("#") or s.startswith("!")):
            return
        self._event(s)
        self.query_one("#console", RichLog).write(s)
        if s.startswith("!"):
            self.notify(s, severity="error", timeout=15)
        # The firmware stream is silent while disarmed; when it tells us the
        # drive ended (fault/coast/disarm), stop treating quiet as a stall.
        if "disarmed" in s or s.startswith("#coast") or s.startswith("#stop"):
            self._last_mode = "H"
        mv = re.search(r"vlim\s+([\d.]+)", s)
        if mv:
            self._vlim = float(mv.group(1))
        ml = re.search(r"target=([\d.]+)A trip=([\d.]+)A", s)
        if ml:
            self._lim_str = f"{ml.group(1)}/{ml.group(2)} A"
        if s.startswith("#vds"):
            self._vds_str = s[5:]
        if ml or mv or s.startswith(("#vds", "#tlimit")):
            self.query_one("#lims", Label).update(
                f"lim  {self._lim_str}  vlim {self._vlim:.1f} V")
            self.query_one("#vds", Label).update(f"vds  {self._vds_str}")

    def on_row_received(self, m: RowReceived) -> None:
        r = m.row
        now = time.monotonic()
        self._last_row_t = now
        self._last_mode = r.mode
        self._n_rows += 1
        self._mode_s[r.mode] = self._mode_s.get(r.mode, 0.0) + 0.1

        for dq_, v in zip(self._temps, r.temps):
            dq_.append(v)
        self._id.append(r.id_a)
        self._iq.append(r.iq)
        self._p.append(r.p_w)
        self._rpm_est.append(r.rpm_est)
        self._rpm_hall.append(r.rpm_hall)

        # dT/dt across a 10 s window.
        self._temp_hist.append((now, r.temps))
        while self._temp_hist and now - self._temp_hist[0][0] > DTDT_WINDOW_S:
            self._temp_hist.popleft()
        t_old, temps_old = self._temp_hist[0]
        span = now - t_old
        for i in range(4):
            self._dtdt[i] = ((r.temps[i] - temps_old[i]) / span
                             if span > 1.0 else float("nan"))

        fresh = self._torque_src.poll()
        if fresh:
            self._torque = (fresh[0], fresh[1], now)
        torque_nm, torque_src = (self._torque[0], self._torque[1]) \
            if self._torque else (float("nan"), "")
        p_mech = torque_nm * r.rpm_est * 2 * math.pi / 60.0
        eff = p_mech / r.p_w if (not math.isnan(p_mech) and r.p_w > 1.0) else float("nan")
        sat = r.vmag / self._vlim if self._vlim else float("nan")
        psu = self._psu_w if self._psu_w is not None else float("nan")

        self._csv.write(
            f"{datetime.now().isoformat(timespec='milliseconds')},{r.t:.2f},"
            f"{r.mode},{r.id_a:.2f},{r.iq:.2f},{r.iqref:.2f},{r.vd:.2f},"
            f"{r.vq:.2f},{r.vmag:.2f},{sat:.3f},{r.p_w:.1f},{r.ibus:.2f},"
            f"{r.aerr:.0f},{r.rpm_drive:.0f},{r.rpm_est:.0f},{r.rpm_hall:.0f},"
            f"{r.flux:.4f},"
            + ",".join(f"{t:.1f}" for t in r.temps) + ","
            + ",".join(f"{d:.3f}" for d in self._dtdt)
            + f",{torque_nm:.2f},{torque_src},{p_mech:.1f},{eff:.3f},{psu:.1f}\n")
        self._csv.flush()

        for key, v in (("iq", abs(r.iq)), ("id", abs(r.id_a)), ("p_w", r.p_w),
                       ("ibus", r.ibus), ("sat", sat), ("rpm_est", r.rpm_est),
                       ("flux", r.flux)):
            self._acc(key, v)
        for i in range(4):
            self._acc(f"t{i}", r.temps[i])
            self._acc(f"dtdt{i}", self._dtdt[i])

        # Host-side thermal-runaway watchdog (firmware 'tl' is authoritative).
        if r.mode in DRIVING_MODES and now - self._watchdog_at > WATCHDOG_COOLDOWN_S:
            hot = [i for i, d in enumerate(self._dtdt)
                   if not math.isnan(d) and d > DTDT_STOP_C_PER_S]
            if hot:
                self._watchdog_at = now
                self._event(f"WATCHDOG: dT/dt {[f'T{i}={self._dtdt[i]:.2f}' for i in hot]}"
                            f" > {DTDT_STOP_C_PER_S} C/s — sending stop")
                self.notify("watchdog: thermal runaway slope — stop sent",
                            severity="error", timeout=15)
                self._send("stop")

        self._refresh_panels(r, sat, p_mech)

    def _refresh_panels(self, r: Row, sat: float, p_mech: float) -> None:
        q = self.query_one
        q("#mode", Label).update(f"mode     {r.mode} {MODE_NAMES.get(r.mode, '?')}")
        q("#idq", Label).update(f"id/iq    {r.id_a:+6.2f} / {r.iq:+6.2f} A")
        q("#iqref", Label).update(f"iqref    {r.iqref:+6.2f} A")
        q("#vdq", Label).update(f"vd/vq    {r.vd:+6.2f} / {r.vq:+6.2f} V")
        q("#sat", Label).update(f"|v|/vlim {r.vmag:5.2f}/{self._vlim:.1f} V "
                                f"({100 * sat:.0f}%)" + ("  SATURATED" if sat > 0.95 else ""))
        q("#pw", Label).update(f"P elec   {r.p_w:7.1f} W")
        q("#ibus", Label).update(f"I bus    {r.ibus:7.2f} A (est @ {VBUS:.0f} V)")
        q("#pmech", Label).update(
            f"P mech   {p_mech:7.1f} W" if not math.isnan(p_mech) else "P mech   ---")
        q("#rpms", Label).update(
            f"rpm      {r.rpm_drive:5.0f} / {r.rpm_est:5.0f} / {r.rpm_hall:5.0f}")
        q("#flux", Label).update(f"flux     {r.flux:.4f} Wb")
        q("#aerr", Label).update(f"aerr     {r.aerr:+4.0f}°")
        for i in range(4):
            q(f"#t{i}", Label).update(
                f"T{i}  {r.temps[i]:6.2f}  ({self._dtdt[i]:+.3f}/s)")
        q("#plot-rpm", SeriesPlot).feed((self._rpm_est, self._rpm_hall))
        q("#plot-temp", SeriesPlot).feed(self._temps)
        q("#plot-i", SeriesPlot).feed((self._id, self._iq))
        q("#plot-p", SeriesPlot).feed((self._p,))

    def on_serial_status(self, m: SerialStatus) -> None:
        s = self.query_one("#status", Static)
        if m.connected:
            s.update(f"● connected  {PORT}  {BAUD} baud  →  {self._run_dir.name}")
            s.remove_class("err")
            self._last_mode = "H"  # a (re)connected board is disarmed until told otherwise
            self._stalled_shown = False
        else:
            s.update(f"✗ {m.detail or 'disconnected'} — retrying")
            s.add_class("err")
            self._event(f"serial lost: {m.detail}")

    def _tick(self) -> None:
        now = time.monotonic()
        # Stall = NO serial traffic at all while the last-seen mode was driving.
        # (No data rows but live console lines just means disarmed — silent by
        # design.) Latches off again as soon as traffic resumes.
        stalled = (self._last_line_t is not None
                   and self._last_mode in DRIVING_MODES
                   and now - self._last_line_t > STALL_ALERT_S)
        st = self.query_one("#status", Static)
        if stalled and not self._stalled_shown:
            self._stalled_shown = True
            st.update(f"✗ no serial traffic >{STALL_ALERT_S:.0f}s while driving — "
                      "check link / PSU display")
            st.add_class("err")
        elif not stalled and self._stalled_shown:
            self._stalled_shown = False
            if self._ser is not None:
                st.update(f"● connected  {PORT}  {BAUD} baud  →  {self._run_dir.name}")
                st.remove_class("err")
        if self._ser is not None and now - self._stats_at > STATS_POLL_S:
            self._stats_at = now
            try:
                self._ser.write(b"s\n")  # silent poll: counters land in events.log
            except serial.SerialException:
                pass

    # ---------------- user actions ----------------

    def action_estop(self) -> None:
        self._send("stop")

    def action_export(self) -> None:
        self._write_report()
        self.notify(f"report written: {self._run_dir / 'report.md'}")

    def on_button_pressed(self, _: Button.Pressed) -> None:
        self.action_estop()

    def on_input_submitted(self, m: Input.Submitted) -> None:
        text = m.value.strip()
        m.input.value = ""
        if not text:
            return
        if text.startswith("t="):
            try:
                nm = float(text[2:])
            except ValueError:
                return
            self._torque = (nm, "m", time.monotonic())
            self._event(f"manual torque reading: {nm} Nm")
            self.query_one("#torque", Label).update(f"torque   {nm:.2f} Nm (manual)")
        elif text.startswith("p="):
            try:
                self._psu_w = float(text[2:])
            except ValueError:
                return
            self._event(f"manual PSU reading: {self._psu_w} W")
            self.query_one("#psu", Label).update(f"psu      {self._psu_w:.1f} W (manual)")
        else:
            self._send(text)

    # ---------------- report ----------------

    def _write_report(self) -> None:
        a = self._agg

        def rng(k, unit=""):
            if k not in a:
                return "—"
            lo, hi = a[k]
            return f"{lo:.2f} … {hi:.2f}{unit}"

        lines = [
            f"# Sensorless high-power run — {self._started:%Y-%m-%d %H:%M:%S}",
            "",
            f"- Port: `{PORT}` @ {BAUD}",
            f"- Duration: {datetime.now() - self._started}",
            f"- Rows: {self._n_rows} (10 Hz stream)",
            f"- Last limits seen: lim {self._lim_str}, vlim {self._vlim:.1f} V, "
            f"vds {self._vds_str}",
            "- Time per mode (s): "
            + ", ".join(f"{k}={v:.0f}" for k, v in sorted(self._mode_s.items())),
            "",
            "## Extremes",
            "",
            "| quantity | min … max |",
            "|---|---|",
            f"| \\|iq\\| (A) | {rng('iq')} |",
            f"| \\|id\\| (A) | {rng('id')} |",
            f"| P elec (W) | {rng('p_w')} |",
            f"| I bus est (A) | {rng('ibus')} |",
            f"| \\|v\\|/vlim | {rng('sat')} |",
            f"| rpm_est | {rng('rpm_est')} |",
            f"| flux (Wb) | {rng('flux')} |",
        ]
        for i in range(4):
            lines.append(f"| T{i} (°C) | {rng(f't{i}')} |")
            lines.append(f"| dT{i}/dt (°C/s) | {rng(f'dtdt{i}')} |")
        lines += [
            "",
            "## Events",
            "",
            "See `events.log` (all `#`/`!` firmware lines, sent commands, "
            "manual torque/PSU readings, watchdog actions) and `data.csv` "
            "for the full parsed stream.",
            "",
        ]
        (self._run_dir / "report.md").write_text("\n".join(lines))
        self._event("report.md written")


if __name__ == "__main__":
    Monitor().run()
