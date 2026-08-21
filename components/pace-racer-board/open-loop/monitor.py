#!/usr/bin/env python3
"""
PACE RACER FOC live monitor.

Usage:
    python monitor.py [port [baud]]
    python monitor.py /dev/cu.usbmodem2101
    python monitor.py /dev/cu.usbmodem2101 115200
"""

import math
import sys
from collections import deque
from dataclasses import dataclass
from typing import Optional

import serial
from textual import work
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.message import Message
from textual.widgets import Footer, Header, Label, Rule, Sparkline, Static
from textual_plotext import PlotextPlot

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2101"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
HIST = 200  # sparkline history depth
PP   = 15   # pole pairs — must match kPolePairs in open_loop.cpp
TARGET_RPM = 30.0  # must match kTargetVelocityRpm in main/open_loop.cpp

_INV_SQRT3 = 1.0 / math.sqrt(3)


# ── data ──────────────────────────────────────────────────────────────────────

@dataclass
class Row:
    t: float
    hall_rpm: float
    shaft_angle: float
    shaft_rpm: float
    vref_mv: float
    ia: float
    ib: float
    ic: float


def parse(line: str) -> Optional[Row]:
    s = line.strip()
    if not s or s.startswith("%"):
        return None
    parts = s.split(",")
    if len(parts) != 8:
        return None
    try:
        return Row(*[float(p) for p in parts])
    except ValueError:
        return None


def dq(ia: float, ib: float, ic: float, shaft_angle: float) -> tuple[float, float]:
    """Clarke then Park transform → (id, iq)."""
    theta = (shaft_angle * PP) % (2 * math.pi)
    ialpha = ia
    ibeta  = (ia + 2 * ib) * _INV_SQRT3
    c, s = math.cos(theta), math.sin(theta)
    return ialpha * c + ibeta * s, -ialpha * s + ibeta * c


def bar(value: float, scale: float = 3.0, width: int = 16) -> str:
    filled = min(int(abs(value) / scale * width), width)
    return f"{value:+.3f} A  {'█' * filled}{'░' * (width - filled)}"


# ── messages ──────────────────────────────────────────────────────────────────

class RowReceived(Message):
    def __init__(self, row: Row) -> None:
        super().__init__()
        self.row = row


class SerialStatus(Message):
    def __init__(self, connected: bool, detail: str = "") -> None:
        super().__init__()
        self.connected = connected
        self.detail = detail


# ── widgets ───────────────────────────────────────────────────────────────────

class CurrentPlot(PlotextPlot):
    def on_mount(self) -> None:
        self.plt.theme("dark")
        self.plt.ylabel("A")

    def feed(self, ia: list, ib: list, ic: list) -> None:
        self.plt.clear_data()
        self.plt.plot(ia, label="A", color="red")
        self.plt.plot(ib, label="B", color="green")
        self.plt.plot(ic, label="C", color="blue")
        self.refresh()


class DQPlot(PlotextPlot):
    def on_mount(self) -> None:
        self.plt.theme("dark")
        self.plt.ylabel("A")

    def feed(self, id_: list, iq: list) -> None:
        self.plt.clear_data()
        self.plt.plot(id_, label="id (flux)", color="magenta")
        self.plt.plot(iq,  label="iq (torque)", color="cyan")
        self.refresh()


# ── app ───────────────────────────────────────────────────────────────────────

class Monitor(App):
    CSS = """
    Screen { background: $surface; }

    #status { height: 1; background: $primary; color: $text; padding: 0 1; }
    #status.err { background: $error; }

    Horizontal { height: 1fr; }
    #left  { width: 44; padding: 1 2; border-right: solid $primary; }
    #right { width: 1fr; padding: 1 2; }

    .head { color: $accent; text-style: bold; margin-bottom: 1; }
    .val  { height: 1; }

    Sparkline { height: 8; margin: 1 0; }
    CurrentPlot { height: 1fr; margin-top: 1; }
    DQPlot      { height: 1fr; margin-top: 1; }
    Rule { margin: 1 0; color: $primary; }
    """

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Static(f"connecting  {PORT}  {BAUD} baud", id="status")
        with Horizontal():
            with Vertical(id="left"):
                yield Static("VELOCITY", classes="head")
                yield Label("hall rpm    ---", id="hall-rpm",  classes="val")
                yield Label("shaft rpm   ---", id="shaft-rpm", classes="val")
                yield Label(f"target      {TARGET_RPM:5.1f}", id="target",   classes="val")
                yield Label("angle       --- rad", id="angle", classes="val")
                yield Rule()
                yield Static("PHASE CURRENT", classes="head")
                yield Label("A  ---", id="ia",   classes="val")
                yield Label("B  ---", id="ib",   classes="val")
                yield Label("C  ---", id="ic",   classes="val")
                yield Label("vref  --- mV", id="vref", classes="val")
            with Vertical(id="right"):
                yield Static("RPM — hall sensor", classes="head")
                yield Sparkline([], id="rpm-spark", summary_function=max)
                yield Static("PHASE CURRENTS (A, B, C)", classes="head")
                yield CurrentPlot(id="current-plot")
                yield Static("DQ CURRENTS", classes="head")
                yield DQPlot(id="dq-plot")
        yield Footer()

    def on_mount(self) -> None:
        self._rpm: deque[float] = deque([0.0], maxlen=HIST)
        self._ia:  deque[float] = deque([0.0], maxlen=HIST)
        self._ib:  deque[float] = deque([0.0], maxlen=HIST)
        self._ic:  deque[float] = deque([0.0], maxlen=HIST)
        self._id:  deque[float] = deque([0.0], maxlen=HIST)
        self._iq:  deque[float] = deque([0.0], maxlen=HIST)
        self._read()

    @work(thread=True)
    def _read(self) -> None:
        try:
            with serial.Serial(PORT, BAUD, timeout=1.0) as ser:
                self.post_message(SerialStatus(connected=True))
                while True:
                    try:
                        line = ser.readline().decode("utf-8", errors="replace")
                    except serial.SerialException as e:
                        self.post_message(SerialStatus(connected=False, detail=str(e)))
                        return
                    row = parse(line)
                    if row:
                        self.post_message(RowReceived(row))
        except serial.SerialException as e:
            self.post_message(SerialStatus(connected=False, detail=str(e)))

    def on_row_received(self, m: RowReceived) -> None:
        r = m.row
        id_, iq = dq(r.ia, r.ib, r.ic, r.shaft_angle)
        self._rpm.append(r.hall_rpm)
        self._ia.append(r.ia)
        self._ib.append(r.ib)
        self._ic.append(r.ic)
        self._id.append(id_)
        self._iq.append(iq)

        self.query_one("#hall-rpm",  Label).update(f"hall rpm    {r.hall_rpm:7.2f}")
        self.query_one("#shaft-rpm", Label).update(f"shaft rpm   {r.shaft_rpm:7.2f}")
        self.query_one("#angle",     Label).update(f"angle       {r.shaft_angle:.4f} rad")
        self.query_one("#ia",        Label).update(f"A  {bar(r.ia)}")
        self.query_one("#ib",        Label).update(f"B  {bar(r.ib)}")
        self.query_one("#ic",        Label).update(f"C  {bar(r.ic)}")
        self.query_one("#vref",      Label).update(f"vref  {r.vref_mv:.1f} mV")

        self.query_one("#rpm-spark", Sparkline).data = list(self._rpm)
        self.query_one("#current-plot", CurrentPlot).feed(
            list(self._ia), list(self._ib), list(self._ic)
        )
        self.query_one("#dq-plot", DQPlot).feed(list(self._id), list(self._iq))

    def on_serial_status(self, m: SerialStatus) -> None:
        s = self.query_one("#status", Static)
        if m.connected:
            s.update(f"● connected  {PORT}  {BAUD} baud")
            s.remove_class("err")
        else:
            s.update(f"✗ {m.detail or 'disconnected'}")
            s.add_class("err")


if __name__ == "__main__":
    Monitor().run()
