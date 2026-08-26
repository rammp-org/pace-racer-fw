#!/usr/bin/env python3
"""
PACE RACER temperature-sweep live monitor.

Reads the CSV that temp_sweep.cpp streams over the USB console:
    %time(s), target_rpm, hall_rpm, enabled, ia_a, ib_a, ic_a, temp0..temp3_c
Plots the four board temperatures, and the "Toggle Motor" button (or key `t`)
writes a space back to the firmware to enable/disable the motor.

Usage:
    python monitor.py [port [baud]]
    python monitor.py /dev/cu.usbmodem2101
"""

import sys
from collections import deque
from dataclasses import dataclass
from typing import Optional

import serial
from textual import work
from textual.app import App, ComposeResult
from textual.containers import Horizontal, Vertical
from textual.message import Message
from textual.widgets import Button, Footer, Header, Label, Rule, Static
from textual_plotext import PlotextPlot

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem2101"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
HIST = 300  # temps drift slowly; keep a long window

NCOLS = 11  # keep in lockstep with the CSV header in temp_sweep.cpp


@dataclass
class Row:
    t: float
    target_rpm: float
    hall_rpm: float
    enabled: float
    ia: float
    ib: float
    ic: float
    temps: tuple  # (t0, t1, t2, t3)


def parse(line: str) -> Optional[Row]:
    s = line.strip()
    if not s or s.startswith("%"):
        return None
    parts = s.split(",")
    if len(parts) != NCOLS:
        return None
    try:
        f = [float(p) for p in parts]
    except ValueError:
        return None
    return Row(f[0], f[1], f[2], f[3], f[4], f[5], f[6], tuple(f[7:11]))


class RowReceived(Message):
    def __init__(self, row: Row) -> None:
        super().__init__()
        self.row = row


class SerialStatus(Message):
    def __init__(self, connected: bool, detail: str = "") -> None:
        super().__init__()
        self.connected = connected
        self.detail = detail


class TempPlot(PlotextPlot):
    def on_mount(self) -> None:
        self.plt.theme("dark")
        self.plt.ylabel("°C")

    def feed(self, series) -> None:
        self.plt.clear_data()
        colors = ("red", "orange", "green", "cyan")
        for i, (data, color) in enumerate(zip(series, colors)):
            self.plt.plot(list(data), label=f"T{i}", color=color)
        self.refresh()


class Monitor(App):
    BINDINGS = [("t", "toggle_motor", "Toggle motor")]

    CSS = """
    Screen { background: $surface; }
    #status { height: 1; background: $primary; color: $text; padding: 0 1; }
    #status.err { background: $error; }
    Horizontal { height: 1fr; }
    #left  { width: 40; padding: 1 2; border-right: solid $primary; }
    #right { width: 1fr; padding: 1 2; }
    .head { color: $accent; text-style: bold; margin-bottom: 1; }
    .val  { height: 1; }
    TempPlot { height: 1fr; margin-top: 1; }
    Rule { margin: 1 0; color: $primary; }
    #toggle { margin-top: 1; width: 100%; }
    """

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Static(f"connecting  {PORT}  {BAUD} baud", id="status")
        with Horizontal():
            with Vertical(id="left"):
                yield Static("SWEEP", classes="head")
                yield Label("target rpm  ---", id="target", classes="val")
                yield Label("hall rpm    ---", id="hall-rpm", classes="val")
                yield Label("motor       ---", id="enabled", classes="val")
                yield Rule()
                yield Static("TEMPERATURE (°C)", classes="head")
                yield Label("T0  ---", id="t0", classes="val")
                yield Label("T1  ---", id="t1", classes="val")
                yield Label("T2  ---", id="t2", classes="val")
                yield Label("T3  ---", id="t3", classes="val")
                yield Button("Toggle Motor (t)", id="toggle", variant="warning")
            with Vertical(id="right"):
                yield Static("BOARD TEMPERATURES", classes="head")
                yield TempPlot(id="temp-plot")
        yield Footer()

    def on_mount(self) -> None:
        self._ser: Optional[serial.Serial] = None
        self._temps = [deque([float("nan")], maxlen=HIST) for _ in range(4)]
        self._read()

    @work(thread=True)
    def _read(self) -> None:
        try:
            with serial.Serial(PORT, BAUD, timeout=1.0) as ser:
                self._ser = ser
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
        finally:
            self._ser = None

    def action_toggle_motor(self) -> None:
        if self._ser is not None:
            try:
                self._ser.write(b" ")  # firmware toggles enable on space
            except serial.SerialException as e:
                self.post_message(SerialStatus(connected=False, detail=str(e)))

    def on_button_pressed(self, _: Button.Pressed) -> None:
        self.action_toggle_motor()

    def on_row_received(self, m: RowReceived) -> None:
        r = m.row
        for dq_, v in zip(self._temps, r.temps):
            dq_.append(v)

        self.query_one("#target", Label).update(f"target rpm  {r.target_rpm:6.1f}")
        self.query_one("#hall-rpm", Label).update(f"hall rpm    {r.hall_rpm:6.2f}")
        self.query_one("#enabled", Label).update(
            f"motor       {'ON' if r.enabled >= 0.5 else 'OFF'}")
        for i in range(4):
            self.query_one(f"#t{i}", Label).update(f"T{i}  {r.temps[i]:.2f} °C")

        self.query_one("#temp-plot", TempPlot).feed(self._temps)

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
