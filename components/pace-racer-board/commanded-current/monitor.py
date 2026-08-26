#!/usr/bin/env python3
"""
PACE RACER commanded-current TUI.

Talks to the commanded-current firmware over the USB console:
    %time(s), target_a, i_a, u_v, enabled
Type a current and press Enter, or nudge with the arrow keys. Space toggles
the motor (disable zeroes the target). Alert lines from the firmware ('!...')
appear in the status bar.

Usage:
    python monitor.py [port [baud]]
    python monitor.py /dev/cu.usbmodem1101
"""

import sys
from collections import deque
from dataclasses import dataclass
from typing import Optional

import serial
from textual import work
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.message import Message
from textual.widgets import Button, Footer, Header, Input, Label, Rule, Static
from textual_plotext import PlotextPlot

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem1101"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
HIST = 250  # 25 Hz stream -> 10 s window
NCOLS = 5  # lockstep with the CSV header in commanded_current.cpp
STEP_A = 0.5  # arrow-key increment
MAX_A = 40.0  # keep in lockstep with kMaxTargetAmps in commanded_current.cpp


@dataclass
class Row:
    t: float
    target: float
    i: float
    u: float
    enabled: bool


def parse(line: str) -> Optional[Row]:
    s = line.strip()
    if not s or s.startswith("%") or s.startswith("#"):
        return None
    parts = s.split(",")
    if len(parts) != NCOLS:
        return None
    try:
        f = [float(p) for p in parts]
    except ValueError:
        return None
    return Row(f[0], f[1], f[2], f[3], f[4] >= 0.5)


class RowReceived(Message):
    def __init__(self, row: Row) -> None:
        super().__init__()
        self.row = row


class AlertReceived(Message):
    def __init__(self, text: str) -> None:
        super().__init__()
        self.text = text


class SerialStatus(Message):
    def __init__(self, connected: bool, detail: str = "") -> None:
        super().__init__()
        self.connected = connected
        self.detail = detail


class CurrentPlot(PlotextPlot):
    def on_mount(self) -> None:
        self.plt.theme("dark")
        self.plt.ylabel("A")

    def feed(self, target, measured) -> None:
        self.plt.clear_data()
        self.plt.plot(list(target)[::2], label="target", color="gray")
        self.plt.plot(list(measured)[::2], label="measured", color="cyan")
        self.refresh()


class Monitor(App):
    # priority=False: App bindings are priority-by-default in Textual and would
    # steal keys from the Input while typing ("10" -> the 0 fired action_zero).
    BINDINGS = [
        Binding("space", "toggle_motor", "Toggle motor", priority=False),
        Binding("up", "nudge(1)", f"+{STEP_A} A", priority=False),
        Binding("down", "nudge(-1)", f"-{STEP_A} A", priority=False),
        Binding("0", "zero", "Target 0 A", priority=False),
    ]

    CSS = """
    Screen { background: $surface; }
    #status { height: 1; background: $primary; color: $text; padding: 0 1; }
    #status.err { background: $error; }
    #alert { height: 1; color: $warning; padding: 0 1; }
    Horizontal { height: 1fr; }
    #left  { width: 44; padding: 1 2; border-right: solid $primary; }
    #right { width: 1fr; padding: 1 2; }
    .head { color: $accent; text-style: bold; margin-bottom: 1; }
    .val  { height: 1; }
    CurrentPlot { height: 1fr; margin-top: 1; }
    Rule { margin: 1 0; color: $primary; }
    #amps { margin-top: 1; }
    #toggle { margin-top: 1; width: 100%; }
    """

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        yield Static(f"connecting  {PORT}  {BAUD} baud", id="status")
        yield Static("", id="alert")
        with Horizontal():
            with Vertical(id="left"):
                yield Static("COMMAND", classes="head")
                yield Label("target    ---", id="target", classes="val")
                yield Label("measured  ---", id="measured", classes="val")
                yield Label("drive     ---", id="drive", classes="val")
                yield Label("motor     ---", id="enabled", classes="val")
                yield Input(placeholder="amps, Enter to send", id="amps", type="number")
                yield Button("Toggle Motor (space)", id="toggle", variant="warning")
                yield Rule()
                yield Static(f"↑/↓ nudge ±{STEP_A} A · 0 zeroes target", classes="val")
            with Vertical(id="right"):
                yield Static("PHASE CURRENT (A→B)", classes="head")
                yield CurrentPlot(id="plot")
        yield Footer()

    def on_mount(self) -> None:
        self._ser: Optional[serial.Serial] = None
        self._target = deque([0.0], maxlen=HIST)
        self._measured = deque([0.0], maxlen=HIST)
        self._last_target = 0.0
        self._last_row: Optional[Row] = None
        # Rows only buffer data; the plot/labels redraw at 4 Hz with decimated
        # points. The full plotext canvas render is expensive enough to starve
        # the input loop if done per-row or too often.
        self.set_interval(0.25, self._refresh_ui)
        self._read()

    @work(thread=True)
    def _read(self) -> None:
        try:
            with serial.Serial(PORT, BAUD, timeout=1.0, write_timeout=0.5) as ser:
                self._ser = ser
                self.post_message(SerialStatus(connected=True))
                while True:
                    try:
                        line = ser.readline().decode("utf-8", errors="replace")
                    except serial.SerialException as e:
                        self.post_message(SerialStatus(connected=False, detail=str(e)))
                        return
                    if line.lstrip().startswith("!"):
                        self.post_message(AlertReceived(line.strip()))
                        continue
                    row = parse(line)
                    if row:
                        self.post_message(RowReceived(row))
        except serial.SerialException as e:
            self.post_message(SerialStatus(connected=False, detail=str(e)))
        finally:
            self._ser = None

    def _send(self, data: bytes) -> None:
        if self._ser is None:
            return
        try:
            self._ser.write(data)
        except serial.SerialTimeoutException:
            # firmware isn't draining stdin — surface it, never hang the UI
            self.query_one("#alert", Static).update("! write timeout — firmware not reading stdin")
        except serial.SerialException as e:
            self.post_message(SerialStatus(connected=False, detail=str(e)))

    def _send_target(self, amps: float) -> None:
        amps = max(-MAX_A, min(MAX_A, amps))
        self._last_target = amps
        self._send(f"i {amps:.3f}\n".encode())

    def action_toggle_motor(self) -> None:
        self._send(b" ")

    def action_nudge(self, direction: int) -> None:
        self._send_target(self._last_target + direction * STEP_A)

    def action_zero(self) -> None:
        self._send_target(0.0)

    def on_button_pressed(self, _: Button.Pressed) -> None:
        self.action_toggle_motor()

    def on_input_submitted(self, event: Input.Submitted) -> None:
        try:
            self._send_target(float(event.value))
        except ValueError:
            pass
        event.input.value = ""
        self.set_focus(None)  # so space/arrows act on the motor right after Enter

    def on_row_received(self, m: RowReceived) -> None:
        r = m.row
        self._last_target = r.target
        self._target.append(r.target)
        self._measured.append(r.i)
        self._last_row = r

    def _refresh_ui(self) -> None:
        r = self._last_row
        if r is None:
            return
        self.query_one("#target", Label).update(f"target    {r.target:6.2f} A")
        self.query_one("#measured", Label).update(f"measured  {r.i:6.2f} A")
        self.query_one("#drive", Label).update(f"drive     {r.u:6.2f} V")
        self.query_one("#enabled", Label).update(
            f"motor     {'ON' if r.enabled else 'OFF'}")
        self.query_one("#plot", CurrentPlot).feed(self._target, self._measured)

    def on_alert_received(self, m: AlertReceived) -> None:
        self.query_one("#alert", Static).update(m.text)

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
