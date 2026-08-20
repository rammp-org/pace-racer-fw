"""Stdlib-only telemetry model for the halls-sensorless CSV stream.

Kept free of runtime dependencies (pyserial/textual) so test_parse.py can run
anywhere; monitor.py layers the TUI on top of this.
"""

import math
from dataclasses import dataclass
from typing import Optional

VBUS = 48.0  # must match kBusVoltage in sensorless.cpp
NCOLS = 16  # keep in lockstep with the CSV header in sensorless.cpp


@dataclass
class Row:
    t: float
    mode: str
    id_a: float
    iq: float
    iqref: float
    vd: float
    vq: float
    aerr: float
    rpm_drive: float
    rpm_est: float
    rpm_hall: float
    flux: float
    temps: tuple  # (t0, t1, t2, t3)

    @property
    def p_w(self) -> float:
        return 1.5 * (self.vd * self.id_a + self.vq * self.iq)

    @property
    def ibus(self) -> float:
        return self.p_w / VBUS

    @property
    def vmag(self) -> float:
        return math.hypot(self.vd, self.vq)


def parse(line: str) -> Optional[Row]:
    s = line.strip()
    if not s or s.startswith("%") or s.startswith("#") or s.startswith("!"):
        return None
    parts = s.split(",")
    if len(parts) != NCOLS:
        return None
    mode = parts[1].strip()
    if len(mode) != 1:
        return None
    try:
        f = [float(p) for p in parts[:1] + parts[2:]]
    except ValueError:
        return None
    return Row(f[0], mode, *f[1:11], tuple(f[11:15]))
