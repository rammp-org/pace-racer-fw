#!/usr/bin/env python3
"""Report control-quality statistics from the telemetry stream.

`late%` only says samples were taken in a bad window; it does not say the loop
misbehaved. These are the numbers that answer that directly: how steady speed and
torque current are, and how well the observer tracks the halls.

Usage: foc_quality.py <label> <seconds>
"""
import statistics as st
import sys
import time

import serial

import prlib


def main():
    label = sys.argv[1]
    dur = float(sys.argv[2])
    s = serial.Serial(prlib.PORT, prlib.BAUD, timeout=1)
    time.sleep(0.3)
    s.reset_input_buffer()
    end = time.time() + dur
    buf = b""
    while time.time() < end:
        buf += s.read(4096)
    s.close()

    iq, rpm_est, rpm_hall, aerr, flux = [], [], [], [], []
    for line in buf.decode("utf-8", "replace").splitlines():
        line = prlib.strip_ansi(line)
        if not line or line[0] in "#!IWE":
            continue
        p = [x.strip() for x in line.split(",")]
        if len(p) < 12 or p[1] not in ("C", "S", "V", "H", "X"):
            continue
        try:
            iq.append(float(p[3]))
            aerr.append(float(p[7]))
            rpm_est.append(float(p[9]))
            rpm_hall.append(float(p[10]))
            flux.append(float(p[11]))
        except ValueError:
            continue

    if len(rpm_hall) < 10:
        print("%-22s INSUFFICIENT SAMPLES (%d)" % (label, len(rpm_hall)))
        return
    track = [e - h for e, h in zip(rpm_est, rpm_hall)]
    print("%-22s n=%3d | rpm %6.2f sd %.3f | iq %5.3f sd %.4f | |aerr| %5.2f | "
          "obs_err sd %.3f | rows/s %.1f"
          % (label, len(rpm_hall), st.mean(rpm_hall), st.pstdev(rpm_hall), st.mean(iq),
             st.pstdev(iq), st.mean([abs(a) for a in aerr]), st.pstdev(track),
             len(rpm_hall) / dur))


if __name__ == "__main__":
    main()
