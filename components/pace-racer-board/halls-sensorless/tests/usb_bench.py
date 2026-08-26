#!/usr/bin/env python3
"""Benchmark the USB console as a telemetry transport, for comparison with RTPS.

Runs the firmware's `usbtx` flood and measures it host-side too. USB-Serial-JTAG
writes block once the host stops draining, so both numbers describe the same
end-to-end path and should agree.

USB is bandwidth-limited (~140-180 kB/s) but sustains thousands of small
messages/s; ethernet is the opposite (packet-rate limited, ~250 pps).

Usage: usb_bench.py <seconds> <bytes>
"""
import sys
import time

import serial

import prlib


def main():
    secs, blen = int(sys.argv[1]), int(sys.argv[2])
    s = serial.Serial(prlib.PORT, prlib.BAUD, timeout=0.5)
    time.sleep(0.3)
    s.reset_input_buffer()
    s.write(("usbtx %d %d\n" % (secs, blen)).encode())
    s.flush()

    total = 0
    t_start = None
    tail = b""
    end = time.time() + secs + 6
    while time.time() < end:
        chunk = s.read(65536)
        if not chunk:
            continue
        if t_start is None:
            t_start = time.time()
        total += len(chunk)
        tail = (tail + chunk)[-4000:]
        if b"#usbtx done" in tail:
            break
    el = (time.time() - t_start) if t_start else 0
    s.close()

    for line in tail.decode("utf-8", "replace").splitlines():
        line = prlib.strip_ansi(line)
        if line.startswith("#usbtx"):
            print("FW  :", line)
    if el > 0:
        print("HOST: received %d bytes in %.2fs -> %.1f kB/s" % (total, el, total / 1024.0 / el))


if __name__ == "__main__":
    main()
