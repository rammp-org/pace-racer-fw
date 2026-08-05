#!/usr/bin/env python3
"""Flood the board with UDP to load the ethernet RX path.

Used to characterise how inbound traffic disturbs the control loop. Disturbance
scales with PACKET rate, not bandwidth, so sweep pps rather than payload size.

Usage: udp_load.py <pps> <bytes> <seconds>
"""
import socket
import sys
import time

import prlib


def main():
    pps, size, dur = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3])
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((prlib.host_addr(), 0))
    s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 20)
    payload = b"\x5a" * size

    interval = 1.0 / pps
    sent = 0
    t0 = time.perf_counter()
    nxt = t0
    end = t0 + dur
    while True:
        now = time.perf_counter()
        if now >= end:
            break
        if now >= nxt:
            try:
                s.sendto(payload, (prlib.BOARD_IP, prlib.UDP_PORT))
                sent += 1
            except OSError:
                pass
            nxt += interval
            if nxt < now:  # fell behind: resync rather than spiral
                nxt = now + interval
        else:
            time.sleep(0)
    el = time.perf_counter() - t0
    s.close()
    print("host sent %d pkts x %dB in %.2fs = %.0f pps, %.2f Mbit/s"
          % (sent, size, el, sent / el, sent * size * 8 / el / 1e6))


if __name__ == "__main__":
    main()
