#!/usr/bin/env python3
"""Drain UDP from the board and report the arrival rate.

Sends a probe first: the board learns its peer from the first datagram it
receives, so a probe from any other socket would point the stream at a port
nothing is listening on and every packet would be discarded.

Usage: udp_recv.py <seconds>
"""
import socket
import sys
import time

import prlib


def main():
    dur = float(sys.argv[1])
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    s.bind((prlib.host_addr(), prlib.UDP_PORT))
    s.settimeout(0.5)
    s.sendto(b"probe", (prlib.BOARD_IP, prlib.UDP_PORT))
    time.sleep(0.2)

    pkts = total = 0
    t0 = None
    end = time.time() + dur
    while time.time() < end:
        try:
            d, _ = s.recvfrom(2048)
        except socket.timeout:
            continue
        if t0 is None:
            t0 = time.time()
        pkts += 1
        total += len(d)
    s.close()
    el = (time.time() - t0) if t0 else 0
    if el > 0:
        print("HOST rx: %d pkts, %d B in %.2fs -> %.0f pps, %.1f kB/s"
              % (pkts, total, el, pkts / el, total / 1024.0 / el))
    else:
        print("HOST rx: nothing arrived")


if __name__ == "__main__":
    main()
