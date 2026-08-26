#!/usr/bin/env python3
"""Verify RTPS SPDP discovery announcements reach the host.

The first thing to run when RTPS looks broken: if SPDP arrives, the participant
is alive and the multicast path works, so any problem is further up (endpoint
matching, addressing) rather than in discovery.

Multicast DOES work over the direct dock link, including across a subnet
mismatch — IGMP snooping concerns apply to switched networks, not a direct cable.

Usage: spdp_sniff.py [seconds]
"""
import socket
import sys
import time

import prlib


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10
    local = prlib.host_addr(require_subnet=False)

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", prlib.SPDP_MCAST_PORT))
    mreq = socket.inet_aton(prlib.MCAST_GROUP) + socket.inet_aton(local)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    s.settimeout(1.0)
    print("listening on %s:%d via %s" % (prlib.MCAST_GROUP, prlib.SPDP_MCAST_PORT, local))

    seen = 0
    end = time.time() + dur
    while time.time() < end:
        try:
            data, addr = s.recvfrom(4096)
        except socket.timeout:
            continue
        seen += 1
        if seen <= 3:
            print("  from %s:%d  %d B  magic=%s ver=%d.%d vendor=%02x%02x guid=%s"
                  % (addr[0], addr[1], len(data), data[:4].decode("ascii", "replace"),
                     data[4], data[5], data[6], data[7], data[8:20].hex()))
    s.close()
    print("SPDP packets received: %d in %.0fs" % (seen, dur))
    if seen == 0:
        print("  -> no announcements. Check 'rtpstat up=1', and that the board has an IP.")


if __name__ == "__main__":
    main()
