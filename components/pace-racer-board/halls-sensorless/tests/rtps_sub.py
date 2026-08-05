#!/usr/bin/env python3
"""Receive RTPS user-data telemetry from the board and report the arrival rate.

Joins the RTPS user multicast group on the dock interface, parses the RTPS
header + DATA submessage far enough to reach the CDR payload, and decodes the
telemetry sample. Decoding (rather than just counting bytes) verifies the values
survive the round trip.

Usage:
    rtps_sub.py <seconds> [--decode] [--quiet]

Exposes measure() so the sweep scripts can reuse it.
"""
import socket
import struct
import sys
import time

import prlib


def parse_data_payload(pkt):
    """Return the CDR payload of the first DATA submessage, or None."""
    if len(pkt) < 20 or pkt[:4] != b"RTPS":
        return None
    off = 20  # RTPS header: magic(4) version(2) vendor(2) guidPrefix(12)
    while off + 4 <= len(pkt):
        sub_id = pkt[off]
        flags = pkt[off + 1]
        endian = "<" if (flags & 0x01) else ">"
        (sub_len,) = struct.unpack_from(endian + "H", pkt, off + 2)
        body = off + 4
        if sub_id == 0x15:  # DATA
            if body + 20 > len(pkt):
                return None
            (octets_to_iqos,) = struct.unpack_from(endian + "H", pkt, body + 2)
            # octetsToInlineQos is measured from the start of readerId
            payload_off = body + 4 + octets_to_iqos
            end = body + sub_len if sub_len else len(pkt)
            if payload_off < end <= len(pkt):
                return pkt[payload_off:end]
            return None
        if sub_len == 0:
            break
        off = body + sub_len
    return None


def decode(cdr):
    """Decode the telemetry sample.

    Layout: 4-byte CDR encapsulation, float seconds, uint8 state, 3 bytes pad
    (CDR aligns the next float to 4), then 11 floats and 4 temperatures.
    """
    if len(cdr) < 8:
        return None
    body = cdr[4:]
    try:
        (seconds,) = struct.unpack_from("<f", body, 0)
        state = body[4]
        vals = struct.unpack_from("<15f", body, 8)
        return {
            "seconds": seconds,
            "state": chr(state),
            "id": vals[0], "iq": vals[1], "iqref": vals[2],
            "vd": vals[3], "vq": vals[4], "aerr": vals[5],
            "rpm_drive": vals[6], "rpm_est": vals[7], "rpm_hall": vals[8],
            "flux": vals[9],
            "temps": vals[10:14],
        }
    except struct.error:
        return None


def measure(duration, show=0):
    """Listen for `duration` seconds. Returns (pkts, decoded, bytes, elapsed)."""
    local = prlib.host_addr(require_subnet=False)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    s.bind(("", prlib.USER_MCAST_PORT))
    mreq = socket.inet_aton(prlib.MCAST_GROUP) + socket.inet_aton(local)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    s.settimeout(0.5)

    pkts = decoded = total = shown = 0
    t0 = None
    end = time.time() + duration
    while time.time() < end:
        try:
            data, _ = s.recvfrom(4096)
        except socket.timeout:
            continue
        if t0 is None:
            t0 = time.time()
        pkts += 1
        total += len(data)
        payload = parse_data_payload(data)
        if payload:
            d = decode(payload)
            if d:
                decoded += 1
                if shown < show:
                    print("  t=%.2f state=%s iq=%.2f rpm_est=%.0f t0=%.1f"
                          % (d["seconds"], d["state"], d["iq"], d["rpm_est"], d["temps"][0]))
                    shown += 1
    s.close()
    return pkts, decoded, total, ((time.time() - t0) if t0 else 0.0)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10
    show = 3 if "--decode" in sys.argv else 0
    pkts, decoded, total, el = measure(dur, show)
    if el <= 0:
        print("RTPS user data: nothing arrived")
        return
    if "--quiet" not in sys.argv:
        print("RTPS user data: %d pkts (%d decoded) %d B in %.2fs -> %.1f Hz, %.1f kB/s, %.0f B/pkt"
              % (pkts, decoded, total, el, pkts / el, total / 1024.0 / el, total / pkts))


if __name__ == "__main__":
    main()
