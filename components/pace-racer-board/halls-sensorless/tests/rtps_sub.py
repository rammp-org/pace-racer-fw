#!/usr/bin/env python3
"""Receive RTPS user-data telemetry from the board and report the arrival rate.

!! DOES NOT WORK ON THE espp v1.2.0 ENGINE -- USE tests/host_sub/ INSTEAD !!

This is a PASSIVE MULTICAST SNIFFER: it joins the user multicast group and waits.
That worked on the pre-v1.2.0 engine, whose `use_multicast_for_user_data` mode
sprayed user data at the group with no reader required. v1.2.0 removed it and
transmits ONLY to matched readers -- and this script never announces itself, so
it is never discovered, so nothing is ever sent to it. The delivered rate always
reads 0.0 Hz, and so does the `rx Hz` column of sweep_rtps.py / sweep_endpoints.py,
which call measure() below.

Verified on hardware 2026-08-25: user data is NOT multicast even with a real
subscriber attached and matched -- the board unicasts to that reader's own
locator. So no amount of listening here will help.

decode() is ALSO wrong independently of the above: it assumes 15 contiguous
floats then a trailing uint8 state, but the wire puts `state` in a 4-byte slot at
offset 4, shifting every later field. Read as written it silently returns `flux`
as temps[0] and a padding byte as `state`. Left unfixed because the script cannot
receive anything to decode; tests/host_sub/ uses the Sample struct directly and
lets espp's own reflection handle the layout, so it cannot drift.

Kept for reference and for the RTPS header/DATA parsing, which is still correct.

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

    Layout (espp v1.2.0 typed publisher, XCDR1, field order = the Sample struct
    in main/rtps_telem.hpp): 4-byte CDR encapsulation, then 15 floats (seconds,
    id, iq, iqref, vd, vq, aerr, rpm_drive, rpm_est, rpm_hall, flux, temps[4])
    and a trailing uint8 state.
    """
    if len(cdr) < 4 + 15 * 4 + 1:
        return None
    body = cdr[4:]
    try:
        vals = struct.unpack_from("<15f", body, 0)
        state = body[60]
        return {
            "seconds": vals[0],
            "state": chr(state),
            "id": vals[1], "iq": vals[2], "iqref": vals[3],
            "vd": vals[4], "vq": vals[5], "aerr": vals[6],
            "rpm_drive": vals[7], "rpm_est": vals[8], "rpm_hall": vals[9],
            "flux": vals[10],
            "temps": vals[11:15],
        }
    except (struct.error, IndexError):
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
        print("  This script CANNOT receive on the espp v1.2.0 engine -- it is a passive")
        print("  multicast sniffer, and v1.2.0 unicasts user data only to matched readers.")
        print("  Use tests/host_sub/ for any delivered-rate measurement. See the module")
        print("  docstring and tests/host_sub/README.md.")
        return
    if "--quiet" not in sys.argv:
        print("RTPS user data: %d pkts (%d decoded) %d B in %.2fs -> %.1f Hz, %.1f kB/s, %.0f B/pkt"
              % (pkts, decoded, total, el, pkts / el, total / 1024.0 / el, total / pkts))


if __name__ == "__main__":
    main()
