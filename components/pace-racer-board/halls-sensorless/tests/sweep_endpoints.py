#!/usr/bin/env python3
"""Measure how RTPS cost scales with the number of writers and readers.

Each writer is a separate endpoint: publish() does its own topic lookup, message
build, and send per writer, so N writers cost roughly N times one. Measured
2026-08-04: ~11.4 ms per writer per cycle, near-perfectly linear to 8 writers.

READER LIMITATION — read before trusting the reader columns. The intent was for
readers to consume the board's own multicast publications, avoiding the need for
a second node. That does NOT work: espp's socket layer can set IP_MULTICAST_LOOP,
but the RTPS component never enables it, so the board does not receive its own
traffic and `rx_smp` stays 0. What the reader sweep therefore measures is the
cost of IDLE readers (answer: nothing measurable), not inbound processing.

To measure real inbound cost you need an external RTPS writer the board can
discover — samples are routed by writer GUID via SEDP, so raw DATA packets from
an unannounced writer are dropped. That means implementing SPDP + SEDP + DATA on
the host, or running a second board / a real DDS stack.

Endpoint counts are fixed at participant start, so each point needs a reboot.
The board is reset between points by reflashing, which is the only reliable way
to restart the participant.

Usage:
    sweep_endpoints.py --writers 1,2,4,8            # writer scaling
    sweep_endpoints.py --readers 0,1,2,4            # reader scaling
    sweep_endpoints.py --writers 1,2,4 --hz 50
    sweep_endpoints.py --no-reflash                 # if you reset the board yourself

Requires ESP-IDF on PATH for the reflash step (idf.py).
"""
import argparse
import os
import subprocess
import sys
import time

import prlib
import rtps_sub

APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def reflash():
    """Reflash to reset the board — the participant cannot be restarted in place."""
    r = subprocess.run(["idf.py", "-p", prlib.PORT, "flash"], cwd=APP_DIR,
                       capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-2000:])
        sys.exit("reflash failed — is ESP-IDF exported? (. $IDF_PATH/export.sh)")
    time.sleep(4)


def run_point(writers, readers, hz, window, do_reflash):
    if do_reflash:
        reflash()
    prlib.console("eth", 5.0)
    prlib.console("rtps %d %d" % (writers, readers), 5.0)
    prlib.console("rtpshz %d" % hz, 1.0)
    prlib.rtps_stats(1.0)
    prlib.foc_stats(1.0)
    time.sleep(1.5)

    pkts, decoded, total, el = rtps_sub.measure(window)
    rt = prlib.rtps_stats(2.0)
    foc = prlib.foc_stats(2.0)

    return {
        "writers": writers,
        "readers": readers,
        "hz_rx": (pkts / el) if el > 0 else 0.0,
        "kBps": (total / 1024.0 / el) if el > 0 else 0.0,
        "pub_ok": rt.get("pub_ok", 0) if rt else 0,
        "pub_us_avg": rt.get("pub_us_avg", 0) if rt else 0,
        "pub_us_max": rt.get("pub_us_max", 0) if rt else 0,
        "rx_samples": rt.get("rx_samples", 0) if rt else 0,
        "late_pct": foc.get("late_pct") if foc else None,
        "coalesce": foc.get("coalesce") if foc else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--writers", default="1")
    ap.add_argument("--readers", default="0")
    ap.add_argument("--hz", type=int, default=50)
    ap.add_argument("--window", type=float, default=9.0)
    ap.add_argument("--no-reflash", action="store_true")
    args = ap.parse_args()

    writer_counts = [int(w) for w in args.writers.split(",")]
    reader_counts = [int(r) for r in args.readers.split(",")]

    hdr = ("%3s %3s %8s %8s %9s %9s %9s %7s %8s"
           % ("W", "R", "rx Hz", "kB/s", "pub_ok", "pub_us", "pub_max", "late%", "rx_smp"))
    print("requested rate: %d Hz" % args.hz)
    print(hdr)
    print("-" * len(hdr))

    for w in writer_counts:
        for r in reader_counts:
            p = run_point(w, r, args.hz, args.window, not args.no_reflash)
            print("%3d %3d %8.1f %8.2f %9d %9d %9d %7s %8d"
                  % (p["writers"], p["readers"], p["hz_rx"], p["kBps"], p["pub_ok"],
                     p["pub_us_avg"], p["pub_us_max"],
                     ("%.1f" % p["late_pct"]) if p["late_pct"] is not None else "-",
                     p["rx_samples"]))

    print()
    print("pub_us is per CYCLE (all writers), so it scales ~linearly with W.")
    print("rx_smp will be 0: the RTPS component does not enable IP_MULTICAST_LOOP, so the")
    print("board never receives its own publications. The reader columns therefore show the")
    print("cost of IDLE readers, not inbound processing — see the module docstring.")


if __name__ == "__main__":
    sys.exit(main())
