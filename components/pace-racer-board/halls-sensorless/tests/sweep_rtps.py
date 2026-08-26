#!/usr/bin/env python3
"""Sweep the RTPS publish rate and report bandwidth margin vs FOC disturbance.

For each requested rate this records:
  - what the host actually received (the honest delivered rate)
  - the board's own publish counters and per-publish cost
  - the sampler `late%`, `coalesce`, and `cmax` over the same window

The publish rate saturates at 1/publish_time, so `pub_us avg` is the number that
sets the ceiling. `overrun` counts cycles where the deadline had already passed.

Usage:
    sweep_rtps.py                          # motor idle (safe default)
    sweep_rtps.py --spin                   # spin first: run 4 A / 100 rpm
    sweep_rtps.py --spin --amps 4 --rpm 100
    sweep_rtps.py --rates 0,10,50,100,500
    sweep_rtps.py --window 10

SAFETY: --spin energises the motor. Coupled to the mag brake it needs ~4 A to
break away (at 2 A it slips). The script always sends `stop` on exit, including
on Ctrl-C.
"""
import argparse
import sys
import time

import prlib
import rtps_sub


def spin_up(amps, rpm, ramp=3.0):
    print("spinning up: run %g %g %g" % (amps, rpm, ramp))
    for line in prlib.console("run %g %g %g" % (amps, rpm, ramp), 12.0):
        print("  ", line)
    # Confirm it actually took: state should be C (closed-loop sensorless).
    rows = prlib.console("", 2.0, keep_stream=True)
    stream = [r for r in rows if not r.startswith(("#", "!"))]
    if stream:
        state = stream[-1].split(",")[1].strip() if "," in stream[-1] else "?"
        print("   state=%s  (%s)" % (state, stream[-1]))
        if state not in ("C", "S", "V"):
            print("   ! motor does not appear to be spinning")
            return False
    return True


def stop_motor():
    print("stopping motor")
    for line in prlib.console("stop", 4.0):
        print("  ", line)


def run_point(hz, window, settle=1.5):
    prlib.console("rtpshz %d" % hz, 1.0)
    prlib.rtps_stats(1.0)  # reset board counters
    prlib.foc_stats(1.0)  # reset FOC counters
    time.sleep(settle)

    pkts, decoded, total, el = rtps_sub.measure(window)
    rt = prlib.rtps_stats(2.0)
    foc = prlib.foc_stats(2.0)

    return {
        "hz_req": hz,
        "hz_rx": (pkts / el) if el > 0 else 0.0,
        "decoded": decoded,
        "pkts": pkts,
        "bytes_per_pkt": (total / pkts) if pkts else 0,
        "kBps": (total / 1024.0 / el) if el > 0 else 0.0,
        "pub_ok": rt.get("pub_ok", 0) if rt else 0,
        "pub_fail": rt.get("pub_fail", 0) if rt else 0,
        "overrun": rt.get("overrun", 0) if rt else 0,
        "pub_us_avg": rt.get("pub_us_avg", 0) if rt else 0,
        "pub_us_max": rt.get("pub_us_max", 0) if rt else 0,
        "late_pct": foc.get("late_pct") if foc else None,
        "coalesce": foc.get("coalesce") if foc else None,
        "cmax": foc.get("cmax") if foc else None,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rates", default="0,10,25,50,100,200,500,2000")
    ap.add_argument("--window", type=float, default=9.0)
    ap.add_argument("--spin", action="store_true", help="energise the motor first")
    ap.add_argument("--amps", type=float, default=4.0)
    ap.add_argument("--rpm", type=float, default=100.0)
    ap.add_argument("--writers", type=int, default=1)
    ap.add_argument("--readers", type=int, default=0)
    args = ap.parse_args()

    rates = [int(r) for r in args.rates.split(",")]

    st = prlib.rtps_stats(2.0)
    if not st or not st.get("up"):
        print("bringing up ethernet + RTPS")
        prlib.console("eth", 5.0)
        for line in prlib.console("rtps %d %d" % (args.writers, args.readers), 5.0):
            print("  ", line)

    spinning = False
    try:
        if args.spin:
            spinning = spin_up(args.amps, args.rpm)

        print()
        print("motor: %s" % ("SPINNING %g A / %g rpm" % (args.amps, args.rpm) if spinning
                             else "idle"))
        hdr = ("%6s %8s %8s %9s %9s %8s %7s %8s %7s"
               % ("req", "rx Hz", "kB/s", "pub_us", "pub_max", "overrun", "late%", "coalesce",
                  "cmax"))
        print(hdr)
        print("-" * len(hdr))

        results = []
        for hz in rates:
            r = run_point(hz, args.window)
            results.append(r)
            print("%6d %8.1f %8.2f %9d %9d %8d %7s %8s %7s"
                  % (r["hz_req"], r["hz_rx"], r["kBps"], r["pub_us_avg"], r["pub_us_max"],
                     r["overrun"],
                     ("%.1f" % r["late_pct"]) if r["late_pct"] is not None else "-",
                     ("%.2f" % r["coalesce"]) if r["coalesce"] is not None else "-",
                     ("%.0f" % r["cmax"]) if r["cmax"] is not None else "-"))

        # Ceiling is set by per-publish cost, not by the requested rate.
        best = max(results, key=lambda r: r["hz_rx"])
        avg_us = max((r["pub_us_avg"] for r in results if r["pub_us_avg"]), default=0)
        print()
        print("peak delivered: %.1f Hz" % best["hz_rx"])
        if avg_us:
            print("implied ceiling from publish cost: %.1f Hz (1 / %d us)"
                  % (1e6 / avg_us, avg_us))
        print("margin over the 10 Hz telemetry target: %.1fx" % (best["hz_rx"] / 10.0))
    finally:
        prlib.console("rtpshz 10", 1.0)
        if spinning:
            stop_motor()


if __name__ == "__main__":
    sys.exit(main())
