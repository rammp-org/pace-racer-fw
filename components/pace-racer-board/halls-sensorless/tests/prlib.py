#!/usr/bin/env python3
"""Shared helpers for the PACE RACER ethernet / RTPS test scripts.

Everything here talks to a board running the halls-sensorless firmware over the
USB console, or to its ethernet interface over the dock link.
"""
import re
import subprocess
import sys
import time

import serial

# --- board / link configuration ---------------------------------------------

PORT = "/dev/cu.usbmodem2101"
BAUD = 115200

BOARD_IP = "192.168.50.50"
IFACE = "en7"  # the USB dock's ethernet port

RTPS_DOMAIN = 0
# DDSI-RTPS port maths: PB + DG*domain + dN
SPDP_MCAST_PORT = 7400 + 250 * RTPS_DOMAIN + 0
USER_MCAST_PORT = 7400 + 250 * RTPS_DOMAIN + 1
MCAST_GROUP = "239.255.0.1"

UDP_PORT = 3333


def host_addr(iface=IFACE, require_subnet=True):
    """The dock interface's IPv4 address.

    The static address is applied with `ifconfig` and does NOT persist: macOS
    drops it whenever it reconfigures the interface, at which point the port
    falls back to a self-assigned 169.254 address. Re-apply with:

        sudo ifconfig en7 inet 192.168.50.1 netmask 255.255.255.0
    """
    out = subprocess.run(["ifconfig", iface], capture_output=True, text=True).stdout
    if require_subnet:
        m = re.search(r"inet (192\.168\.50\.\d+)", out)
        if not m:
            sys.exit(
                "no 192.168.50.x on %s — re-apply the static address:\n"
                "  sudo ifconfig %s inet 192.168.50.1 netmask 255.255.255.0" % (iface, iface)
            )
        return m.group(1)
    m = re.search(r"inet (\d+\.\d+\.\d+\.\d+)", out)
    if not m:
        sys.exit("no IPv4 on %s" % iface)
    return m.group(1)


# --- console ----------------------------------------------------------------


def strip_ansi(line):
    return line.replace("\x1b[32m", "").replace("\x1b[0m", "").strip()


def console(cmd, read_s=2.0, port=PORT, keep_stream=False):
    """Send a console command; return the firmware's '#'/'!' reply lines.

    Set keep_stream=True to also get the streamed CSV rows.
    """
    s = serial.Serial(port, BAUD, timeout=0.2)
    time.sleep(0.3)
    s.reset_input_buffer()
    if cmd:
        s.write((cmd + "\n").encode())
        s.flush()
    end = time.time() + read_s
    buf = b""
    while time.time() < end:
        buf += s.read(4096)
    s.close()

    out = []
    for raw in buf.decode("utf-8", "replace").splitlines():
        line = strip_ansi(raw)
        if not line:
            continue
        if line.startswith("#") or line.startswith("!"):
            out.append(line)
        elif keep_stream:
            out.append(line)
    return out


def console_first(cmd, prefix, read_s=2.0):
    """Send a command and return the first reply line starting with `prefix`."""
    for line in console(cmd, read_s):
        if line.startswith(prefix):
            return line
    return None


def parse_kv(line):
    """Parse the firmware's '#tag k=v k=v' reply format into a dict.

    Values that look numeric are converted; '(4.9%)' style extras are ignored.
    """
    out = {}
    for token in line.split():
        if "=" not in token:
            continue
        k, _, v = token.partition("=")
        try:
            out[k] = int(v)
        except ValueError:
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


def foc_stats(read_s=2.0):
    """Read and RESET the sampler/FOC statistics. Returns a dict.

    Call once to zero the counters, wait the measurement window, then call again
    to get the stats for that window.
    """
    line = console_first("s", "#stats", read_s)
    if not line:
        return None
    out = parse_kv(line)
    out["raw"] = line
    # late% is printed as "late=NNN (X.Y%)" so the percentage needs its own parse
    m = re.search(r"late=(\d+) \(([\d.]+)%\)", line)
    if m:
        out["late"] = int(m.group(1))
        out["late_pct"] = float(m.group(2))
    m = re.search(r"isr=([\d.]+)/([\d.]+)/([\d.]+)us", line)
    if m:
        out["isr_min"], out["isr_avg"], out["isr_max"] = (float(g) for g in m.groups())
    m = re.search(r"coalesce=([\d.]+)", line)
    if m:
        out["coalesce"] = float(m.group(1))
    m = re.search(r"cmax=([\d.]+)us", line)
    if m:
        out["cmax"] = float(m.group(1))
    return out


def rtps_stats(read_s=2.0):
    """Read and RESET the RTPS publisher statistics. Returns a dict."""
    line = console_first("rtpstat", "#rtpstat", read_s)
    if not line:
        return None
    out = parse_kv(line)
    out["raw"] = line
    m = re.search(r"pub_us=(\d+)/(\d+)/(\d+)", line)
    if m:
        out["pub_us_min"], out["pub_us_avg"], out["pub_us_max"] = (int(g) for g in m.groups())
    m = re.search(r"rx=(\d+)smp/(\d+)B", line)
    if m:
        out["rx_samples"], out["rx_bytes"] = int(m.group(1)), int(m.group(2))
    return out


def measure_foc(window_s=10.0):
    """Reset the FOC counters, wait `window_s`, return the stats for that window."""
    foc_stats(read_s=1.0)  # reset
    time.sleep(window_s)
    return foc_stats(read_s=2.0)
