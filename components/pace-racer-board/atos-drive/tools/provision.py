#!/usr/bin/env python3
"""Plug a laptop into a PACE RACER board and make it a flatbot wheel.

    tools/provision.py                # one board on USB: flash + axis + fb hall, hcal if needed
    tools/provision.py --axis 3       # override the roster
    tools/provision.py --no-flash     # NVS only, keep whatever firmware is on it
    tools/provision.py --hcal         # redo the hall table even if one is saved
    tools/provision.py --list         # just say which boards are plugged in

Steps, each reported as it goes:
  1. find the board: every /dev/cu.usbmodem* is asked for its MAC with esptool.
     Boards on the blocklist below are never touched. With several boards
     plugged in, pass --port.
  2. flash the build in --build (default build-flatbot; run idf.py there first).
  3. over the console: axis <n>, fb hall, then a reset so both take effect.
  4. if the hall table is not saved (or --hcal): the calibration spin. Wheel
     off the ground, motor supply on. The wheel turns both ways for ~15 s each.
  5. read back 'axis' and 'fb' and print the result line.

Needs pyserial and esptool. The ESP-IDF python env has both; the script finds
it under ~/.espressif when the system python lacks them.
"""
import argparse
import glob
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.dirname(HERE)

# Boards by base MAC (what esptool read-mac prints; the ethernet MAC is the
# same on this board). Add a line when a new board joins the robot.
ROSTER = {
    "10:bd:a3:66:15:94": 2,  # drive_left
    "10:bd:a3:64:ee:fc": 3,  # drive_right
}
AXES = {0: "front_caster_left", 1: "front_caster_right", 2: "drive_left", 3: "drive_right"}
# Never flash these: other projects' boards that share the laptop.
BLOCKLIST = {"d8:3b:da:4d:ab:b0"}

HCAL_SECS = 6      # per direction, rotating ticks only
SPIN_RPM = 30
SPIN_AMPS = 3
SPIN_RAMP_S = 5


def say(msg):
    print(f"== {msg}", flush=True)


def die(msg):
    print(f"!! {msg}", file=sys.stderr, flush=True)
    sys.exit(1)


# ---------------------------------------------------------------- tooling
def find_python():
    """A python that imports esptool and serial: this one, else the IDF venv."""
    for exe in [sys.executable] + sorted(glob.glob(os.path.expanduser("~/.espressif/python_env/*/bin/python"))):
        r = subprocess.run([exe, "-c", "import esptool, serial"], capture_output=True)
        if r.returncode == 0:
            return exe
    die("no python with esptool + pyserial; run 'pip install esptool pyserial' or install ESP-IDF")


PY = None


def esptool(port, *args, timeout=120):
    cmd = [PY, "-m", "esptool", "--chip", "esp32s3", "--port", port, *args]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return r.returncode, r.stdout + r.stderr


def read_mac(port):
    rc, out = esptool(port, "--after", "hard-reset", "read-mac", timeout=30)
    m = re.search(r"MAC:\s*([0-9a-f:]{17})", out)
    return m.group(1).lower() if rc == 0 and m else None


def candidate_ports():
    return sorted(glob.glob("/dev/cu.usbmodem*"))


# ---------------------------------------------------------------- console
class Console:
    def __init__(self, port):
        import serial
        self.s = serial.Serial(port, 115200, timeout=0.2)
        time.sleep(0.3)
        self.s.write(b"sq 1\n")
        time.sleep(0.4)
        self.s.reset_input_buffer()

    def close(self):
        self.s.close()

    def cmd(self, line, wait=1.0):
        # No input flush: the board prints results such as '#hcal OK' on its
        # own schedule and they must not be thrown away between commands.
        self.s.write((line + "\n").encode())
        t0 = time.time()
        buf = b""
        while time.time() - t0 < wait:
            d = self.s.read(4096)
            if d:
                buf += d
        return buf.decode("utf-8", "replace")

    def listen(self, secs):
        t0 = time.time()
        buf = b""
        while time.time() - t0 < secs:
            d = self.s.read(4096)
            if d:
                buf += d
        return buf.decode("utf-8", "replace")


def first(text, prefix):
    for ln in text.splitlines():
        if ln.startswith(prefix):
            return ln.strip()
    return ""


def wait_for_console(port, secs=8):
    """After a reset the USB-Serial-JTAG port disappears and comes back."""
    t0 = time.time()
    while time.time() - t0 < secs:
        if os.path.exists(port):
            time.sleep(1.5)
            try:
                c = Console(port)
                if first(c.cmd("axis"), "#axis"):
                    return c
                c.close()
            except Exception:
                pass
        time.sleep(0.3)
    die(f"{port}: console did not answer after reset")


def reset(port):
    rc, out = esptool(port, "--after", "hard-reset", "read-mac", timeout=30)
    if rc != 0:
        die(f"reset via esptool failed:\n{out}")


# ---------------------------------------------------------------- steps
def flash(port, build):
    bins = [("0x0", "bootloader/bootloader.bin"),
            ("0x8000", "partition_table/partition-table.bin"),
            ("0x10000", "pace_racer_atos_drive.bin")]
    args = []
    for off, rel in bins:
        p = os.path.join(build, rel)
        if not os.path.exists(p):
            die(f"{p} missing; build first: idf.py -B {os.path.basename(build)} -DATOS_MOTOR=flatbot build")
        args += [off, p]
    say(f"flashing {os.path.basename(build)} -> {port}")
    rc, out = esptool(port, "-b", "460800", "--before", "default-reset", "--after", "hard-reset",
                      "write-flash", "--flash-mode", "dio", "--flash-size", "2MB", "--flash-freq", "80m", *args,
                      timeout=300)
    if rc != 0 or "Hash of data verified" not in out:
        die(f"flash failed:\n{out[-2000:]}")
    say("flash verified")


def provision_nvs(con, axis):
    r = con.cmd(f"axis {axis}")
    if "saved" not in r:
        die(f"'axis {axis}' not accepted:\n{r}")
    r = con.cmd("fb hall")
    if not first(r, "#fb hall"):
        die(f"'fb hall' not accepted:\n{r}")
    say(f"saved axis {axis} ({AXES[axis]}), feedback hall")


def hall_cal(con):
    say(f"hall calibration: wheel turns {SPIN_RPM} rpm each way, {HCAL_SECS} s of rotation per direction")
    con.cmd("ho 9999 45 1")
    r = con.cmd(f"hcal {HCAL_SECS}")
    if "#hcal armed" not in r:
        die(f"hcal did not arm:\n{r}")
    log = ""
    for rpm in (SPIN_RPM, -SPIN_RPM):
        r = con.cmd(f"run {SPIN_AMPS} {rpm} {SPIN_RAMP_S}")
        if "#run" not in r:
            con.cmd("stop")
            die(f"'run' refused (sampler off? see 'e 1'):\n{r}")
        log += con.listen(SPIN_RAMP_S + 1)
        hs = first(con.cmd("hs"), "#hall state")
        m = re.search(r"rpm=(-?[\d.]+)", hs)
        got = abs(float(m.group(1))) if m else 0.0
        if got < 0.5 * SPIN_RPM:
            con.cmd("stop")
            die(f"wheel did not turn ({got:.0f} rpm): motor supply off, wheel blocked, or hall wiring. {hs}")
        say(f"  spinning {rpm:+d} rpm ({got:.0f} measured), collecting...")
        log += con.listen(HCAL_SECS + 3)
        log += con.cmd("stop", wait=2.5)
    r = con.cmd("hs")
    log += r
    hs = first(r, "#hall state")
    if "hcal FAILED" in log:
        die("hall calibration failed: " + first(log, "! hcal FAILED"))
    if "calleft=0/0" in hs and "cal=1" in hs and ("#hcal OK" in log or "hall table saved" in con.cmd("axis")):
        say("hall table saved: " + first(r, "#hall centers"))
    else:
        die(f"hall calibration did not finish ({hs}); run again with the wheel free")


def report(con):
    ax = first(con.cmd("axis"), "#axis")
    fb = first(con.cmd("fb"), "#fb")
    bl = con.cmd("bootlog").strip().splitlines()
    print(ax)
    print(fb)
    if bl:
        print(bl[-1].strip())
    ok = "NOT CALIBRATED" not in ax and "fb hall" in ax and "hall table saved" in ax
    return ok


# ---------------------------------------------------------------- main
def main():
    global PY
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port; default: the one PACE board found on USB")
    ap.add_argument("--axis", type=int, choices=sorted(AXES), help="wheel; default from the roster by MAC")
    ap.add_argument("--build", default=os.path.join(APP, "build-flatbot"), help="idf.py build dir to flash")
    ap.add_argument("--no-flash", action="store_true", help="keep the firmware that is on the board")
    ap.add_argument("--hcal", action="store_true", help="redo the hall table even if one is saved")
    ap.add_argument("--list", action="store_true", help="show boards on USB and exit")
    a = ap.parse_args()
    PY = find_python()

    ports = [a.port] if a.port else candidate_ports()
    if not ports:
        die("no /dev/cu.usbmodem* ports; plug the board in")
    boards = []
    for p in ports:
        mac = read_mac(p)
        tag = "BLOCKED (not a PACE board)" if mac in BLOCKLIST else \
              (f"axis {ROSTER[mac]} {AXES[ROSTER[mac]]}" if mac in ROSTER else "not in roster")
        say(f"{p}: {mac or 'no answer'}  {tag if mac else ''}")
        if mac and mac not in BLOCKLIST:
            boards.append((p, mac))
    if a.list:
        return
    if len(boards) != 1:
        die("need exactly one PACE board on USB, or --port")
    port, mac = boards[0]
    axis = a.axis if a.axis is not None else ROSTER.get(mac)
    if axis is None:
        die(f"{mac} is not in the roster; pass --axis N (or add it to ROSTER)")

    if not a.no_flash:
        flash(port, a.build)
    con = wait_for_console(port)
    before = first(con.cmd("axis"), "#axis")
    say(f"before: {before}")
    provision_nvs(con, axis)
    con.close()
    say("resetting so axis and feedback take effect")
    reset(port)
    con = wait_for_console(port)

    ax = first(con.cmd("axis"), "#axis")
    if a.hcal or "hall table saved" not in ax:
        input(f"\n   Wheel {AXES[axis]} off the ground and free, motor supply ON. Enter to spin, Ctrl-C to skip: ")
        hall_cal(con)
    else:
        say("hall table already saved (use --hcal to redo)")

    ok = report(con)
    con.close()
    say("DONE: board is ready" if ok else "NOT READY, see the axis line above")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n!! interrupted", file=sys.stderr)
        sys.exit(130)
