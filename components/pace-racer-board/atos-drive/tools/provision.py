#!/usr/bin/env python3
"""Make a PACE RACER board a robot wheel: flash, axis, feedback, hall table.

Over USB (a board on the bench, or its first flash before it goes into a wheel):

    tools/provision.py --motor hub20            # one board on USB: flash + axis + fb hall, hcal if needed
    tools/provision.py --motor hub20 --axis 3   # override the roster
    tools/provision.py --no-flash               # NVS only, keep whatever firmware is on it
    tools/provision.py --hcal                   # redo the hall table even if one is saved
    tools/provision.py --list                   # just say which boards are plugged in

Over ethernet (a board inside a wheel; needs firmware with the service stream):

    tools/provision.py --host auto --motor hub20        # the one board on the LAN
    tools/provision.py --host drive_left --axis 2       # by current axis name, or IP
    tools/provision.py --host list                      # who answers on the LAN

Steps, each reported as it goes:
  1. find the board. USB: every PACE / Espressif serial port is asked for its MAC with
     esptool; boards on the blocklist below are never touched; with several
     plugged in, pass --port. Ethernet: a "PACE?" broadcast; the board
     reports its MAC so the same roster applies.
  2. flash the build in --build (default build-<motor>; run idf.py there
     first). USB = esptool with the full table; ethernet = OTA of the app,
     wait for it to come back, mark it valid.
  3. over the console: axis <n>, fb hall, then a reset so both take effect.
  4. if the hall table is not saved (or --hcal): the calibration spin. Wheel
     off the ground, motor supply on. The wheel turns both ways for ~15 s each.
  5. read back 'axis' and 'fb' and print the result line.

USB needs pyserial and esptool. The ESP-IDF python env has both; the script
finds it under ~/.espressif when the system python lacks them. Ethernet needs
only the standard library plus the espp_ota package in managed_components.
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
sys.path.insert(0, HERE)
import eth_link  # noqa: E402  (the ethernet twin of the serial console + esptool below)

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
    venvs = os.path.expanduser("~/.espressif/python_env")
    cands = glob.glob(os.path.join(venvs, "*", "bin", "python")) + \
        glob.glob(os.path.join(venvs, "*", "Scripts", "python.exe"))  # Windows venv layout
    for exe in [sys.executable] + sorted(cands):
        r = subprocess.run([exe, "-c", "import esptool, serial, usb.core"], capture_output=True)
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


# USB ids a PACE board shows up with: the app's own (pid.codes VID, the PIDs
# atos-drive and the dyno demo use) and Espressif's (ROM download mode and
# USB-Serial-JTAG builds, i.e. a blank or older board).
PACE_USB_IDS = {(0x1209, 0x0D41), (0x1209, 0x0D32)}
ESPRESSIF_VID = 0x303A


def candidate_ports():
    """Serial ports that look like a PACE board, on any OS (/dev/cu.usbmodem*,
    /dev/ttyACM*, COM*)."""
    from serial.tools import list_ports
    return sorted(p.device for p in list_ports.comports()
                  if p.vid == ESPRESSIF_VID or (p.vid, p.pid) in PACE_USB_IDS)


def port_present(port):
    from serial.tools import list_ports
    return any(p.device == port for p in list_ports.comports())


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
        if port_present(port):
            time.sleep(1.5)
            try:
                c = Console(port)
                if first(c.cmd("axis"), "#axis"):
                    return c
                c.close()
            except Exception:
                pass
        time.sleep(0.3)
    die(f"{port}: console did not answer after reset. Either the app is not booting (wrong "
        "partition layout: reflash from a build whose app offset matches its table) or this build "
        "does not put the console on USB-Serial-JTAG (CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG).")


def reset(port):
    """The WebUSB builds own the S3's one USB PHY, so esptool's DTR/RTS reset
    does nothing to them: ask the System module over the vendor stream first
    (needs pyusb), and fall back to esptool for a board without that link."""
    try:
        import usb_reset
        if usb_reset.reset_via_vendor():
            return
    except ImportError:
        pass
    rc, out = esptool(port, "--after", "hard-reset", "read-mac", timeout=30)
    if rc != 0:
        die(f"reset via esptool failed (and no WebUSB vendor interface answered):\n{out}")


# ---------------------------------------------------------------- steps
def flash(port, build):
    """Flash exactly what the build says: offsets and flash settings come from
    flasher_args.json, never from here. A build whose app is not at the
    offset the bootloader expects would brick the board (2026-09-28: a
    partition table with the app at 0x20000 was written at 0x10000)."""
    import json
    fa_path = os.path.join(build, "flasher_args.json")
    if not os.path.exists(fa_path):
        die(f"{fa_path} missing; build first: idf.py -B {os.path.basename(build)} -DATOS_MOTOR=flatbot build")
    fa = json.load(open(fa_path))
    fs = fa["flash_settings"]
    files = sorted(fa["flash_files"].items(), key=lambda kv: int(kv[0], 16))
    args = []
    for off, rel in files:
        pth = os.path.join(build, rel)
        if not os.path.exists(pth):
            die(f"{pth} missing; rebuild")
        args += [off, pth]
    app_off = fa.get("app", {}).get("offset")
    # The app must land inside an app partition of the table being flashed.
    pt = os.path.join(build, fa["partition-table"]["file"])
    idf = os.environ.get("IDF_PATH") or os.path.expanduser(os.path.join("~", "esp", "esp-idf"))
    gen = os.path.join(idf, "components", "partition_table", "gen_esp32part.py")
    if app_off and os.path.exists(gen):
        r = subprocess.run([PY, gen, pt], capture_output=True, text=True)
        rows = [ln.split(",") for ln in r.stdout.splitlines() if "," in ln and not ln.startswith("#")]
        apps = [row for row in rows if len(row) > 3 and row[1].strip() == "app"]
        if not any(int(row[3], 16) == int(app_off, 16) for row in apps):
            die(f"build puts the app at {app_off} but the partition table has app partitions at "
                f"{[row[3].strip() for row in apps]}; refusing to flash a board that would not boot")
    say(f"flashing {build} -> {port} ({fs['flash_size']}, app at {app_off})")
    rc, out = esptool(port, "-b", "460800", "--before", "default-reset", "--after", "hard-reset",
                      "write-flash", "--flash-mode", fs["flash_mode"], "--flash-size", fs["flash_size"],
                      "--flash-freq", fs["flash_freq"], *args, timeout=300)
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


# ---------------------------------------------------------------- ethernet
def main_eth(a):
    """Same steps as main(), board reached over tcp :5555 (see eth_link.py)."""
    if a.host == "list" or a.list:
        boards = eth_link.discover()
        if not boards:
            die("no boards answered on the LAN (is the DHCP server / MIB up? right interface?)")
        for b in boards:
            mac = b.get("mac", "?")
            tag = "BLOCKED" if mac in BLOCKLIST else \
                  (f"roster: axis {ROSTER[mac]} {AXES[ROSTER[mac]]}" if mac in ROSTER else "not in roster")
            say(f"{b['ip']}: {mac}  now axis {b.get('axis')} {b.get('axis_name')}  fw {b.get('version')} "
                f"{b.get('motor')}  {tag}")
        return
    try:
        host, inf = eth_link.resolve(a.host)
        if inf is None:
            inf = eth_link.info(host)
    except eth_link.LinkError as e:
        die(str(e))
    mac = inf.get("mac")
    if mac in BLOCKLIST:
        die(f"{host} ({mac}) is on the blocklist")
    axis = a.axis if a.axis is not None else ROSTER.get(mac)
    if axis is None:
        die(f"{host} ({mac}) is not in the roster; pass --axis N (or add it to ROSTER)")
    say(f"{host}: {mac}  running {inf.get('version')} ({inf.get('motor')}) on {inf.get('partition')}, "
        f"axis {inf.get('axis')} {inf.get('axis_name')}")

    if not a.no_flash:
        binary = os.path.join(a.build, "pace_racer_atos_drive.bin")
        if not os.path.exists(binary):
            die(f"{binary} missing; build first: idf.py -B {os.path.basename(a.build)} "
                f"-DATOS_MOTOR={a.motor} build")
        try:
            eth_link.flash(host, binary, say=say)
        except (eth_link.LinkError, eth_link.OtaError, OSError) as e:
            die(f"OTA failed: {e}. The board keeps its old image; power-cycle if it does not answer.")

    con = eth_link.EthConsole(host)
    before = first(con.cmd("axis"), "#axis")
    say(f"before: {before}")
    provision_nvs(con, axis)
    con.close()
    say("resetting so axis and feedback take effect")
    eth_link.reset(host)
    time.sleep(3.0)
    try:
        eth_link.wait_for_board(host, say=say)
    except eth_link.LinkError as e:
        die(str(e))
    con = eth_link.EthConsole(host)

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


# ---------------------------------------------------------------- main
def main():
    global PY
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port; default: the one PACE board found on USB")
    ap.add_argument("--host", help="ethernet instead of USB: IP, axis name, axis number, 'auto' or 'list'")
    ap.add_argument("--axis", type=int, choices=sorted(AXES), help="wheel; default from the roster by MAC")
    ap.add_argument("--motor", default="flatbot", choices=["flatbot", "hub20", "ninebot"],
                    help="which profile's build dir to flash (build-<motor>); default flatbot")
    ap.add_argument("--build", help="idf.py build dir to flash (overrides --motor)")
    ap.add_argument("--no-flash", action="store_true", help="keep the firmware that is on the board")
    ap.add_argument("--hcal", action="store_true", help="redo the hall table even if one is saved")
    ap.add_argument("--list", action="store_true", help="show boards on USB and exit")
    a = ap.parse_args()
    if not a.build:
        a.build = os.path.join(APP, f"build-{a.motor}")
    if a.host:
        return main_eth(a)
    # The USB path needs pyserial, esptool and pyusb in THIS interpreter (the
    # console, the vendor-stream reset). Re-exec under one that has them.
    try:
        import esptool, serial, usb.core  # noqa: F401
    except ImportError:
        py = find_python()
        if os.path.realpath(py) != os.path.realpath(sys.executable):
            os.execv(py, [py, os.path.abspath(__file__), *sys.argv[1:]])
    PY = find_python()

    ports = [a.port] if a.port else candidate_ports()
    if not ports:
        die("no PACE board on USB (no serial port with VID 1209:0d41/0d32 or 303a); plug the board in"
            + ("; on Linux check the udev rule in tools/README.md" if sys.platform.startswith("linux") else ""))
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
