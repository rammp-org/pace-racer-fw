#!/usr/bin/env python3
"""Wheelchair-profile hall-drive test rig: standstill -> 60 rpm under brake load.

Drives the whole bench: board console, VM supply (BK MR3K, for DRV-latch /
wedge recovery), and the mag brake (Rigol DP2031 CH2). CH1 of the Rigol is the
torque sensor's 24 V rail and is NEVER touched here.

Phases:
  capture-centers — print the live hall table (`hs`)
  hcal   — brake to 0 V, bidirectional I/f spin, capture the table
  stall  — hiq staircase 2..10 A, wheel must NOT move (heavy-brake gate)
  torque — hiq 10..25 A x 8 s, operator reads the torque display; breakaway is
           data (true brake torque), `hs` each step watches the hall rej counter
  drive  — the target test: from stop, `hrun <A> 60 15`; pass = 55+ rpm,
           8 s hold with rpm std < 8, no trips

Watchdogs (any -> immediate `stop`): firmware `!` line, |iq| > 27 A,
temp > 75 C, dT/dt > 2 C/s, 3 s stream silence.

DRV latch / chip wedge: VM power-cycle + ESP reset + VERIFIED re-arm
(limits acked, hset re-injected), retried up to 3x. (2026-08-06: the chip can
wedge hard enough that esptool can't connect; a VM cycle un-wedges it.)

History: built 2026-08-06 during the low-speed/high-torque diagnosis. Findings
that shaped it: PI windup at coast (fixed in fw), hall-angle corruption at
~22 A stalled DC (fixed in fw: speed-aware resync gate), stall-creep 60 deg
blind spot (inherent to hall-only; encoder is the real fix), VDS thermal
derating ceiling ~24 A stall at `vds 4`.

Usage:
  python3 hall_test.py hcal
  python3 hall_test.py drive --brake 1.25 --amps 10 --skip-stall --centers <6 deg>
  python3 hall_test.py torque --brake 2.5 --centers <6 deg>
  python3 hall_test.py drive --brake 2.5 --centers <6 deg>

Logs: logs/run-<ts>-agent/{raw.log,data.csv,events.log,report.md}
"""
import argparse
import csv
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import serial

BOARD_PORT = "/dev/cu.usbmodem2101"
PSU_PORT = "/dev/cu.usbmodem615J231251"
BRAKE_VISA = "USB0::6833::42152::DP2A284M00016::0::INSTR"  # Rigol DP2031
# CH1 = torque sensor 24 V — NEVER touched. CH2 = mag brake coil.
# Bench map (rough, static hold runs ~25% above it): 1.0 V ~ 1 Nm, 1.5 ~ 3,
# 2.0 ~ 5, 2.5 ~ 8 (measured static hold at 2.5 V: 9-10 Nm).
BRAKE_MAX_V = 2.5
IDF_PYTHON = "/Users/alxchunlin/.espressif/python_env/idf6.0_py3.11_env/bin/python"
APP_DIR = Path(__file__).resolve().parent

LIMITS = ["lim 25 30", "vds 4", "vl 20"]
HALL_CENTERS = None
# 8-10 Nm needs ~15 A steady (Kt ~ 0.62 Nm/A): 15 A is the marginal point,
# 20/25 have real margin.
AMPS_STAIRCASE = [15.0, 20.0, 25.0]
TARGET_RPM = 60.0
RAMP_RPM_S = 15.0
HOLD_S = 8.0
IQ_ABORT = 27.0
TEMP_ABORT = 75.0
DTDT_ABORT = 2.0

STREAM_COLS = ["t", "mode", "id", "iq", "iqref", "vd", "vq", "aerr",
               "rpm_drive", "rpm_est", "rpm_hall", "flux", "t0", "t1", "t2", "t3"]


class Rig:
    def __init__(self, run_dir):
        self.ser = serial.Serial(BOARD_PORT, 115200, timeout=0.3)
        self.run_dir = run_dir
        self.raw = open(run_dir / "raw.log", "a")
        self.events = open(run_dir / "events.log", "a")
        self.csv_f = open(run_dir / "data.csv", "a", newline="")
        self.csv = csv.writer(self.csv_f)
        if self.csv_f.tell() == 0:
            self.csv.writerow(["host_time"] + STREAM_COLS)
        self.temps_prev = None
        self.last_rx = time.time()
        self.fault = None  # (kind, detail) set by watchdogs

    def event(self, msg):
        line = f"{datetime.now().isoformat(timespec='milliseconds')} {msg}"
        print(line, flush=True)
        self.events.write(line + "\n")
        self.events.flush()

    def send(self, cmd):
        self.event(f"> {cmd}")
        self.ser.write((cmd + "\n").encode())

    def stop(self):
        self.ser.write(b"stop\n")
        self.event("> stop (harness)")

    def poll(self):
        ln = self.ser.readline().decode(errors="replace").strip()
        now = time.time()
        if not ln:
            if now - self.last_rx > 3.0:
                self.fault = ("silence", f"no serial output for {now-self.last_rx:.1f}s")
            return None
        self.last_rx = now
        self.raw.write(ln + "\n")
        if ln.startswith("!"):
            self.event(ln)
            self.fault = ("firmware", ln)
            return ln
        if ln.startswith("#"):
            self.event(ln)
            return ln
        parts = ln.split(", ")
        if len(parts) != len(STREAM_COLS):
            return None
        try:
            row = {"mode": parts[1]}
            for i, k in enumerate(STREAM_COLS):
                if k != "mode":
                    row[k] = float(parts[i])
        except ValueError:
            return None
        self.csv.writerow([datetime.now().isoformat(timespec='milliseconds')] + parts)
        if abs(row["iq"]) > IQ_ABORT:
            self.fault = ("overcurrent", f"iq={row['iq']:.1f}A on stream")
        temps = [row["t0"], row["t1"], row["t2"], row["t3"]]
        if max(temps) > TEMP_ABORT:
            self.fault = ("temp", f"max temp {max(temps):.1f}C")
        if self.temps_prev and now - self.temps_prev[0] >= 1.0:
            dt = now - self.temps_prev[0]
            rates = [(a - b) / dt for a, b in zip(temps, self.temps_prev[1])]
            if max(rates) > DTDT_ABORT:
                self.fault = ("dtdt", f"dT/dt {max(rates):.2f}C/s")
            self.temps_prev = (now, temps)
        elif not self.temps_prev:
            self.temps_prev = (now, temps)
        return row

    def run_for(self, seconds, on_row=None):
        t_end = time.time() + seconds
        while time.time() < t_end:
            r = self.poll()
            if self.fault:
                self.stop()
                return False
            if on_row and isinstance(r, dict):
                on_row(r)
        return True

    def command_and_drain(self, cmd, seconds=1.0):
        self.send(cmd)
        self.run_for(seconds)


class Brake:
    """Rigol DP2031 CH2 only. Channel-explicit SCPI so CH1 can never be hit."""
    def __init__(self):
        import pyvisa
        self.dp = pyvisa.ResourceManager('@py').open_resource(BRAKE_VISA)
        self.dp.timeout = 3000

    def set_v(self, volts, rig=None):
        volts = max(0.0, min(float(volts), BRAKE_MAX_V))
        self.dp.write(f':SOUR2:VOLT {volts:.3f}')
        time.sleep(0.8)
        mv = float(self.dp.query(':MEAS:VOLT? CH2'))
        mi = float(self.dp.query(':MEAS:CURR? CH2'))
        if rig:
            rig.event(f"#brake CH2 -> {volts:.2f} V (meas {mv:.3f} V, {mi:.3f} A)")
        if abs(mv - volts) > 0.05:
            raise RuntimeError(f"brake voltage mismatch: set {volts} meas {mv}")
        return mv

    def close(self):
        self.dp.close()


def psu_q(s, cmd):
    s.reset_input_buffer()
    s.write((cmd + "\n").encode())
    return s.readline().decode(errors="replace").strip()


def vm_cycle_and_rearm(rig):
    """DRV latched (or chip wedged): cycle VM, reset ESP, VERIFY the board is
    talking, re-apply limits + hall table. Retry: the wedge can survive one
    cycle."""
    for attempt in range(3):
        rig.event(f"== DRV/wedge recovery: VM power-cycle (attempt {attempt+1})")
        try:
            rig.ser.close()
        except Exception:
            pass
        ps = serial.Serial(PSU_PORT, 115200, timeout=1.0)
        ps.write(b"OUTP OFF\n")
        t0 = time.time()
        while time.time() - t0 < 30:
            try:
                if float(psu_q(ps, "MEAS:VOLT?")) < 5.0:
                    break
            except ValueError:
                pass
            time.sleep(1.0)
        time.sleep(3.0)
        ps.write(b"OUTP ON\n")
        time.sleep(1.5)
        v = psu_q(ps, "MEAS:VOLT?")
        ps.close()
        rig.event(f"VM restored: {v} V; resetting ESP")
        r = subprocess.run([IDF_PYTHON, "-m", "esptool", "--chip", "esp32s3",
                            "--port", BOARD_PORT, "--before", "default-reset",
                            "--after", "hard-reset", "chip-id"],
                           capture_output=True, timeout=90)
        if r.returncode != 0:
            rig.event(f"! esptool could not connect (attempt {attempt+1}) — chip "
                      f"still wedged, cycling again")
            continue
        deadline = time.time() + 15
        rig.ser = None
        while time.time() < deadline:
            try:
                rig.ser = serial.Serial(BOARD_PORT, 115200, timeout=0.3)
                break
            except serial.SerialException:
                time.sleep(0.5)
        if rig.ser is None:
            rig.event("! console did not re-enumerate")
            continue
        rig.last_rx = time.time()
        rig.fault = None
        rig.run_for(5.0)  # boot banner
        acked = True
        for cmd in LIMITS:
            ack = cmd.split()[0]
            got_ack = False
            for _ in range(3):
                rig.send(cmd)
                got, t_end = [], time.time() + 1.0
                while time.time() < t_end:
                    rr = rig.poll()
                    if isinstance(rr, str):
                        got.append(rr)
                if any(f"#{ack}" in g for g in got):
                    got_ack = True
                    break
            if not got_ack:
                acked = False
                break
        if not acked:
            rig.event("! board not acking after VM cycle — retrying full cycle")
            continue
        if HALL_CENTERS:
            rig.command_and_drain("hset " + " ".join(f"{c:.1f}" for c in HALL_CENTERS))
        rig.event("== re-armed and VERIFIED after VM cycle (test halted for review)")
        return True
    rig.event("! recovery failed after 3 VM cycles — needs hands at the bench")
    return False


def phase_hcal(rig, brake):
    global HALL_CENTERS
    rig.event("== PHASE hcal: brake -> 0 V, bidirectional cal spin")
    brake.set_v(0.0, rig)
    rig.command_and_drain("ho 9999 45 1")   # engage rpm unreachable: stay in I/f
    rig.command_and_drain("hcal 8")         # 8 s of rotation per direction
    centers = None
    # 4 A: brake remanence after a session at 2.5 V adds drag that made the
    # 3 A spin jerky enough to fail the per-sector uniformity check (2026-08-17)
    for direction in ("run 4 30 5", "run 4 -30 5"):
        rig.send(direction)
        t0 = time.time()
        while time.time() - t0 < 40:
            r = rig.poll()
            if rig.fault:
                rig.stop()
                return False, f"watchdog {rig.fault} during hcal spin"
            if isinstance(r, str) and "#hcal OK" in r:
                centers = [float(tok.split("=")[1]) for tok in r.split()
                           if "=" in tok and tok[0].isdigit()]
                break
        rig.command_and_drain("stop", 2.0)
        if centers:
            break
    if not centers or len(centers) != 6:
        return False, "hcal did not complete (check calleft via hs)"
    HALL_CENTERS = centers
    rig.event(f"#= captured centers: {centers}")
    return True, f"cal table captured: {centers}"


def phase_stall(rig):
    rig.event("== PHASE stall: hiq staircase 2..10 A at standstill")
    for amps in [2, 4, 6, 8, 10]:
        rows = []
        rig.send(f"hiq {amps}")
        if not rig.run_for(5.0, on_row=rows.append):
            return False, f"watchdog {rig.fault} at hiq {amps}"
        tail = rows[-20:]
        iq_avg = sum(r["iq"] for r in tail) / max(len(tail), 1)
        rpm_max = max(abs(r["rpm_hall"]) for r in tail) if tail else 0
        rig.event(f"#= hiq {amps}: iq_avg={iq_avg:.2f} rpm_max={rpm_max:.0f}")
        if abs(iq_avg - amps) > 0.1 * amps + 0.3:
            rig.stop()
            return False, f"iq tracking failed at {amps} A (avg {iq_avg:.2f})"
        if rpm_max > 10:
            rig.stop()
            return False, f"rotor moved at hiq {amps} — brake too low for stall phase"
    rig.command_and_drain("stop", 2.0)
    return True, "stall staircase clean"


def phase_torque(rig):
    """Slow hiq staircase, operator reading the torque display. Breakaway ends
    the phase as DATA. `hs` each step watches the hall reject counter."""
    rig.event("== PHASE torque: hiq 10..25 A x 8 s — call out the torque display!")
    for amps in [10, 12, 14, 16, 18, 20, 22, 24, 25]:
        rows = []
        rig.send(f"hiq {amps}")
        if not rig.run_for(8.0, on_row=rows.append):
            return False, f"watchdog {rig.fault} at hiq {amps}"
        rig.send("hs")
        rig.run_for(1.0)
        moved = [r for r in rows if abs(r["rpm_hall"]) > 5]
        tail = rows[-20:]
        iq_avg = sum(r["iq"] for r in tail) / max(len(tail), 1)
        rig.event(f"#= hiq {amps}: iq_avg={iq_avg:.2f} moved={bool(moved)}")
        if moved:
            rig.command_and_drain("stop", 3.0)
            return True, f"BREAKAWAY at {amps} A (iq_avg {iq_avg:.1f})"
        if abs(iq_avg - amps) > 0.1 * amps + 0.3:
            rig.command_and_drain("stop", 3.0)
            return False, f"iq tracking broke at {amps} A (avg {iq_avg:.2f})"
    rig.command_and_drain("stop", 3.0)
    return True, "held 25 A at stall, no breakaway, tracking clean"


def phase_drive(rig, amps):
    rig.event(f"== PHASE drive: hrun {amps} {TARGET_RPM} {RAMP_RPM_S} from standstill")
    rig.run_for(2.5)  # let any prior STOPPING unload finish before re-engaging
    rig.fault = None
    rows = []
    rig.send(f"hrun {amps} {TARGET_RPM:.0f} {RAMP_RPM_S:.0f}")
    t_limit = TARGET_RPM / RAMP_RPM_S + 10.0
    t0 = time.time()
    reached = None
    while time.time() - t0 < t_limit:
        if not rig.run_for(0.2, on_row=rows.append):
            return False, f"watchdog {rig.fault} during ramp", rows
        if rows and rows[-1]["rpm_hall"] >= 0.92 * TARGET_RPM:
            reached = time.time() - t0
            break
    if reached is None:
        peak = max((r["rpm_hall"] for r in rows), default=0)
        rig.stop()
        return False, f"never reached {TARGET_RPM} rpm (peaked ~{peak:.0f})", rows
    rig.event(f"#= reached {0.92*TARGET_RPM:.0f}+ rpm in {reached:.1f}s; holding {HOLD_S}s")
    hold = []
    if not rig.run_for(HOLD_S, on_row=hold.append):
        return False, f"watchdog {rig.fault} during hold", rows + hold
    rpms = [r["rpm_hall"] for r in hold]
    iqs = [r["iq"] for r in hold]
    mean = sum(rpms) / len(rpms)
    std = (sum((x - mean) ** 2 for x in rpms) / len(rpms)) ** 0.5
    iq_mean = sum(iqs) / len(iqs)
    rig.event(f"#= hold: rpm {mean:.1f} +/- {std:.1f}, iq {iq_mean:.1f} A")
    rig.command_and_drain("stop", 3.0)
    if std > 8.0:
        return False, f"held but rough: rpm std {std:.1f}", rows + hold
    return True, f"smooth: {mean:.1f} rpm +/- {std:.1f}, iq {iq_mean:.1f} A", rows + hold


def main():
    global HALL_CENTERS
    ap = argparse.ArgumentParser()
    ap.add_argument("phase", choices=["capture-centers", "hcal", "stall", "torque", "drive"])
    ap.add_argument("--amps", type=float, help="single drive amps instead of staircase")
    ap.add_argument("--centers", type=float, nargs=6, help="hall table to hset at start")
    ap.add_argument("--brake", type=float, default=2.5,
                    help="brake CH2 volts for stall/torque/drive (default 2.5)")
    ap.add_argument("--skip-stall", action="store_true",
                    help="skip the hiq stall gate (light brake would break away)")
    args = ap.parse_args()

    run_dir = APP_DIR / "logs" / f"run-{datetime.now():%Y%m%d-%H%M%S}-agent"
    run_dir.mkdir(parents=True, exist_ok=True)
    rig = Rig(run_dir)
    rig.event(f"session start (agent harness), port={BOARD_PORT}, logs={run_dir}")

    if args.phase == "capture-centers":
        rig.send("hs")
        rig.run_for(1.5)
        return 0

    # First write after port-open can get mangled — verify each limit acked.
    for cmd in LIMITS:
        ack = cmd.split()[0]
        for attempt in range(3):
            rig.send(cmd)
            got, t_end = [], time.time() + 1.0
            while time.time() < t_end:
                r = rig.poll()
                if isinstance(r, str):
                    got.append(r)
            if any(f"#{ack}" in a for a in got):
                break
        else:
            rig.event(f"! limit command never acked: {cmd}")
            return 1
    if args.centers:
        HALL_CENTERS = list(args.centers)
        rig.command_and_drain("hset " + " ".join(f"{c:.1f}" for c in HALL_CENTERS))

    brake = Brake()
    results = []

    if args.phase == "hcal":
        ok, msg = phase_hcal(rig, brake)
        results.append(("hcal", ok, msg))
    else:
        brake.set_v(args.brake, rig)
        if args.phase == "torque":
            ok, msg = phase_torque(rig)
            results.append(("torque", ok, msg))
        elif args.skip_stall:
            ok, msg = True, "skipped (light brake)"
            results.append(("stall", ok, msg))
        else:
            ok, msg = phase_stall(rig)
            results.append(("stall", ok, msg))
    if rig.fault and rig.fault[0] == "firmware" and "DRV8353" in rig.fault[1]:
        vm_cycle_and_rearm(rig)
    if ok and args.phase == "drive":
        for amps in ([args.amps] if args.amps else AMPS_STAIRCASE):
            ok, msg, _ = phase_drive(rig, amps)
            results.append((f"drive {amps:.0f}A", ok, msg))
            rig.fault = None  # performance fails don't stop the staircase
            if "watchdog" in msg and "DRV8353" in msg:
                vm_cycle_and_rearm(rig)
                break
            if "watchdog" in msg:
                break

    rig.event("== RESULTS")
    for name, ok, msg in results:
        rig.event(f"  {'PASS' if ok else 'FAIL'} {name}: {msg}")
    with open(run_dir / "report.md", "w") as f:
        f.write(f"# Agent hall-drive test — {datetime.now():%Y-%m-%d %H:%M}\n\n")
        f.write(f"- Limits: {', '.join(LIMITS)}; brake {args.brake} V; target "
                f"{TARGET_RPM} rpm @ {RAMP_RPM_S} rpm/s\n\n")
        for name, ok, msg in results:
            f.write(f"- **{'PASS' if ok else 'FAIL'}** `{name}` — {msg}\n")
    rig.stop()
    return 0 if all(ok for _, ok, _ in results) else 1


if __name__ == "__main__":
    sys.exit(main())
