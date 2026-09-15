#!/usr/bin/env python3
"""MCP server for the dyno test stand — one tool per bench operation.

The point of this server is that a session should NOT have to know which
supply is which, what voltage the brake wants, or which recovery clears which
kind of wedge. The numbers live here; the caller just asks for the operation.

The bench (see README.md for the full map and gotcha list):
  BK MR3K160120   48 V VM rail, SCPI serial. Vset NEVER written — no tool
                  reaches it. Iset/output only.
  Rigol DP2031    CH1 = torque sensor 24 V rail, READ ONLY. CH2 = mag brake.
  Rigol MHO984    scope, LAN only (raw socket). CH1/CH2 phase-B high-side
                  gate/source (MATH1 = VGS), CH3 speed out, CH4 torque @ 20 N·m/V.
  Board           ESP32-S3 + DRV8353 on the dock hub, USB console 115200.

Scope of this server: bring-up, teardown, instrument setpoints, measurement,
and recovery. It deliberately cannot spin the motor — motion belongs in a
script with a watchdog loop (crib fig6_hold.py).

Protocol is hand-rolled JSON-RPC 2.0 over stdio: the `mcp` package needs
py3.10+ and the bench host runs the system 3.9 that already owns pyserial and
pyvisa. Stdlib only, so there is nothing to install on another machine.
"""
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# stdout is the protocol channel and nothing else. bench_lib.save() and any
# library chatter print()s — rebind stdout so a stray print can't corrupt a
# JSON-RPC frame.
_OUT = sys.stdout
sys.stdout = sys.stderr

import bench_lib as bl  # noqa: E402

SERVER = {"name": "bench", "version": "1.0.0"}

# Live instrument handles. Kept open across calls on purpose: the README's #1
# gotcha is the console wedge, and repeated open/close churn on the board's
# USB-JTAG port is what provokes it.
_h = {"board": None, "brake": None, "psu": None, "scope": None}

ARM_PROFILES = {
    # name: (console commands, what it is for)
    "normal": (["lim 25 30", "vds 5", "vl 20", "odg 8"],
               "25 A / 30 A guard — everyday characterization"),
    "power":  (["lim 30 35", "vds 5", "vl 20", "odg 8"],
               "30 A / 35 A guard — power runs only; 30/35 is the compile-time "
               "hard ceiling, user-authorized 2026-08-19"),
}

# Commands that turn the wheel or run a self-cal that turns the wheel. This
# server is setup+measure only; these are refused with a pointer to the
# scripted path.
MOTION_CMDS = {"run", "hrun", "hiq", "erun", "eiq", "ecal", "hcal", "epos", "pg"}

NUM = r"(-?(?:[\d.]+|nan))"
import re  # noqa: E402
SROW = re.compile(r"^(\d+\.\d+), ([A-Z]), " + ", ".join([NUM] * 5) +
                  r", (-?\d+), (-?\d+), (-?\d+), (-?\d+), " + ", ".join([NUM] * 5))


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def _fmt(x, nd=2):
    x = _f(x)
    return "?" if x != x else ("%.*f" % (nd, x))


# ---------------------------------------------------------------- handles


def board(fresh=False):
    if fresh or _h["board"] is None:
        drop_board()
        _h["board"] = bl.Board()
    return _h["board"]


def drop_board():
    b = _h["board"]
    _h["board"] = None
    if b is not None:
        try:
            b.close()
        except Exception:
            try:
                b.s.close()
            except Exception:
                pass


def brake():
    if _h["brake"] is None:
        _h["brake"] = bl.Brake()
    return _h["brake"]


def psu():
    if _h["psu"] is None:
        _h["psu"] = bl.Psu()
    return _h["psu"]


def drop_psu():
    p = _h["psu"]
    _h["psu"] = None
    if p is not None:
        try:
            p.close()
        except Exception:
            pass


def scope():
    if _h["scope"] is None:
        _h["scope"] = bl.Scope()
    return _h["scope"]


def board_port():
    try:
        return bl._find_board_port()
    except RuntimeError:
        return None


# ---------------------------------------------------------------- tools


def t_bench_status(a):
    out = ["BENCH STATUS"]

    try:
        p = psu()
        v, i = p.meas()
        out.append("  BK MR3K (VM 48 V): output %s, measured %s V / %s A, "
                   "Iset %s A" % ("ON" if p.output_on() else "OFF",
                                  _fmt(v, 2), _fmt(i, 2), _fmt(p.iset_read(), 1)))
        if _f(v) == _f(v) and _f(v) < bl.Psu.VM_MIN:
            out.append("    -> VM is down; the board will not enumerate until it is up.")
    except Exception as e:
        drop_psu()
        out.append("  BK MR3K: UNREACHABLE (%s)" % e)

    try:
        br = brake()
        tv, ti = br.torque_rail_meas()
        bv, bi = br.meas()
        out.append("  Rigol CH1 (torque sensor rail, read-only): %s V / %s A"
                   % (_fmt(tv, 2), _fmt(ti, 3)))
        if _f(ti) < 0.05:
            out.append("    -> draw is far below the healthy ~0.13 A: sensor "
                       "unpowered, torque readings are meaningless. Set CH1 to "
                       "24.000 V / 0.5 A on the front panel.")
        out.append("  Rigol CH2 (mag brake): %s V / %s A" % (_fmt(bv, 2), _fmt(bi, 3)))
        if _f(bv) > 0.5 and _f(bi) > 0 and abs(_f(bv) / _f(bi) - 7.6) > 4:
            out.append("    -> V/I is off the coil's ~7.6 ohm: possible CC clamp.")
    except Exception as e:
        _h["brake"] = None
        out.append("  Rigol DP2031: UNREACHABLE (%s)" % e)

    port = board_port()
    if port is None:
        out.append("  Board: no USB port present (VM down, or the console wedged "
                   "-> bench_usb_replug).")
    else:
        out.append("  Board: port %s" % port)
        try:
            hs = board().cmd("hs", 0.6)
            if "#hall" in hs:
                out.append("    console ALIVE — %s" % hs.strip().splitlines()[-1][:120])
            else:
                out.append("    console MUTE (no #hall ack). If esptool also cannot "
                           "reach it: bench_usb_replug. If the stream shows a DRV "
                           "fault line: bench_vm_recover.")
        except Exception as e:
            drop_board()
            out.append("    console error: %s" % e)

    sc = _h["scope"]
    out.append("  Scope: %s" % ("connected" if sc else "not opened this session "
                                "(bench_torque_read opens it)"))
    return "\n".join(out)


def t_bench_up(a):
    profile = a.get("arm_profile", "normal")
    if profile not in ARM_PROFILES:
        raise ValueError("arm_profile must be one of %s" % list(ARM_PROFILES))
    out = ["BRING-UP (%s profile)" % profile]

    # 1. CH1 torque rail — verified, never written.
    try:
        tv, ti = brake().torque_rail_meas()
        out.append("1. CH1 torque rail: %s V / %s A" % (_fmt(tv, 2), _fmt(ti, 3)))
        if _f(ti) < 0.05 or abs(_f(tv) - 24.0) > 1.0:
            out.append("   WARNING: expected 24.000 V / ~0.13 A. CH1 is set by hand "
                       "on the front panel (never by software). Torque numbers are "
                       "invalid until this is right; everything else still works.")
    except Exception as e:
        out.append("1. CH1: could not read (%s)" % e)

    # 2. CH2 brake parked at zero with a sane limit.
    try:
        br = brake()
        br.dp.write(":SOUR2:CURR 1.000")
        br.off()
        out.append("2. CH2 brake: 0 V, output OFF, current limit 1.000 A")
    except Exception as e:
        out.append("2. CH2 brake: FAILED (%s)" % e)

    # 3. VM up.
    try:
        p = psu()
        if not p.output_on():
            p.output(True)
        v = p.wait_vm()
        out.append("3. BK VM: output ON, rail %s V (Vset untouched)" % _fmt(v, 2))
        if _f(v) < bl.Psu.VM_MIN:
            return "\n".join(out + ["   ABORT: rail never reached %.0f V." % bl.Psu.VM_MIN])
    except Exception as e:
        return "\n".join(out + ["3. BK VM: FAILED (%s) — cannot continue." % e])

    # 4. Board enumerates only once VM is up.
    port = None
    t0 = time.time()
    while time.time() - t0 < 20:
        port = board_port()
        if port:
            break
        time.sleep(1.0)
    if not port:
        return "\n".join(out + ["4. Board: no USB port after 20 s. Try "
                                "bench_usb_replug, then bench_up again."])
    out.append("4. Board: enumerated at %s" % port)

    # 5. Console open (Board() quiets the stream itself) + arm.
    try:
        b = board(fresh=True)
        cmds, why = ARM_PROFILES[profile]
        acks = []
        for c in cmds:
            r = b.ack(c, "#")
            acks.append("%s -> %s" % (c, r.strip().splitlines()[-1][:70]))
        out.append("5. Console open, stream quiet (sq 1)")
        out.append("6. Armed (%s):" % why)
        out.extend("     " + x for x in acks)
        out.append("   NOTE: lim is clamped by the compile-time ceiling — the acks "
                   "above are the truth, not the requested values.")
    except Exception as e:
        drop_board()
        return "\n".join(out + ["5. Console/arm FAILED (%s). If the console is mute, "
                                "bench_usb_replug." % e])

    out.append("READY. Brake is off; raise it with bench_brake_set. This server "
               "cannot spin the motor — write a script with a watchdog for that.")
    return "\n".join(out)


def t_bench_down(a):
    power_off = bool(a.get("power_off", False))
    out = ["TEARDOWN"]

    if _h["board"] is not None or board_port():
        try:
            b = board()
            b.cmd("stop", 0.5)
            time.sleep(1.0)
            out.append("  motor: stop sent")
        except Exception as e:
            out.append("  motor: stop failed (%s)" % e)
    try:
        brake().set(0.0, settle=0)
        brake().off()
        out.append("  brake: 0 V, CH2 off")
    except Exception as e:
        out.append("  brake: %s" % e)
    try:
        out.append("  BK Iset: restored to %.0f A" % psu().iset(bl.Psu.IDLE_ISET))
        if power_off:
            psu().output(False)
            out.append("  BK output: OFF (board will drop off USB)")
        else:
            out.append("  BK output: left ON (Vset untouched)")
    except Exception as e:
        out.append("  BK: %s" % e)
    drop_board()
    out.append("  board console: stream quieted and closed")
    return "\n".join(out)


def t_bench_arm(a):
    profile = a.get("profile", "normal")
    if profile not in ARM_PROFILES:
        raise ValueError("profile must be one of %s" % list(ARM_PROFILES))
    cmds, why = ARM_PROFILES[profile]
    b = board()
    lines = ["ARM %s — %s" % (profile, why)]
    for c in cmds:
        r = b.ack(c, "#")
        lines.append("  %s -> %s" % (c, r.strip().splitlines()[-1][:80]))
    lines.append("Read the acks: the firmware clamps lim to its hard ceiling.")
    return "\n".join(lines)


def t_bench_brake_set(a):
    volts = float(a["volts"])
    ilim = float(a.get("ilim", 3.0))
    ilim = max(0.1, min(3.0, ilim))
    br = brake()
    got = br.set(volts, settle=float(a.get("settle", 0.6)), ilim=ilim)
    v, i = br.meas()
    line = ["brake commanded %.3f V (ilim %.3f A re-asserted), measured %s V / %s A"
            % (got, ilim, _fmt(v, 3), _fmt(i, 3))]
    if got != volts:
        line.append("  (clamped from %.3f — harness allows 0-24 V)" % volts)
    if _f(i) > 0 and _f(v) > 0.5 and _f(v) / _f(i) < 4.0:
        line.append("  WARNING: V/I well under the coil's ~7.6 ohm — channel is "
                    "likely CC-clamping and the brake is starving.")
    line.append("  Remanence: torque at a given voltage rises after high-excitation "
                "events. Approach any edge from below.")
    return "\n".join(line)


def t_bench_brake_off(a):
    brake().off()
    return "brake: 0 V, CH2 output OFF"


def t_bench_psu_iset(a):
    amps = float(a["amps"])
    p = psu()
    got = p.iset(amps)
    v, i = p.meas()
    msg = ["BK Iset -> %.2f A (Vset untouched; rail %s V, drawing %s A)"
           % (got, _fmt(v, 2), _fmt(i, 2))]
    if got != amps:
        msg.append("  (clamped from %.2f — allowed 0.5-%.0f A)" % (amps, bl.Psu.MAX_ISET))
    msg.append("  Idle value is %.0f A; bench_down restores it." % bl.Psu.IDLE_ISET)
    return "\n".join(msg)


def t_bench_bus_read(a):
    v, i = psu().meas()
    w = _f(v) * _f(i)
    return "VM bus: %s V x %s A = %s W" % (_fmt(v, 2), _fmt(i, 2), _fmt(w, 0))


def t_bench_torque_read(a):
    n = int(a.get("samples", 6))
    try:
        sc = scope()
    except Exception as e:
        _h["scope"] = None
        return ("scope unreachable (%s). It is LAN-only at 169.254.100.100:5555 — "
                "if Tailscale is up it hijacks 169.254/16 and blackholes this link. "
                "Never put this scope on USB; it wedges the board console." % e)
    tq = sc.torque_nm(n=n)
    if tq is None:
        return ("no valid torque reading (all samples returned the 9.9e37 sentinel). "
                "CH%d is clipping: give it room (the 0-10 V output needs ~2 V/div "
                "with a 10x probe), ':MEAS:CLE ALL', re-add VAVG,CHAN%d."
                % (bl.Scope.TORQUE_CH, bl.Scope.TORQUE_CH))
    return ("torque %.2f N·m (median of %d VAVG samples on CH%d, %g N·m/V)"
            % (tq, n, bl.Scope.TORQUE_CH, bl.Scope.NM_PER_V))


def t_bench_torque_rail(a):
    """Power the torque sensor's 24 V rail (Rigol CH1)."""
    on = a.get("on", True)
    ilim = float(a.get("ilim", 1.0))
    br = brake()
    v, i = br.torque_rail_set(bool(on), ilim=ilim)
    if not on:
        return "torque-sensor rail OFF (%.2f V / %.3f A). Torque readings are now meaningless." % (v, i)
    if i < 0.05:
        return ("torque-sensor rail commanded ON at 24.000 V but draw is only %.3f A "
                "— the sensor is not powering up. Check the rail wiring before "
                "trusting any torque number." % i)
    if i > 0.40:
        return ("torque-sensor rail ON: %.2f V / %.3f A — draw is well above the "
                "documented ~0.13 A. Check for a short before proceeding." % (v, i))
    return "torque-sensor rail ON: %.2f V / %.3f A (healthy is ~0.13 A)" % (v, i)


def t_bench_speed_read(a):
    """Wheel rpm from the torque sensor's speed output on CH3."""
    n = int(a.get("samples", 6))
    try:
        sc = scope()
    except Exception as e:
        _h["scope"] = None
        return "scope unreachable (%s)." % e
    if bl.Scope.RPM_PER_V is None:
        v = sc.vavg(bl.Scope.SPEED_CH, n=n)
        return ("speed output reads %s V average on CH%d, but Scope.RPM_PER_V is "
                "unset so it cannot be converted to rpm. Set the scale from the "
                "sensor datasheet first — an assumed scale would look right and "
                "be wrong. If the output is a pulse train, measure FREQ instead."
                % ("unreadable (clipping)" if v is None else "%.4f" % v,
                   bl.Scope.SPEED_CH))
    rpm = sc.speed_rpm(n=n)
    if rpm is None:
        return "no valid speed reading (CH%d clipping)." % bl.Scope.SPEED_CH
    return "speed %.1f rpm (median of %d VAVG samples on CH%d)" % (rpm, n, bl.Scope.SPEED_CH)


def t_bench_telemetry(a):
    """One parsed row of the CSV stream: the only read that sees temperatures."""
    b = board()
    b.stream_on()
    try:
        buf = ""
        row = None
        faults = []
        t0 = time.time()
        while time.time() - t0 < 4.0:
            time.sleep(0.4)
            buf += b.s.read(32000).decode(errors="replace")
            lines = buf.split("\n")
            buf = lines[-1]
            for ln in lines[:-1]:
                if ln.startswith("!"):
                    faults.append(ln.strip())
                m = SROW.match(ln.strip())
                if m:
                    row = m
            if row:
                break
    finally:
        try:
            b.stream_off()
        except Exception:
            pass
    out = []
    if faults:
        out.append("FAULT LINES: " + " | ".join(faults[:3]))
        out.append("  A DRV8353 latch needs a VM cycle (bench_vm_recover), not a replug.")
    if row is None:
        out.append("no telemetry row in 4 s. If the console answers 'hs' but never "
                   "streams, a hung LM75/I2C read can block the stream task — which "
                   "also blinds the thermal guard. A VM cycle recovers it.")
        return "\n".join(out)
    temps = [_f(row.group(k)) for k in (13, 14, 15, 16)]
    tmax = max([t for t in temps if t == t] or [float("nan")])
    out.append("iq %s A   rpm %s   temps %s   tmax %s C"
               % (_fmt(row.group(4), 1), _fmt(row.group(11), 0),
                  " ".join(_fmt(t, 1) for t in temps), _fmt(tmax, 1)))
    if any(t != t for t in temps):
        out.append("  nan temps = LM75 init failed; the thermal guard cannot see that "
                   "sensor.")
    out.append("  raw: " + row.group(0)[:200])
    return "\n".join(out)


def t_bench_console(a):
    cmd = str(a["command"]).strip()
    head = cmd.split()[0].lower() if cmd.split() else ""
    if head in MOTION_CMDS:
        return ("REFUSED: %r turns the wheel. This server is setup+measure only.\n"
                "Motion belongs in a script with a watchdog loop — abort on any '!' "
                "line, |iq| over the guard, temp > 75 C, dT/dt > 2 C/s, or 3 s of "
                "stream silence, with stop in a finally block. Crib "
                "%s/fig6_hold.py (power run) or fig4_heatsink.py (thermal dwell)."
                % (head, HERE))
    out = board().cmd(cmd, float(a.get("wait", 0.6)))
    if not out.strip():
        return ("no response to %r. The first command after an open can be mangled — "
                "retry once. Persistent silence = console wedge (bench_usb_replug)."
                % cmd)
    return out[:4000]


def t_bench_usb_replug(a):
    """Clears the stuck-macOS-driver console wedge. Board keeps running."""
    drop_board()
    try:
        port = bl.usb_replug()
    except Exception as e:
        return ("uhubctl cycle of hub %s port %s failed: %s"
                % (bl.BOARD_HUB[0], bl.BOARD_HUB[1], e))
    msg = ["USB replug done (uhubctl hub %s port %s power-cycled; the board kept "
           "running, only its USB reset)." % bl.BOARD_HUB]
    msg.append("port now: %s" % port)
    try:
        hs = board(fresh=True).cmd("hs", 0.8)
        msg.append("console %s" % ("ALIVE" if "#hall" in hs else "still MUTE — if a "
                                   "DRV fault is latched, use bench_vm_recover"))
    except Exception as e:
        drop_board()
        msg.append("console reopen failed: %s" % e)
    return "\n".join(msg)


def t_bench_vm_recover(a):
    """The other recovery: VM power cycle + ESP reset + verified re-arm."""
    drop_board()
    drop_psu()  # vm_recover.py wants the BK port to itself
    script = os.path.join(HERE, "vm_recover.py")
    try:
        r = subprocess.run(["/usr/bin/python3", script], capture_output=True,
                           text=True, timeout=280)
    except subprocess.TimeoutExpired:
        return "vm_recover.py timed out after 280 s."
    tail = (r.stdout + r.stderr).strip()[-1500:]
    verdict = "RECOVERED" if r.returncode == 0 else "FAILED after 3 attempts"
    return ("VM cycle %s (exit %d)\n%s\n\nThe board was reset: re-arm with "
            "bench_arm before any run." % (verdict, r.returncode, tail))


TOOLS = [
    ("bench_status",
     "Read the whole test stand at a glance: BK MR3K VM rail (output state, "
     "measured V/A, Iset), Rigol DP2031 CH1 torque-sensor rail and CH2 brake, "
     "board USB port and whether its console answers. Flags the known failure "
     "signatures (VM down, unpowered torque sensor, brake CC-clamping, console "
     "wedge). Start here — it is read-only and changes nothing.",
     {"type": "object", "properties": {}}, t_bench_status),

    ("bench_up",
     "Bring the bench up, in order, with every value already correct: verify "
     "the Rigol CH1 torque rail (24 V/~0.13 A, read-only — never written), park "
     "CH2 brake at 0 V with a 1 A limit, turn the BK VM output on and wait for "
     "the 48 V rail, wait for the board to enumerate, open the console with the "
     "stream quieted, and arm the firmware guards. Vset is never touched.",
     {"type": "object", "properties": {"arm_profile": {
         "type": "string", "enum": ["normal", "power"], "default": "normal",
         "description": "normal = lim 25 30 (everyday). power = lim 30 35, the "
                        "compile-time hard ceiling, for power runs only."}}},
     t_bench_up),

    ("bench_down",
     "Safe teardown in the right order: stop the motor, drop the brake to 0 V "
     "and turn CH2 off, restore BK Iset to the 8 A idle value, quiet the stream "
     "and close the console cleanly (an abrupt close with data in flight is the "
     "prime suspect for the console wedge). Leaves the VM on unless asked.",
     {"type": "object", "properties": {"power_off": {
         "type": "boolean", "default": False,
         "description": "Also switch the BK output off. The board drops off USB "
                        "when the VM goes down."}}},
     t_bench_down),

    ("bench_arm",
     "Re-apply the firmware guard limits and gate drive. Needed after any reset "
     "or VM cycle. Returns the firmware's acks — read them, because lim is "
     "clamped by a compile-time ceiling and may be lower than requested.",
     {"type": "object", "properties": {"profile": {
         "type": "string", "enum": ["normal", "power"], "default": "normal",
         "description": "normal = lim 25 30, vds 5, vl 20, odg 8. "
                        "power = lim 30 35 (hard ceiling) for power runs."}}},
     t_bench_arm),

    ("bench_brake_set",
     "Set the magnetic-particle brake voltage on Rigol CH2. Re-asserts the "
     "current limit on every write, because the channel can silently revert to "
     "0.1 A and CC-clamp at ~0.75 V while the brake starves. The unit is a "
     "200 N·m-class brake: 1-8 N·m is the bottom of its curve, roughly 2.5-3.7 V. "
     "Torque at a given voltage drifts up after high excitation — approach any "
     "edge from below.",
     {"type": "object",
      "properties": {"volts": {"type": "number",
                               "description": "0-24 V (clamped). ~2.5-3.7 V is the "
                                              "usual 8-12 N·m working range."},
                     "ilim": {"type": "number", "default": 3.0,
                              "description": "Current limit, capped at the 3 A the "
                                             "harness allows."},
                     "settle": {"type": "number", "default": 0.6}},
      "required": ["volts"]},
     t_bench_brake_set),

    ("bench_brake_off",
     "Release the brake: CH2 to 0 V and output off. A stalled wheel may need a "
     "full release before it will break away again.",
     {"type": "object", "properties": {}}, t_bench_brake_off),

    ("bench_psu_iset",
     "Set the BK MR3K current limit on the 48 V VM rail. Idle is 8 A; power runs "
     "raise it to 15-20 A and bench_down puts it back. The supply voltage is NOT "
     "reachable from this server — the whole characterized envelope assumes 48 V.",
     {"type": "object",
      "properties": {"amps": {"type": "number",
                              "description": "0.5-20 A (clamped). 8 = idle, "
                                             "15 = power runs, 20 = speed campaign."}},
      "required": ["amps"]},
     t_bench_psu_iset),

    ("bench_bus_read",
     "Measure the VM rail right now and return volts, amps and bus watts. This "
     "is the number the 514 W peak was recorded from.",
     {"type": "object", "properties": {}}, t_bench_bus_read),

    ("bench_torque_rail",
     "Power the dynamic torque sensor's 24 V rail (Rigol CH1) on or off. The "
     "voltage is fixed at 24.000 V and cannot be set from here — the whole "
     "campaign's torque scale assumes it. Only the current limit is adjustable. "
     "Warns if the draw is not the healthy ~0.13 A.",
     {"type": "object",
      "properties": {"on": {"type": "boolean", "default": True},
                     "ilim": {"type": "number", "default": 1.0,
                              "description": "Current limit, A. Clamped to 0.2-1.5."}}},
     t_bench_torque_rail),

    ("bench_speed_read",
     "Wheel rpm from the dynamic torque sensor's speed output on scope CH3. "
     "Refuses to invent a scale: until Scope.RPM_PER_V is set from the sensor "
     "datasheet it returns the raw volts and says so.",
     {"type": "object",
      "properties": {"samples": {"type": "integer", "default": 6}}},
     t_bench_speed_read),
    ("bench_torque_read",
     "Read shaft torque from the MHO984 scope (CH4, 20 N·m/V with the probe "
     "declared 10x, median of N samples). Explains itself if the scope is "
     "unreachable (Tailscale hijacks the 169.254 link-local the scope lives "
     "on) or if the channel is clipping.",
     {"type": "object", "properties": {"samples": {"type": "integer", "default": 6}}},
     t_bench_torque_read),

    ("bench_telemetry",
     "Grab one parsed row of the board's CSV stream — iq, rpm and the four "
     "temperatures — then re-quiet the stream. This is the only way to see "
     "temperatures, and it surfaces any firmware '!' fault lines. nan temps mean "
     "an LM75 failed, which also blinds the thermal guard.",
     {"type": "object", "properties": {}}, t_bench_telemetry),

    ("bench_console",
     "Send one command to the board console and return its reply, with the "
     "retry-friendly framing the port needs. Good for hs, enc, r, cap/capdump, "
     "hset, sq, vds, odg. Commands that turn the wheel (run, hrun, hiq, erun, "
     "eiq, ecal, hcal, epos, pg) are refused: motion needs a watchdog loop, so "
     "it belongs in a script.",
     {"type": "object",
      "properties": {"command": {"type": "string"},
                     "wait": {"type": "number", "default": 0.6,
                              "description": "Seconds to wait for the reply."}},
      "required": ["command"]},
     t_bench_console),

    ("bench_usb_replug",
     "Recovery #1, for a MUTE CONSOLE: the USB-JTAG console goes silent and even "
     "esptool cannot reach the ROM, because the macOS driver instance is stuck. "
     "Chip resets do not clear it; USB re-enumeration does. Power-cycles the "
     "board's dock hub port with uhubctl — the board keeps running, only USB "
     "resets. Use this when the console is dead but there is no DRV fault.",
     {"type": "object", "properties": {}}, t_bench_usb_replug),

    ("bench_vm_recover",
     "Recovery #2, for a LATCHED DRV8353 FAULT (a '! DRV8353 fault 0x...' line) "
     "or a chip wedged hard enough that esptool cannot connect: cycles the VM "
     "rail, resets the ESP and re-arms the limits, verified, up to 3 attempts. "
     "This is NOT the same failure as a mute console — a replug will not clear a "
     "latch, and a VM cycle is overkill for a wedge. Re-arm afterwards.",
     {"type": "object", "properties": {}}, t_bench_vm_recover),
]

DISPATCH = dict((name, fn) for name, _d, _s, fn in TOOLS)


# ---------------------------------------------------------------- protocol


def handle(msg):
    method = msg.get("method")
    mid = msg.get("id")
    if mid is None:
        return None  # notification

    def ok(result):
        return {"jsonrpc": "2.0", "id": mid, "result": result}

    if method == "initialize":
        pv = (msg.get("params") or {}).get("protocolVersion") or "2024-11-05"
        return ok({"protocolVersion": pv,
                   "capabilities": {"tools": {}},
                   "serverInfo": SERVER})
    if method == "ping":
        return ok({})
    if method == "tools/list":
        return ok({"tools": [{"name": n, "description": d, "inputSchema": s}
                             for n, d, s, _fn in TOOLS]})
    if method == "tools/call":
        params = msg.get("params") or {}
        name = params.get("name")
        args = params.get("arguments") or {}
        fn = DISPATCH.get(name)
        if fn is None:
            return ok({"content": [{"type": "text", "text": "unknown tool %r" % name}],
                       "isError": True})
        try:
            text = fn(args)
            return ok({"content": [{"type": "text", "text": text}]})
        except Exception as e:
            import traceback
            traceback.print_exc(file=sys.stderr)
            return ok({"content": [{"type": "text",
                                    "text": "%s: %s" % (type(e).__name__, e)}],
                       "isError": True})
    return {"jsonrpc": "2.0", "id": mid,
            "error": {"code": -32601, "message": "method not found: %s" % method}}


def main():
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            msg = json.loads(line)
        except ValueError:
            continue
        resp = handle(msg)
        if resp is not None:
            _OUT.write(json.dumps(resp) + "\n")
            _OUT.flush()


if __name__ == "__main__":
    try:
        main()
    finally:
        for closer in (drop_board, drop_psu):
            try:
                closer()
            except Exception:
                pass
