"""Shared bench control for the figures campaign."""
import serial, time, re, sys, os, json

def _find_board_port():
    import glob
    ports = [p for p in glob.glob("/dev/cu.usbmodem*") if "615J" not in p]
    if not ports:
        raise RuntimeError("board USB port not found")
    return ports[0]


BOARD_HUB = ("3-1", "1")  # uhubctl location/port — the dock hub the board sits on


def usb_replug(wait=6.0):
    """Software 'replug': power-cycle the board's hub port. Clears the
    stuck-macOS-driver console wedge that nothing chip-side can."""
    import subprocess
    subprocess.run(["uhubctl", "-l", BOARD_HUB[0], "-p", BOARD_HUB[1], "-a", "cycle",
                    "-d", "2"], capture_output=True, timeout=30)
    time.sleep(wait)
    return _find_board_port()
PSU = "/dev/cu.usbmodem615J231251"
BRAKE_VISA = "USB0::6833::42152::DP2A284M00016::0::INSTR"
LIMITS = ["lim 25 30", "vds 5", "vl 20"]  # vds 5 authorized 2026-08-20
KT = 0.62  # N·m/A, measured 2026-08
GEAR = 2.5


class Board:
    def __init__(self):
        self.s = serial.Serial(_find_board_port(), 115200, timeout=0.3)
        time.sleep(0.4)
        self.s.reset_input_buffer()
        # quiet the CSV firehose immediately: data in flight across an abrupt
        # close is the prime suspect for the replug-only console wedge.
        # Scripts that parse stream rows call stream_on()/stream_off().
        try:
            self.ack("sq 1", "#stream")
        except RuntimeError:
            pass  # older firmware without 'sq' — proceed loud

    def stream_on(self):
        self.ack("sq 0", "#stream")

    def stream_off(self):
        self.ack("sq 1", "#stream")
        time.sleep(0.5)
        self.s.read(65536)

    def cmd(self, c, wait=0.4):
        self.s.write((c + "\n").encode())
        time.sleep(wait)
        return self.s.read(32000).decode(errors="replace")

    def ack(self, c, tok, tries=4):
        for _ in range(tries):
            out = self.cmd(c)
            if tok in out:
                return out
        raise RuntimeError(f"no ack for {c!r}: {out[:300]}")

    def arm_limits(self):
        for lim in LIMITS:
            self.ack(lim, "#")

    def ecal(self, amps=4, rpm=12):
        """Smooth rotating cal: I/f spin both directions, offset = circular
        mean of (drive angle - raw encoder electrical angle). The fwd/rev
        average cancels the load-angle bias; no stepped-angle lurching.
        Returns (offset_deg, spread_deg) where spread = |fwd - rev| mean
        difference (2x the load angle; a mount-health indicator)."""
        import math
        self.ack("hs", "#hall")           # burn the possibly-mangled first cmd
        self.ack("ho 9999 45 1", "#handoff")
        means = []
        for sign in (1, -1):
            self.ack("cap 20 2600 2", "#cap armed")
            self.ack(f"run {amps} {rpm * sign} 2", "#")
            time.sleep(3.4)
            self.stop()
            meta, rows = self.capdump()
            seg = rows[int(len(rows) * 0.55):]
            # require actual rotation in the window (unwrap the enc column)
            trav = 0.0
            for a, b2 in zip(seg, seg[1:]):
                d = b2[1] - a[1]
                if d > 180: d -= 360
                elif d < -180: d += 360
                trav += abs(d)
            if trav < 360:
                raise RuntimeError(f"ecal: rotor barely moved ({trav:.0f} elec-deg) — "
                                   "breakaway failed or capture misaligned")
            ss = sum(math.sin(math.radians(r[0] - r[1])) for r in seg)
            cs = sum(math.cos(math.radians(r[0] - r[1])) for r in seg)
            means.append(math.degrees(math.atan2(ss, cs)))
        d = (means[0] - means[1] + 180) % 360 - 180
        ss = sum(math.sin(math.radians(m)) for m in means)
        cs = sum(math.cos(math.radians(m)) for m in means)
        ofs = math.degrees(math.atan2(ss, cs)) % 360
        self.ack(f"eofs {ofs:.2f}", "#enc ofs")
        return ofs, abs(d)

    def capdump(self, timeout=40):
        """Returns (meta, rows) where rows = [(a,b,c), ...]."""
        self.s.reset_input_buffer()
        self.s.write(b"capdump\n")
        buf = ""
        t0 = time.time()
        while time.time() - t0 < timeout:
            chunk = self.s.read(32000).decode(errors="replace")
            buf += chunk
            if "#capend" in buf:
                break
            time.sleep(0.05)
        m = re.search(r"#capdump n=(\d+) every=(\d+) mode=(\d+)", buf)
        if not m:
            raise RuntimeError("capdump header missing: " + buf[:300])
        meta = {"n": int(m.group(1)), "every": int(m.group(2)), "mode": int(m.group(3)),
                "dt": int(m.group(2)) / 20000.0}
        rows = []
        for ln in buf.splitlines():
            if ln.startswith("@"):
                try:
                    _, a, b, c = ln[1:].split(",")[0], *ln.split(",")[1:4]
                    idx = int(ln[1:].split(",")[0])
                    rows.append((idx, float(a), float(b), float(c)))
                except (ValueError, IndexError):
                    pass
        rows.sort()
        return meta, [(r[1], r[2], r[3]) for r in rows]

    def stop(self):
        self.cmd("stop", 0.5)
        time.sleep(1.5)
        self.s.read(32000)

    def enc_acc_rotor_deg(self):
        for _ in range(4):
            out = self.cmd("enc", 0.3)
            m = re.search(r"acc=(-?[\d.]+)deg", out)
            if m:
                return float(m.group(1)) / GEAR
        raise RuntimeError("enc read failed")

    def close(self):
        # gentle shutdown: silence the stream, drain, then close
        try:
            self.stream_off()
        except (RuntimeError, OSError):
            pass
        self.s.close()


class Brake:
    def __init__(self):
        import pyvisa
        self.dp = pyvisa.ResourceManager("@py").open_resource(BRAKE_VISA)
        self.dp.timeout = 5000

    def set(self, volts, settle=0.6, ilim=1.0):
        # 200 N-m brake: the old 2.5 V cap was just the bottom of its curve.
        # Coil ~7.6 ohm; watch meas() for CC clamping at high commands.
        volts = max(0.0, min(24.0, volts))
        # ALWAYS re-assert the current limit: at the Rigol's 0.1 A default the
        # channel CC-clamps to ~0.75 V and the brake silently starves
        # (cost us the first 500 W attempt on 2026-08-19).
        self.dp.write(f":SOUR2:CURR {ilim:.3f}")
        self.dp.write(f":SOUR2:VOLT {volts:.3f}")
        if volts > 0:
            self.dp.write(":OUTP:STAT CH2,ON")
        if settle:
            time.sleep(settle)
        return volts

    def off(self):
        self.dp.write(":SOUR2:VOLT 0.000")
        self.dp.write(":OUTP:STAT CH2,OFF")

    def meas(self):
        return (float(self.dp.query(":MEAS:VOLT? CH2")),
                float(self.dp.query(":MEAS:CURR? CH2")))

    def close(self):
        self.dp.close()


def save(datadir, name, obj):
    os.makedirs(datadir, exist_ok=True)
    with open(os.path.join(datadir, name + ".json"), "w") as f:
        json.dump(obj, f)
    print(f"saved {name} ({len(json.dumps(obj))} bytes)")


class Scope:
    """MHO984 over raw LAN socket (port 5555), source-bound to the dock link.
    No pyvisa/libusb — the scope's USB presence was wedging the board's
    USB-JTAG console (2026-08-20). Torque: 0-10 V = 0-200 N·m."""
    ADDR = ("169.254.100.100", 5555)
    SRC = "169.254.91.251"
    NM_PER_V = 200.0  # 10:1 probe attenuation confirmed vs sensor display 2026-08-20
    SENTINEL = 9.0e37

    def __init__(self):
        import socket as _s, subprocess
        # link-local source addresses change when the dock re-enumerates —
        # discover whatever 169.254.x.x this host currently owns
        out = subprocess.run(["ifconfig"], capture_output=True, text=True).stdout
        srcs = [ln.split()[1] for ln in out.splitlines()
                if ln.strip().startswith("inet 169.254")]
        last_err = None
        for cand in srcs + [None]:
            try:
                self.sock = _s.socket(_s.AF_INET, _s.SOCK_STREAM)
                if cand:
                    self.sock.bind((cand, 0))
                self.sock.settimeout(3)
                self.sock.connect(self.ADDR)
                break
            except OSError as e:
                last_err = e
                self.sock.close()
        else:
            raise last_err
        self.q("*IDN?")
        self.w(":MEAS:ITEM VAVG,CHAN1")
        self.w(":RUN")
        time.sleep(0.6)

    def w(self, cmd):
        self.sock.sendall((cmd + "\n").encode())

    def q(self, cmd):
        self.w(cmd)
        return self.sock.recv(300).decode(errors="replace").strip()

    def torque_nm(self, n=6, settle=0.2):
        import statistics
        vals = []
        for _ in range(n):
            try:
                v = float(self.q(":MEAS:ITEM? VAVG,CHAN1"))
                if abs(v) < self.SENTINEL:
                    vals.append(v * self.NM_PER_V)
            except (ValueError, OSError):
                pass
            time.sleep(settle)
        return statistics.median(vals) if vals else None

    def close(self):
        try: self.sock.close()
        except OSError: pass
