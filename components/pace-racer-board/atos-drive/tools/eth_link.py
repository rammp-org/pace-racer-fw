#!/usr/bin/env python3
"""Talk to a PACE RACER board over ethernet: find it, flash it, drive its
console, reset it. The board's USB port is inside the wheel; this is the
ethernet twin of the WebUSB tools (same frames, same modules, see
main/eth_link.hpp).

    tools/eth_link.py list                         # every board that answers "PACE?" on the LAN
    tools/eth_link.py <board> info                 # JSON: axis, fw, partition, pending_verify, ...
    tools/eth_link.py <board> flash [app.bin]      # OTA, wait for it to come back, confirm it
    tools/eth_link.py <board> status|mark-valid|rollback
    tools/eth_link.py <board> reset
    tools/eth_link.py <board> cmd "axis"           # one console line, print what it said
    tools/eth_link.py <board> console              # interactive console (Ctrl-D to leave)

<board> is an IP address, an axis name (drive_left), an axis number (2), or
'auto' (exactly one board must answer). Names and 'auto' go through the UDP
discovery on :5556, which is a broadcast question and a unicast answer, so it
works through the bench switch that eats board-originated multicast.

Also importable: provision.py uses discover(), EthConsole, flash(), reset().

Transport notes: 169.254/16 targets are reached from a socket bound to the
physical interface that owns that range, because Tailscale's utun claims the
whole range on this Mac and would blackhole the traffic otherwise.
"""
import argparse
import json
import os
import re
import socket
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(APP, "managed_components", "espp__ota", "python"))
from espp_ota import frame as sf  # noqa: E402
from espp_ota.client import OtaClient  # noqa: E402
from espp_ota.protocol import OtaError  # noqa: E402

SERVICE_PORT = 5555
DISCOVERY_PORT = 5556
CONSOLE_MODULE, SYSTEM_MODULE = 0x10, 0x20
CON_LINE, CON_ACK, CON_LOG = 0x01, 0x81, 0x82
SYS_RESET, SYS_INFO, SYS_OK, SYS_INFO_REPLY = 0x01, 0x02, 0x81, 0x82
AXES = {0: "front_caster_left", 1: "front_caster_right", 2: "drive_left", 3: "drive_right"}
DEFAULT_BIN = os.path.join(APP, "build-hub20", "pace_racer_atos_drive.bin")
IP_BOUND_IF = 25  # macOS setsockopt(IPPROTO_IP, ...) to pin a socket to one interface


class LinkError(RuntimeError):
    pass


# ---------------------------------------------------------------- interfaces
def _ifaces():
    """[(name, ipv4, netmask)] of physical interfaces with an IPv4 address."""
    out = subprocess.run(["ifconfig"], capture_output=True, text=True).stdout
    found, name = [], None
    for ln in out.splitlines():
        if ln and not ln[0].isspace():
            name = ln.split(":")[0]
        elif name and not name.startswith(("utun", "lo", "bridge", "awdl", "llw")) and "inet " in ln:
            parts = ln.split()
            ip = parts[1]
            mask = "255.255.255.255"
            if "netmask" in parts:
                mask = socket.inet_ntoa(int(parts[parts.index("netmask") + 1], 16).to_bytes(4, "big"))
            found.append((name, ip, mask))
    return found


def _same_subnet(ip, net_ip, mask):
    a, b, m = (int.from_bytes(socket.inet_aton(x), "big") for x in (ip, net_ip, mask))
    return (a & m) == (b & m)


def _bind_for(sock, target_ip):
    """Bind the socket's source to the interface that owns the target's subnet."""
    for name, ip, mask in _ifaces():
        if _same_subnet(target_ip, ip, mask):
            sock.bind((ip, 0))
            if sys.platform == "darwin":
                try:
                    sock.setsockopt(socket.IPPROTO_IP, IP_BOUND_IF, socket.if_nametoindex(name))
                except OSError:
                    pass
            return name
    return None


# ---------------------------------------------------------------- discovery
def discover(timeout=1.5, ifaces=None):
    """Broadcast "PACE?" from every physical interface; return [info dict] with 'ip'."""
    socks = []
    for name, ip, mask in ifaces or _ifaces():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.bind((ip, 0))
            if sys.platform == "darwin":
                s.setsockopt(socket.IPPROTO_IP, IP_BOUND_IF, socket.if_nametoindex(name))
        except OSError:
            s.close()
            continue
        s.setblocking(False)
        socks.append(s)
        ipn = int.from_bytes(socket.inet_aton(ip), "big")
        mn = int.from_bytes(socket.inet_aton(mask), "big")
        directed = socket.inet_ntoa(((ipn & mn) | (~mn & 0xFFFFFFFF)).to_bytes(4, "big"))
        for dst in {"255.255.255.255", directed}:
            try:
                s.sendto(b"PACE?", (dst, DISCOVERY_PORT))
            except OSError:
                pass
    boards, t0 = {}, time.time()
    while time.time() - t0 < timeout:
        for s in socks:
            try:
                data, (ip, _) = s.recvfrom(2048)
            except (BlockingIOError, OSError):
                continue
            try:
                info = json.loads(data.decode())
            except ValueError:
                continue
            info["ip"] = ip
            boards[ip] = info
        time.sleep(0.05)
    for s in socks:
        s.close()
    return sorted(boards.values(), key=lambda b: b.get("axis", 99))


def resolve(target, timeout=1.5):
    """IP, axis name, axis number or 'auto' -> (ip, info-or-None)."""
    if re.match(r"^\d+\.\d+\.\d+\.\d+$", target):
        return target, None
    boards = discover(timeout)
    if target == "auto":
        if len(boards) != 1:
            raise LinkError(f"'auto' needs exactly one board on the LAN, found {len(boards)}: "
                            + ", ".join(f"{b['ip']} {b.get('axis_name')}" for b in boards))
        return boards[0]["ip"], boards[0]
    want_num = int(target) if target.isdigit() else None
    hits = [b for b in boards if b.get("axis_name") == target or b.get("axis") == want_num]
    if len(hits) != 1:
        raise LinkError(f"{len(hits)} boards match '{target}' (found: "
                        + ", ".join(f"{b['ip']} {b.get('axis_name')}" for b in boards) + ")")
    return hits[0]["ip"], hits[0]


# ---------------------------------------------------------------- transport
class TcpTransport:
    """write(bytes, timeout_ms) / read(max_len, timeout_ms) -> bytes, b'' on
    timeout: what espp_ota.OtaClient expects of a transport."""

    def __init__(self, host, port=SERVICE_PORT, connect_timeout=3.0):
        self.host, self.port, self.connect_timeout = host, port, connect_timeout
        self.sock = None

    def open(self):
        if self.sock:
            return self
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        _bind_for(s, self.host)
        s.settimeout(self.connect_timeout)
        try:
            s.connect((self.host, self.port))
        except OSError as e:
            s.close()
            raise LinkError(f"connect {self.host}:{self.port}: {e}")
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock = s
        return self

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    def write(self, data, timeout_ms=5000):
        self.sock.settimeout(timeout_ms / 1000)
        self.sock.sendall(data)

    def read(self, max_len, timeout_ms=5000):
        self.sock.settimeout(timeout_ms / 1000)
        try:
            d = self.sock.recv(max_len)
        except socket.timeout:
            return b""
        if not d:
            raise LinkError("connection closed by the board")
        return d

    @property
    def description(self):
        return f"{self.host}:{self.port}"


def _await(t, module, types, timeout):
    """Read until a frame of `module` with a type in `types` arrives."""
    parser = sf.StreamParser()
    deadline = time.time() + timeout
    while time.time() < deadline:
        d = t.read(sf.MAX_FRAME_SIZE, timeout_ms=200)
        for f in parser.feed(d):
            if f.module == module and f.type in types:
                return f
    return None


# ---------------------------------------------------------------- console
class EthConsole:
    """Same shape as provision.py's serial Console: cmd(line, wait) and listen(secs)
    return the text the board printed."""

    def __init__(self, host, port=SERVICE_PORT):
        self.t = TcpTransport(host, port).open()
        self.parser = sf.StreamParser()
        self.cmd("sq 1", wait=0.4)

    def close(self):
        self.t.close()

    def _pump(self, secs):
        buf = b""
        t0 = time.time()
        while time.time() - t0 < secs:
            d = self.t.read(sf.MAX_FRAME_SIZE, timeout_ms=100)
            for f in self.parser.feed(d):
                if f.module == CONSOLE_MODULE and f.type == CON_LOG:
                    buf += f.payload
        return buf.decode("utf-8", "replace")

    def cmd(self, line, wait=1.0):
        self.t.write(sf.build_frame(CONSOLE_MODULE, CON_LINE, line.encode()))
        return self._pump(wait)

    def listen(self, secs):
        return self._pump(secs)


# ---------------------------------------------------------------- actions
def info(host):
    with TcpTransport(host) as t:
        t.write(sf.build_frame(SYSTEM_MODULE, SYS_INFO))
        f = _await(t, SYSTEM_MODULE, (SYS_INFO_REPLY,), 3.0)
    if f is None:
        raise LinkError("no INFO reply")
    return json.loads(f.payload.decode())


def reset(host):
    with TcpTransport(host) as t:
        t.write(sf.build_frame(SYSTEM_MODULE, SYS_RESET))
        f = _await(t, SYSTEM_MODULE, (SYS_OK,), 3.0)
    if f is None:
        raise LinkError("reset not acknowledged")


def wait_for_board(host, timeout=90.0, say=print):
    """After a reset the W5500 re-leases; poll until the service port answers INFO."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            return info(host)
        except (LinkError, OSError, ValueError):
            time.sleep(1.0)
    raise LinkError(f"{host}: no answer on :{SERVICE_PORT} after {timeout:.0f} s")


def flash(host, binary, say=print, confirm=True):
    """OTA the image, wait for the board to come back on the same IP, confirm it."""
    image = open(binary, "rb").read()
    if not image:
        raise LinkError("image is empty")
    with TcpTransport(host) as t:
        client = OtaClient(t)
        try:
            before = client.get_status().firmware_str()
        except OtaError:
            before = "?"
        say(f"flashing {os.path.basename(binary)} ({len(image) / 1024:.0f} KiB) -> {host}, was {before}")
        t0, last = time.time(), [0]

        def prog(written, total):
            pct = int(100 * written / max(total, 1))
            if pct // 10 != last[0] // 10:
                say(f"  {pct}%")
            last[0] = pct

        client = OtaClient(t, chunk_size=4096, progress=prog)
        client.flash(image, image_size=len(image))
        say(f"OTA complete in {time.time() - t0:.1f} s; board restarting")
    if not confirm:
        return None
    time.sleep(3.0)
    inf = wait_for_board(host, say=say)
    say(f"back: {inf.get('version')} on {inf.get('partition')}, pending_verify={inf.get('pending_verify')}")
    with TcpTransport(host) as t:
        client = OtaClient(t)
        st = client.get_status()
        if st.pending_verify:
            client.mark_valid()
            say("new image answered over ethernet: marked valid, rollback cancelled")
        else:
            say("image already confirmed")
    return inf


def mark_valid(host):
    with TcpTransport(host) as t:
        OtaClient(t).mark_valid()


def rollback(host):
    with TcpTransport(host) as t:
        OtaClient(t).mark_invalid()


def status(host):
    with TcpTransport(host) as t:
        return OtaClient(t).get_status()


# ---------------------------------------------------------------- cli
def _say(msg):
    print(f"== {msg}", flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("target", help="'list', or a board: IP, axis name, axis number, 'auto'")
    ap.add_argument("action", nargs="?", default="info",
                    choices=["info", "flash", "status", "mark-valid", "rollback", "reset", "cmd", "console"])
    ap.add_argument("arg", nargs="?", help="binary for flash (default build-hub20), line for cmd")
    ap.add_argument("--no-confirm", action="store_true", help="flash: do not mark the new image valid")
    ap.add_argument("--wait", type=float, default=1.5, help="cmd: seconds to collect output")
    a = ap.parse_args(argv)

    if a.target == "list":
        boards = discover()
        if not boards:
            print("no boards answered on udp :%d (ifaces: %s)" % (
                DISCOVERY_PORT, ", ".join(f"{n} {ip}" for n, ip, _ in _ifaces())))
            return 1
        for b in boards:
            print(f"{b['ip']:15s} axis {b.get('axis')} {b.get('axis_name', '?'):18s} fb {b.get('fb', '?')} "
                  f"hall_cal {b.get('hall_cal')} motor {b.get('motor', '?')} fw {b.get('version', '?')} "
                  f"{b.get('partition', '?')}{' PENDING-VERIFY' if b.get('pending_verify') else ''}")
        return 0

    host, _ = resolve(a.target)
    if a.action == "info":
        print(json.dumps(info(host), indent=2))
    elif a.action == "flash":
        flash(host, a.arg or DEFAULT_BIN, say=_say, confirm=not a.no_confirm)
    elif a.action == "status":
        st = status(host)
        print(f"{host}: {st.firmware_str()}" + (" PENDING VERIFY" if st.pending_verify else " confirmed"))
    elif a.action == "mark-valid":
        mark_valid(host)
        print("marked valid")
    elif a.action == "rollback":
        rollback(host)
        print("rolling back to the previous image; board restarting")
    elif a.action == "reset":
        reset(host)
        print("reset acknowledged; board restarts in ~300 ms")
    elif a.action == "cmd":
        if not a.arg:
            ap.error("cmd needs a console line")
        c = EthConsole(host)
        sys.stdout.write(c.cmd(a.arg, wait=a.wait))
        c.close()
    elif a.action == "console":
        c = EthConsole(host)
        print(f"console on {host}; type commands, Ctrl-D to leave")
        try:
            while True:
                try:
                    line = input("> ")
                except EOFError:
                    break
                if line.strip():
                    sys.stdout.write(c.cmd(line.strip(), wait=a.wait))
        finally:
            c.close()
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except LinkError as e:
        print(f"!! {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(130)
