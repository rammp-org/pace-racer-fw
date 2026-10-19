#!/usr/bin/env python3
"""Tiny DHCP server standing in for the MIB on the bench switch.

Leases 169.254.50.10+ /16, so boards land next to the laptop's own 169.254
address on the bench interface. The interface needs an IPv4 address first:
macOS and Windows self-assign one; on Linux add it yourself
(`sudo ip addr add 169.254.50.1/16 dev eth0`).

    python3 mock_mib_dhcp.py [iface]   # macOS: en7 (default). Linux: eth0, enp3s0...
                                       # Windows: the adapter name, e.g. "Ethernet 2"
    python3 mock_mib_dhcp.py --list    # interfaces with an IPv4 address

Per OS: macOS runs unprivileged and pins the socket to the interface
(IP_BOUND_IF) so offers go out the bench dongle, not the Wi-Fi default route.
Linux needs root for port 67 (sudo, or CAP_NET_BIND_SERVICE) and pins with
SO_BINDTODEVICE. Windows binds the interface's address; allow it through the
firewall when asked.
"""
import os, socket, struct, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eth_link  # noqa: E402  (interface list per OS)

POOL_BASE = (169, 254, 50, 10)
MASK = bytes([255, 255, 0, 0])
LEASE_S = 3600
IFACES = eth_link._ifaces()

if "--list" in sys.argv:
    for name, ip, mask in IFACES:
        print(f"{name:24s} {ip:16s} {mask}")
    sys.exit(0)
IFACE = sys.argv[1] if len(sys.argv) > 1 else ("en7" if sys.platform == "darwin" else None)
if not IFACE:
    sys.exit("name the bench interface: " + ", ".join(n for n, _, _ in IFACES) + " (--list for addresses)")
SERVER_IP = next((ip for n, ip, _ in IFACES if n == IFACE), None)
if not SERVER_IP:
    sys.exit(f"{IFACE} has no IPv4 address yet (have: {', '.join(n for n, _, _ in IFACES) or 'none'})")
server_ip_b = socket.inet_aton(SERVER_IP)

leases = {}  # mac -> ip bytes


def lease_for(mac):
    if mac not in leases:
        n = len(leases)
        ip = bytes([POOL_BASE[0], POOL_BASE[1], POOL_BASE[2] + (POOL_BASE[3] + n) // 256, (POOL_BASE[3] + n) % 256])
        if ip == server_ip_b:  # never hand out our own address
            n += 1
            ip = bytes([POOL_BASE[0], POOL_BASE[1], POOL_BASE[2] + (POOL_BASE[3] + n) // 256, (POOL_BASE[3] + n) % 256])
        leases[mac] = ip
    return leases[mac]


def parse_options(data):
    opts = {}
    i = 240
    while i < len(data):
        t = data[i]
        if t == 255:
            break
        if t == 0:
            i += 1
            continue
        l = data[i + 1]
        opts[t] = data[i + 2:i + 2 + l]
        i += 2 + l
    return opts


def build_reply(req, msg_type, yiaddr):
    xid = req[4:8]
    flags = req[10:12]
    chaddr = req[28:44]
    hdr = b"\x02\x01\x06\x00" + xid + b"\x00\x00" + flags + b"\x00" * 4 + yiaddr + server_ip_b + b"\x00" * 4 + chaddr
    hdr += b"\x00" * 192 + b"\x63\x82\x53\x63"
    opts = bytes([53, 1, msg_type])
    opts += bytes([54, 4]) + server_ip_b
    opts += bytes([51, 4]) + struct.pack("!I", LEASE_S)
    opts += bytes([1, 4]) + MASK
    opts += bytes([3, 4]) + server_ip_b
    opts += bytes([28, 4]) + bytes([169, 254, 255, 255])
    opts += b"\xff"
    return hdr + opts


s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
try:
    if sys.platform.startswith("win"):
        s.bind((SERVER_IP, 67))  # Windows delivers the interface's broadcasts to its address
    else:
        eth_link._pin_to_iface(s, IFACE)
        s.bind(("", 67))
except PermissionError:
    sys.exit("port 67 needs root here: run it with sudo")
print(f"mock MIB DHCP on {IFACE} ({SERVER_IP}), pool from 169.254.50.10", flush=True)

while True:
    data, addr = s.recvfrom(2048)
    if len(data) < 240 or data[0] != 1:
        continue
    opts = parse_options(data)
    mt = opts.get(53, b"\x00")[0]
    mac = data[28:34].hex(":")
    host = opts.get(12, b"").decode(errors="replace")
    ip = lease_for(mac)
    ipstr = socket.inet_ntoa(ip)
    name = {1: "DISCOVER", 3: "REQUEST", 4: "DECLINE", 7: "RELEASE", 8: "INFORM"}.get(mt, str(mt))
    print(f"{time.strftime('%H:%M:%S')} {name:8s} from {mac} host='{host}' -> {ipstr}", flush=True)
    if mt == 1:
        s.sendto(build_reply(data, 2, ip), ("255.255.255.255", 68))
    elif mt == 3:
        s.sendto(build_reply(data, 5, ip), ("255.255.255.255", 68))
        print(f"           ACK {ipstr} to {mac}", flush=True)
