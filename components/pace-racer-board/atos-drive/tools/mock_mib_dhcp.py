#!/usr/bin/env python3
"""Tiny DHCP server standing in for the MIB on the bench switch.

Runs unprivileged on macOS (UDP 67 is bindable without root). Binds the socket
to one interface with IP_BOUND_IF so offers go out the bench dongle and not the
Wi-Fi default route. Leases 169.254.50.10+ /16 so clients land next to the
Mac's own self-assigned 169.254 address on the same interface.

    python3 mock_mib_dhcp.py [iface]      # default en7
"""
import socket, struct, sys, time, subprocess

IFACE = sys.argv[1] if len(sys.argv) > 1 else "en7"
POOL_BASE = (169, 254, 50, 10)
MASK = bytes([255, 255, 0, 0])
LEASE_S = 3600
IP_BOUND_IF = 25


def iface_ip(name):
    out = subprocess.run(["ipconfig", "getifaddr", name], capture_output=True, text=True).stdout.strip()
    if not out:
        sys.exit(f"{name} has no IPv4 address yet")
    return out


SERVER_IP = iface_ip(IFACE)
server_ip_b = socket.inet_aton(SERVER_IP)
if_index = socket.if_nametoindex(IFACE)

leases = {}  # mac -> ip bytes


def lease_for(mac):
    if mac not in leases:
        n = len(leases)
        leases[mac] = bytes([POOL_BASE[0], POOL_BASE[1], POOL_BASE[2] + (POOL_BASE[3] + n) // 256, (POOL_BASE[3] + n) % 256])
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
s.setsockopt(socket.IPPROTO_IP, IP_BOUND_IF, if_index)
s.bind(("", 67))
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
