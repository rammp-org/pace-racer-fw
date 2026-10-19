# Bench instrument handoff: find and talk to the Rigol scope and supply

Paste this whole file into Claude Code on the Windows laptop that is cabled to the bench switch.

## What is on the switch

Two Rigol instruments. Both speak raw SCPI over TCP port 5555 (newline-terminated, no VISA, no drivers).

| Device | Model | MAC | Addressing | Expected `*IDN?` reply |
|---|---|---|---|---|
| Power supply | DP2031 | 00-19-AF-17-00-D8 | DHCP client, leases from this laptop's DHCP server | `Rigol Technologies,DP2031,DP2A284M00016,...` |
| Oscilloscope | MHO984 | 00-19-AF-81-04-0A | Static 192.168.1.101 / 255.255.255.0 | `RIGOL TECHNOLOGIES,MHO984,MHO9B286M00520,...` |

A PACE motor board (ESP32, MAC prefix 10:bd:a3) may also be on the switch. Ignore it for this task.

## Task

Locate both instruments, confirm each answers `*IDN?`, and report their IP addresses. Use only read-only queries.

## Steps

1. Confirm the Ethernet adapter is up and has an IPv4 address:
   ```powershell
   Get-NetIPAddress -AddressFamily IPv4 | Where-Object InterfaceAlias -notlike "*Loopback*" | Format-Table InterfaceAlias, IPAddress, PrefixLength
   ```
2. Find the supply in the DHCP server's lease table by MAC `00-19-AF-17-00-D8`. If the lease table is not accessible, use `arp -a` after step 4, or sweep (step 4).
3. The scope is on 192.168.1.0/24. If the Ethernet adapter is not on that subnet, add a second address to the adapter (needs an admin PowerShell, replace the alias if it is not "Ethernet"):
   ```powershell
   New-NetIPAddress -InterfaceAlias "Ethernet" -IPAddress 192.168.1.50 -PrefixLength 24
   ```
   Do not set a gateway on it. Do not remove the adapter's existing address.
4. Probe. Save and run this script; it sweeps every /24 the adapter is on, plus 192.168.1.0/24, for anything answering `*IDN?` on port 5555:
   ```python
   import socket, ipaddress, concurrent.futures, subprocess, re

   nets = {"192.168.1.0/24"}
   out = subprocess.run(["ipconfig"], capture_output=True, text=True).stdout
   for ip in re.findall(r"IPv4 Address[ .]*: (\d+\.\d+\.\d+\.\d+)", out):
       if not ip.startswith("127."):
           nets.add(str(ipaddress.ip_network(ip + "/24", strict=False)))

   def probe(ip):
       try:
           s = socket.create_connection((str(ip), 5555), timeout=0.5)
           s.sendall(b"*IDN?\n"); s.settimeout(1.5)
           return str(ip), s.recv(256).decode(errors="replace").strip()
       except OSError:
           return None

   hosts = [h for n in nets for h in ipaddress.ip_network(n).hosts()]
   with concurrent.futures.ThreadPoolExecutor(64) as ex:
       for r in ex.map(probe, hosts):
           if r: print(r[0], "->", r[1])
   ```
5. Report the two IP addresses and the `*IDN?` strings. The supply's own IP display and its `:SYST:COMM:LAN:IPAD?` reply can lag or read 0.0.0.0 while it is actually serving; trust the socket, not the panel.

## If the supply is not found

It only takes a lease at boot. Power-cycle the supply with its front switch, wait 30 seconds, re-run step 4. If it still does not appear, the DHCP server is not answering it: it will have fallen back to a self-assigned 169.254.x.x address, which is not worth sweeping. Fix the DHCP server and power-cycle again.

## Rules when talking to the supply

- Read-only queries are fine. Unknown commands make the panel show "remote command error"; it is harmless, clear it with `:SYST:ERR?` until it returns `0,"No error"`.
- Do not write LAN settings over SCPI. The commands are accepted and silently ignored on this firmware.
- Before any output or setpoint change, read `components/pace-racer-board/halls-sensorless/bench/README.md` in the pace-racer-fw repo. In short: CH1 is the torque sensor's 24 V rail and its voltage must never be changed, and CH2's current limit must be re-asserted on every brake write.
