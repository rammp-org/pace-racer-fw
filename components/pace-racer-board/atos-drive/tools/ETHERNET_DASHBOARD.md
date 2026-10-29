# Program a wheel over ethernet with the dashboard

How to flash and provision a PACE RACER wheel board from a browser when the
board is sealed inside a wheel and only ethernet reaches it. The page is
`tools/dashboard.html`; `tools/eth_bridge.py` serves it and relays it to the
board's service stream on tcp :5555.

All commands run from `components/pace-racer-board/atos-drive`. On Windows,
run the scripts through Python (`python tools\eth_bridge.py`) instead of
calling them directly.

## Before you start

- **macOS, Linux or Windows.** Developed on macOS; Linux and Windows support
  is new (2026-10-19) and not yet tried on a real machine, so report what
  breaks.
- **Python 3.** Standard library only, nothing to install.
- **ESP-IDF 6.0 and the submodules**, to build: after cloning, run
  `git submodule update --init --recursive` once (the build needs
  `external/rammp-rtps`).
- **The board already runs ethernet-capable firmware** (2026-10-15 or later).
  A board's first flash has to go over USB (`tools/provision.py`); the chair's
  two drive boards already qualify.
- **Something on the LAN hands out addresses.** The board takes its IP from
  DHCP only. There is no fallback, so with no DHCP server it never gets an
  address and the page cannot find it.

## 1. Build

If you built this app before 2026-10-29, delete the generated `sdkconfig`
first. It keeps old settings even after `sdkconfig.defaults` changes, and an
old 8 KB main-task stack makes the image crash or hang at boot, before
ethernet comes up. The build now stops with "main task stack below 16 KB"
if that happens.

```
rm -f sdkconfig
idf.py -B build-hub20 -DATOS_MOTOR=hub20 build
```

The image to flash is `build-hub20/pace_racer_atos_drive.bin`.

## 2. Get on the same network as the board

**On the chair:** plug the laptop into the chair's ethernet switch. The MIB is
the DHCP server; nothing else to do.

**On a bench, without a MIB:** plug the laptop and the board into the same
switch and run the stand-in DHCP server on the laptop's ethernet adapter. It
leases 169.254.50.10 and up, and the adapter needs its own address first
(the script says so and exits if it has none). `--list` shows the names:

```
python3 tools/mock_mib_dhcp.py --list             # adapters with an IPv4 address
```

| OS | adapter address | run it |
| --- | --- | --- |
| macOS | self-assigned 169.254 once the cable is in | `python3 tools/mock_mib_dhcp.py en7` |
| Linux | add one: `sudo ip addr add 169.254.50.1/16 dev eth0` (and tell NetworkManager to leave the adapter alone, or it removes it) | `sudo python3 tools/mock_mib_dhcp.py eth0` (port 67 needs root) |
| Windows | self-assigned 169.254 (APIPA) after about a minute without DHCP | `python tools\mock_mib_dhcp.py "Ethernet 2"`, and allow it through the firewall |

Leave it running. Power the board after the server is running, or wait for
its next DHCP retry.

## 3. Start the bridge and open the page

```
tools/eth_bridge.py                    # --port 9000 to use another port
```

Open http://localhost:8765/ in any browser. Served by the bridge, the page
starts in Ethernet mode.

## 4. Connect

1. Press **Find boards**. It asks the LAN who is there (`PACE?` on udp :5556)
   and fills in a board IP. With both wheels up it picks `drive_left` and
   says so in the event log; type the other IP to reach `drive_right`.
2. Press **Connect**. The page shows the axis, feedback source, hall
   calibration state and firmware live.

From a terminal the same check is `tools/eth_link.py list`.

## 5. Flash

1. Pick the image in the **build list** next to the Flash button. The bridge
   lists every `build-*/pace_racer_atos_drive.bin` it finds, newest first,
   with its version and build time, so there is no file dialog to fight.
   For an image from elsewhere, use the file picker or drop the `.bin` on
   the Firmware panel.
2. Press **Flash**. The page shows progress, waits for the board to reboot and
   come back, then marks the new image valid. A failure, including a browser
   that refuses to read a picked file, is printed on the status line and in
   the event log. Hover a greyed-out Flash button to see why it is off.

Restart `eth_bridge.py` after pulling a new version of it: the page is read
fresh on every load, but the build list comes from the running bridge.

Do one wheel at a time. Each wheel is its own board: repeat for the other IP.

The terminal equivalent is `tools/eth_link.py drive_left flash`, and
`tools/provision.py --host auto --motor hub20` runs the whole flow (flash,
axis, feedback, hall calibration, report).

## If a new image does not come back

That is the safety net, not a brick. A new image boots on probation and is
confirmed only after it holds a DHCP lease and serves ethernet for 20 s. If it
never gets there, the next power cycle boots the previous firmware. Fix the
network (step 2), power-cycle the board, and flash again.

## If Find boards finds nothing

Run `tools/eth_link.py list` in a terminal first. It does the same lookup as
the button and prints errors the page hides.

- **No DHCP server.** The most common cause: on a bench, is
  `mock_mib_dhcp.py` running on the right adapter?
- **Wrong adapter.** The laptop's ethernet adapter needs an IPv4 address on
  the board's subnet. `python3 tools/mock_mib_dhcp.py --list` shows the
  adapters the tools can see; virtual ones (Docker, WSL, Hyper-V, Tailscale)
  are skipped on purpose.
- **Firewall.** Discovery broadcasts to udp :5556 and the board answers by
  unicast to the laptop's sending port, which a firewall does not see as a
  reply. Windows Firewall and some Linux setups (ufw, firewalld) drop it:
  allow Python to receive inbound UDP. Typing the board's IP skips discovery.
- **Tailscale.** On macOS it claims the 169.254 range and silently drops
  the traffic. The tools pin their sockets to the physical adapter to get
  around it, but if in doubt, turn Tailscale off.
- **"bridge refused"** in the page: `eth_bridge.py` is not running, or the
  board IP is wrong.
- **Old firmware.** A board flashed before 2026-10-15 does not answer on
  ethernet at all; it needs one USB flash first.

## Status

Ethernet flashing is how the current chair was programmed in OCT 2026
