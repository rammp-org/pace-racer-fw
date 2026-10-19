# Program a wheel over ethernet with the dashboard

How to flash and provision a PACE RACER wheel board from a browser when the
board is sealed inside a wheel and only ethernet reaches it. The page is
`tools/dashboard.html`; `tools/eth_bridge.py` serves it and relays it to the
board's service stream on tcp :5555.

All commands run from `components/pace-racer-board/atos-drive`.

## Before you start

- **A laptop running macOS.** The tools read `ifconfig` / `ipconfig` output
  in the macOS format; on Linux or Windows discovery and the bridge do not
  work yet.
- **Python 3.** Standard library only, nothing to install.
- **The board already runs ethernet-capable firmware** (2026-10-15 or later).
  A board's first flash has to go over USB (`tools/provision.py`); the chair's
  two drive boards already qualify.
- **Something on the LAN hands out addresses.** The board takes its IP from
  DHCP only. There is no fallback, so with no DHCP server it never gets an
  address and the page cannot find it.

## 1. Build

```
idf.py -B build-hub20 -DATOS_MOTOR=hub20 build
```

The image to flash is `build-hub20/pace_racer_atos_drive.bin`.

## 2. Get on the same network as the board

**On the chair:** plug the laptop into the chair's ethernet switch. The MIB is
the DHCP server; nothing else to do.

**On a bench, without a MIB:** plug the laptop and the board into the same
switch and run the stand-in DHCP server on the laptop's ethernet adapter:

```
networksetup -listallhardwareports     # find the adapter, e.g. en7
python3 tools/mock_mib_dhcp.py en7     # leave it running; no root needed
```

It leases 169.254.50.10 and up. Wait for the adapter to have its own
169.254 address first (it says so and exits if it has none). Power the
board after the server is running, or wait for its next DHCP retry.

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

1. Choose `build-hub20/pace_racer_atos_drive.bin` in the firmware file picker.
2. Flash. The page shows progress, waits for the board to reboot and come
   back, then marks the new image valid.

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

- **No DHCP server.** The most common cause: on a bench, is
  `mock_mib_dhcp.py` running on the right adapter?
- **Wrong adapter.** The laptop's ethernet adapter needs an IPv4 address on
  the board's subnet (`ifconfig en7`).
- **Tailscale.** It claims the 169.254 range and silently drops the traffic.
  The tools bind to the physical adapter to get around it, but if in doubt,
  turn Tailscale off.
- **"bridge refused"** in the page: `eth_bridge.py` is not running, or the
  board IP is wrong.
- **Old firmware.** A board flashed before 2026-10-15 does not answer on
  ethernet at all; it needs one USB flash first.

## Status

Ethernet flashing was tested against a mock on 2026-10-15. It has not yet
been confirmed end to end on a wheel board.
