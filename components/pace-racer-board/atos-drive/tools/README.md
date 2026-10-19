# tools/provision.py

Make a PACE RACER board a robot wheel, over USB (bench, or a board's first
flash) or over ethernet (a board already inside a wheel). One board at a time;
the script finds it by MAC, so the port name or IP does not matter.

```
tools/provision.py --motor hub20  # USB: flash build-hub20, axis + fb hall, reset, hcal if needed, report
tools/provision.py --host auto --motor hub20   # the same over ethernet (OTA instead of esptool)
tools/provision.py --host list    # boards answering on the LAN, with roster lookup by MAC
tools/provision.py --host drive_left --hcal    # by axis name, number or IP
tools/provision.py --list         # which boards are plugged in, and what the roster says they are
tools/provision.py --axis 3       # override the roster for this board
tools/provision.py --hcal         # redo the hall table even if one is saved
tools/provision.py --no-flash     # keep the firmware on the board; NVS and calibration only
tools/provision.py --port /dev/cu.usbmodem2101   # several boards plugged in
tools/provision.py --build build-hub20           # another profile's build dir
```

## What it does

1. **Identify.** Every `/dev/cu.usbmodem*` is asked for its MAC with esptool.
   `ROSTER` at the top of the script maps MAC to wheel; `BLOCKLIST` names
   boards that are never touched (other projects sharing the laptop). Anything
   not in the roster needs `--axis`.
2. **Flash.** USB: bootloader, partition table and app from `--build`
   (default `build-<motor>`, `--motor flatbot` unless told otherwise). Build
   it first: `idf.py -B build-hub20 -DATOS_MOTOR=hub20 build`. Ethernet:
   OTA of the app only through `eth_link.py`, then wait for the board to
   come back on the same IP and mark the image valid.
3. **Provision.** `axis <n>` and `fb hall` over the console, then a reset so
   both take effect.
4. **Hall calibration**, only if the axis line says the table is not saved, or
   with `--hcal`. It waits for Enter: wheel off the ground and free, motor
   supply on. Then `ho 9999 45 1`, `hcal 6`, and a 30 rpm I/f spin each way
   for about 15 s. It checks the wheel actually turned (supply off or a
   blocked wheel stops the run with a message) and that the board reported
   `cal=1` with both direction budgets filled.
5. **Report.** The `axis` and `fb` lines and the boot check. Exit 0 means the
   board arms.

## Adding a board

Read its MAC with `--list` and add a line to `ROSTER`:

```python
ROSTER = {
    "10:bd:a3:66:15:94": 2,  # drive_left
    "10:bd:a3:64:ee:fc": 3,  # drive_right
}
```

## Needs

pyserial and esptool. The system python is tried first, then the ESP-IDF
venv under `~/.espressif/python_env/`. No `export.sh` needed.

## If it stops

- `no answer` next to a port: not an ESP32-S3 in download mode; unplug and
  replug, or it is not a PACE board.
- `'run' refused (sampler off?)`: the FOC sampler is disabled after a trip.
  Power-cycle the board with the motor supply already on.
- `wheel did not turn`: supply off, wheel on the ground, or hall wiring.
  `hs` on the console shows the raw hall state.
- `hall calibration did not finish`: it needs 6 s of *rotation* per
  direction; a wheel that stalls part way does not count. Run again.

# tools/eth_link.py

The ethernet twin of the WebUSB tools: discovery (`PACE?` broadcast on udp
:5556), OTA, console, reset, rollback over the service stream on tcp :5555
(`main/eth_link.hpp`). `tools/eth_link.py --help` lists the actions;
`provision.py --host` imports it. Standard library only, plus the `espp_ota`
package already in `managed_components`.

A board flashed with firmware older than 2026-10-15 does not serve the
stream: its first flash with this build has to go over USB.

# tools/dashboard.html and tools/eth_bridge.py

One page for the whole provisioning job, so nobody has to remember the
console commands: connect, read axis / feedback / hall state live, save the
axis and feedback source, run the hall calibration with each step reported,
flash an app image with progress and automatic mark-valid after the reboot,
roll back, reset, and a free-form console. Styled per alex-style-guide.

Two ways in:

```
open tools/dashboard.html          # WebUSB: board on the laptop, Chrome, pick "PACE RACER" in the dialog
tools/eth_bridge.py                # Ethernet: serves the page at http://localhost:8765/ and relays
                                   # its WebSocket to the board's tcp :5555; "Find boards" lists the LAN
```

The page defaults to Ethernet when it is served by the bridge and to WebUSB
when opened as a file. Both transports carry the same frames, so every
button works on either. The bridge is standard library only.

Step-by-step setup for a wheel on the chair or a bench switch, network
included: [ETHERNET_DASHBOARD.md](ETHERNET_DASHBOARD.md).
