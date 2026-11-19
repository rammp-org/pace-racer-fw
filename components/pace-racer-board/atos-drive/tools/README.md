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
tools/provision.py --port /dev/cu.usbmodem2101   # several boards plugged in (Linux /dev/ttyACM0, Windows COM5)
tools/provision.py --build build-hub20           # another profile's build dir
tools/provision.py --host drive_left --hcal --mib 192.168.4.1   # the MIB to ask for a calibration hold (this is the default)
```

## What it does

1. **Identify.** Every serial port with a PACE (1209:0d41, 1209:0d32) or
   Espressif (303a) USB id is asked for its MAC with esptool.
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

   **In the chair** the MIB commands every wheel all the time, and the board
   refuses console motion while that stream is live. So before the spin the
   script reads the board's `atos` line, and if `cmd_age` is under 200 ms it
   asks the MIB (`--mib`, default 192.168.4.1) for a *calibration hold* on
   this wheel: the MIB stops commanding it, the stream goes stale, and the
   console may drive. The hold is renewed every 20 s and released when the
   calibration ends, fails, or is interrupted with Ctrl-C (the wheel gets
   `stop` first). Released, the MIB commands the wheel DISARMED again. The MIB
   grants it only when it is IDLE with the e-stop clear, so disable the drive
   first. With no stream (a bench) no MIB is contacted.
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
venv under `~/.espressif/python_env/`. No `export.sh` needed. The partition
check uses `$IDF_PATH`, else `~/esp/esp-idf`.

## Linux and Windows

Developed on macOS; the tools also run on Linux and Windows (new
2026-10-19, not yet tried on either, so report what breaks).

- **Windows:** run every script through Python, e.g.
  `python tools\provision.py --list`. WebUSB needs no driver: the board
  asks Windows for WinUSB itself. `usb_reset.py` (pyusb) also needs
  `libusb-1.0.dll` on the PATH, or `pip install libusb-package`.
- **Linux:** install the udev rule so Chrome, pyusb and the serial port
  work without sudo, then replug the board:

  ```
  sudo cp tools/99-pace-racer.rules /etc/udev/rules.d/
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```

  pyusb needs `libusb-1.0-0` (apt). Over ssh, join the `dialout` group for
  the serial port.
- **Ethernet on either:** see [ETHERNET_DASHBOARD.md](ETHERNET_DASHBOARD.md)
  for the bench DHCP server per OS and the firewall note.

## If it stops

- `no answer` next to a port: not an ESP32-S3 in download mode; unplug and
  replug, or it is not a PACE board.
- `'run' refused (sampler off?)`: the FOC sampler is disabled after a trip.
  Power-cycle the board with the motor supply already on.
- `wheel did not turn`: supply off, wheel on the ground, or hall wiring.
  `hs` on the console shows the raw hall state.
- `hall calibration did not finish`: it needs 6 s of *rotation* per
  direction; a wheel that stalls part way does not count. Run again.
- `the MIB does not know request 0x0a`: the MIB firmware predates the
  calibration hold. Update it, or unplug the MIB's ethernet cable for the
  calibration (the wheel's stream goes stale) and plug it back in after.
- `the MIB refused the calibration hold`: the reason follows. Usually the
  drive is enabled or the e-stop is latched; the MIB must be IDLE.
- `no MIB answered at ...`: a stream is live but nothing serves :5555 at
  that address. Pass `--mib <ip>`.
- `the wheel's stream is still live`: the MIB held the axis but something
  else still commands the board (a second MIB, a stand-in on the laptop),
  or the board's axis is not the one held. The hold is released.
- `! refused: a MotorCommand stream is live` from the board itself (the
  dashboard, or a console by hand): take a hold first,
  `tools/eth_link.py mib hold drive_left`, and `tools/eth_link.py mib
  release` when done. A hold left behind lapses after 60 s.

# tools/eth_link.py

The ethernet twin of the WebUSB tools: discovery (`PACE?` broadcast on udp
:5556), OTA, console, reset, rollback over the service stream on tcp :5555
(`main/eth_link.hpp`). `tools/eth_link.py --help` lists the actions;
`provision.py --host` imports it. Standard library only, plus the `espp_ota`
package already in `managed_components`.

It also speaks to the MIB's calibration hold (RAMMP-mib-fw control module
0x40 on the MIB's own tcp :5555): `tools/eth_link.py mib hold drive_left
[--secs 60]`, `mib release [axis]` and `mib hold-status`; `mib` is
192.168.4.1, or give the MIB's IP. A hold lasts 60 s unless renewed (ask
again) and never more than 300 s in total. The MIB serves one TCP client and
a newcomer replaces it, so a control panel connected to the MIB over
ethernet is dropped by each request and reconnects.

A board flashed with firmware older than 2026-10-15 does not serve the
stream: its first flash with this build has to go over USB.

# tools/dashboard.html and tools/eth_bridge.py

One page for the whole provisioning job, so nobody has to remember the
console commands: connect, read axis / feedback / hall state live, save the
axis and feedback source, run the hall calibration with each step reported,
flash an app image with progress and automatic mark-valid after the reboot,
roll back, reset, and a free-form console. Served by the bridge, the Firmware
panel lists the app images in `build-*/` and flashes one without a file dialog. Styled per alex-style-guide.

Two ways in:

```
open tools/dashboard.html          # WebUSB: board on the laptop, Chrome or Edge (any OS), pick "PACE RACER"
tools/eth_bridge.py                # Ethernet: serves the page at http://localhost:8765/ and relays
                                   # its WebSocket to the board's tcp :5555; "Find boards" lists the LAN
```

The page defaults to Ethernet when it is served by the bridge and to WebUSB
when opened as a file. Both transports carry the same frames, so every
button works on either. The bridge is standard library only.

Step-by-step setup for a wheel on the chair or a bench switch, network
included: [ETHERNET_DASHBOARD.md](ETHERNET_DASHBOARD.md).
