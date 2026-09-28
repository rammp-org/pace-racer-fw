# tools/provision.py

Plug a PACE RACER board into the laptop and make it a robot wheel. One board
on USB at a time; the script finds it by MAC, so the port name does not matter.

```
tools/provision.py                # flash build-flatbot, axis + fb hall, reset, hcal if needed, report
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
2. **Flash.** Bootloader, partition table and app from `--build`
   (default `build-flatbot`). Build it first:
   `idf.py -B build-flatbot -DATOS_MOTOR=flatbot build`.
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
