# PACE RACER ethernet / RTPS test scripts

Host-side tests for the W5500 ethernet and RTPS telemetry work. They drive the
`halls-sensorless` firmware over the USB console and measure the ethernet link
from the other end.

Findings from these tests are written up in
[`../../ETHERNET_FOC_COEXISTENCE.md`](../../ETHERNET_FOC_COEXISTENCE.md).

## Setup

```sh
pip install pyserial

# Static address on the dock interface. Address ONLY - do not configure a
# gateway here: en7 sits above Wi-Fi in macOS's service order, so giving it a
# router installs a higher-priority default route and cuts the host's internet.
sudo ifconfig en7 inet 192.168.50.1 netmask 255.255.255.0
```

This does **not** persist. macOS drops it whenever it reconfigures the
interface, after which the port falls back to a self-assigned `169.254.x.x` and
the scripts will tell you to re-apply it.

Board is `192.168.50.50`. Ports and interface names live at the top of
`prlib.py`.

## Scripts

| Script | Purpose |
| --- | --- |
| `prlib.py` | shared helpers — console I/O, stat parsing, interface address |
| `spdp_sniff.py` | verify RTPS discovery announcements arrive. **Run this first when RTPS looks broken** |
| `rtps_sub.py` | receive and decode RTPS telemetry; reports delivered rate |
| `sweep_rtps.py` | sweep publish rate vs FOC disturbance; finds the bandwidth margin |
| `sweep_endpoints.py` | scale writers/readers to measure per-endpoint cost |
| `udp_load.py` | flood the board with UDP (raw-transport RX load) |
| `udp_recv.py` | drain UDP from the board (raw-transport TX) |
| `usb_bench.py` | benchmark the USB console for comparison |
| `foc_quality.py` | rpm/current/observer stability from the telemetry stream |

## Typical runs

```sh
cd tests

# Is RTPS alive at all?
python3 spdp_sniff.py 8

# Is telemetry flowing, and are the values right?
python3 rtps_sub.py 10 --decode

# Bandwidth margin, motor idle
python3 sweep_rtps.py

# Same, with the motor spinning (SEE SAFETY BELOW)
python3 sweep_rtps.py --spin --amps 4 --rpm 100

# How does cost scale with endpoints? (reflashes between points)
. $IDF_PATH/export.sh
python3 sweep_endpoints.py --writers 1,2,4,8
python3 sweep_endpoints.py --readers 0,1,2,4

# Transport comparison
python3 usb_bench.py 5 64
```

## Firmware console commands these drive

| Command | Effect |
| --- | --- |
| `eth` | bring up the W5500 (not done at boot, so a run before it is a clean baseline) |
| `rtps [writers] [readers]` | start the RTPS participant. Endpoint counts are fixed at start — changing them needs a reboot |
| `rtpshz <hz>` | publish rate; `0` stops publishing but leaves the participant up |
| `rtpstat` | publish counters, per-publish cost, reader receive counts — resets on read |
| `s` | sampler + FOC loop statistics — resets on read |
| `etx <hz> <bytes>` / `etx max <secs> <bytes>` | raw UDP transmit, paced or flat out |
| `nstat` | raw UDP counters |
| `usbtx <secs> <bytes>` | USB console throughput benchmark |
| `run <A> <rpm> <ramp>` / `stop` | spin / de-energise the motor |

## Reading the numbers

- **`late%`** — the sampling ISR fired late or overran, so the current sample
  landed outside the null window and is corrupted. It is a *signal-quality*
  metric, **not** "the control loop fell behind" — that is `coalesce`.
- **`coalesce`** — notifications ÷ task runs. 1.00 means the FOC task never
  missed a tick.
- **`pub_us`** — per-cycle publish cost. The publish rate saturates at
  `1/pub_us`, so this sets the ceiling.
- **`overrun`** — cycles where the deadline had already passed; the publisher
  could not keep up with the requested rate.

## Safety

`sweep_rtps.py --spin` energises the motor. Coupled to the mag brake it needs
**~4 A** to break away — at 2 A the field spins at the commanded rate while the
rotor slips at ~13 rpm. The script always sends `stop` on exit, including on
Ctrl-C, but keep the bench clear and stay ready to hit stop yourself.

The firmware's own guards stay active throughout: 15 A trip, 80 °C thermal
limit.
