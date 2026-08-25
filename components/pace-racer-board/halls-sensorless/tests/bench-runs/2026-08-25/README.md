# Bench run — 2026-08-24/25

Hardware validation of PR #11 (`feat/ethernet-rtps-telemetry`) on real hardware, motor
idle and spinning under load.

**Start here:** [`report.html`](report.html) is the full write-up with figures — open it
in a browser. [`results.md`](results.md) has the same numbers as plain tables.
Also published as a Claude artifact:
<https://claude.ai/code/artifact/465e0680-f113-495a-a75d-9795989cf7e7>

Firmware under test: `v0.1.0-30-ga3da403`, built with ESP-IDF v6.0.0, identity confirmed
by boot banner **and** by string-matching the flashed binary.

## Status in one paragraph

The PR's central claim holds: network traffic does not disturb the control loop.
`coalesce` stayed at **1.00 in every condition tested** — every publish rate, every flood
level, idle and driving under load — and speed regulation is statistically unchanged by
saturating bidirectional traffic. Three things need attention before merge: delivery
**collapses silently above ~485 Hz**, the participant **fails to start at ≥5 writers**
against a documented claim of linear scaling to 8, and the host test tooling could not
receive telemetry at all until an espp bug was fixed.

## What was found

| | Finding |
|---|---|
| ✅ | **Coexistence confirmed.** `coalesce` 1.00 everywhere; `cmax` ≤ 77 µs. Speed regulation quiet → saturated, loaded: 0.832 → 0.815 rpm sd, smaller than the spread between two identical quiet runs. |
| ✅ | **Raw transport exceeds the PR's figures.** 1457 pps @ 64 B and 797 pps @ 1400 B, zero loss, vs 795/539 documented. |
| ✅ | **Delivery is exact and lossless to 300 Hz**, saturating at 485 Hz — above the PR's 303 Hz. |
| ❌ | **Delivery cliff.** Above ~485 Hz delivery collapses rather than degrading: at 2000 Hz the board publishes 734/s and **none arrive**, with `pub_fail = 0` throughout. The firmware cannot see the loss. |
| ❌ | **Writers cap at 4.** W=5, 6, 8 all fail to start (`up=0`). Per-writer cost is ~114 µs, not the documented 11.4 ms. |
| ❌ | **espp bug (upstream, not this PR).** RTPS multicast ignores `interface_address` when transmitting — see [`../../host_sub/`](../../host_sub/). |
| ⚠️ | **Telemetry stream starves under load.** 9.9 → 7.3 rows/s at saturation, ~25% of rows never emitted. This answers open question #1 in the PR description: yes, the priority-0 stream task starves. |
| ⚠️ | **Inbound is the weak direction.** RX saturates ~1247 pps @ 64 B and ~138 pps @ 1400 B, against 1457 pps outbound. Not previously characterised. |
| ⚠️ | **`rtps_sub.py` decoder is wrong** — assumes a trailing `state` byte; the wire puts it at offset 4, so it returns `flux` as `temps[0]`. Left unpatched because it cannot receive on this engine anyway. |

## Still open

1. The delivery cliff and the missing loss counter.
2. Participant start failure at ≥5 writers.
3. **Inbound RTPS processing cost** — still the PR's own biggest stated gap. `host_sub`
   is one half of what closing it needs; the other half is an external *writer* the board
   can subscribe to.
4. High-power operation. Everything here is ≤ 4.1 A and 100 rpm.

## Re-running this

```sh
# 1. firmware
cd ../..            # halls-sensorless/
idf.py build && idf.py -p /dev/cu.usbmodemXXXX flash

# 2. host subscriber (needed for any delivered-rate number)
cd tests/host_sub
patch -p1 -d ../../managed_components < espp-multicast-interface.patch
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j8

# 3. host link -- does NOT persist across replug
sudo ifconfig en7 inet 192.168.50.1 netmask 255.255.255.0

# 4. measurements
cd ../bench-runs/2026-08-25/scripts
python3 sweep_delivered.py                 # published vs delivered, finds the cliff
python3 writer_boundary.py                 # writer-count failure boundary
python3 rx_load_matrix.py                  # inbound flood vs control loop
python3 udp_tx_bench.py                    # raw UDP transmit ceiling
BENCH_LIB=/path/to/dyno/bench python3 spin_matrix.py    # SPINS THE MOTOR
BENCH_LIB=/path/to/dyno/bench python3 spin_loaded.py    # SPINS + BRAKE
```

Scripts resolve paths off their own location, so they run from anywhere. Two environment
overrides: `TELEM_SUB` (subscriber binary) and `BENCH_LIB` (the dyno `bench/` directory
holding `bench_lib.py`, which is **not** in this repo — only the spin and bring-up
scripts need it).

`prlib.py` reads `PR_PORT`, `PR_IFACE` and `PR_BOARD_IP` from the environment (defaults `/dev/cu.usbmodem2101`, `en7`, `192.168.50.50`). The board's USB port name changes between hosts and between replugs, so expect to set `PR_PORT`.

## Bench notes and hazards

- **Motor spin.** `spin_matrix.py` and `spin_loaded.py` energise the motor. Both stop it
  on every exit path including exceptions. This firmware has no `lim`/`vds`/`vl`/`odg`
  commands, so the documented arm profile cannot be applied — runs use boot defaults:
  8 A console clamp, 15 A trip, 80 °C.
- **The brake stalls far earlier than the dyno notes say.** The stall edge is between
  **1.0 V and 1.8 V**, not ~2.5 V. At 1.8 V the rotor stopped and the controller reverted
  from closed-loop sensorless to hall mode with zero current. Loaded captures used
  **1.2 V**, which draws the ~4 A the PR's condition calls for. Approach from below.
- **Hall table was uncalibrated** for this run (`hs` reports `cal=0`), which inflates
  absolute rpm standard deviation ~1.8× versus the PR's figures. Relative comparisons
  across conditions are unaffected — every capture shares the same table.
- **`late%` is not comparable across reboots.** Idle baseline measured 0.2%, 5.0% and
  0.4% in different sessions with the motor idle and the network quiet. It tracks I2C
  temperature reads and DRV fault polls preempting the sampling ISR
  (`main/foc_sampler.hpp:543`), and the phase relationship appears to be set at boot.
  Compare within a boot only.
- **Stale reader proxies** inflate repeated measurements — reboot between subscriber
  lifetimes. See [`../../host_sub/README.md`](../../host_sub/README.md).
- **Console wedge.** The USB-JTAG console can go mute such that even esptool cannot reach
  the ROM; a chip reset does not clear it, only USB re-enumeration does.

## Files

```
report.html                        full write-up with figures
results.md                         all numbers as plain tables
data/delivered_sweep.json          per-rate summary from sweep_delivered.py
data/delivered_sweep_samples.log   raw: one line per received sample
                                   "S <board_seconds> <host_epoch_seconds>"
scripts/                           everything used to produce the above
```
