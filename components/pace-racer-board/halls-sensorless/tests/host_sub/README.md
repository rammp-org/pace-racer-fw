# `telem_sub` — host-side DDS subscriber

A real RTPS participant for the `pace_racer/telemetry` topic, built natively from
the espp sources that the firmware itself uses.

## Why this exists

`../rtps_sub.py` cannot receive anything from the v1.2.0 engine. It is a passive
multicast sniffer: it joins the user multicast group and waits. The old engine had a
`use_multicast_for_user_data` mode that sprayed user data at that group with no reader
required, and the script was written against it. That mode is gone.

The current engine only transmits to *matched* readers. With none matched,
`StatelessWriter::progress()` iterates an empty proxy list and no packet leaves the
board — while `pub_ok` still counts up, because `publish()` itself succeeded. That is
correct RTPS behaviour, not a regression, but it means every delivered-rate number the
Python suite reports is `0.0 Hz`.

This program announces itself properly (SPDP), declares a reader (SEDP), gets matched,
and receives real traffic. It reuses the `Sample` struct from `main/rtps_telem.hpp`
verbatim, so espp's own reflection handles the wire layout on both ends and there is no
hand-written decoder to drift out of sync.

## Build

espp is not ESP32-only — the same sources compile for the host. No ESP-IDF needed.

```sh
patch -p1 -d ../../managed_components < espp-multicast-interface.patch   # see below
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j8
```

The CMakeLists points at `../../managed_components`, so **the app must have been built
once** (`idf.py build` in `halls-sensorless/`) to populate that directory.

## The patch is mandatory on a multi-homed host

`EsppTransport` never passes `multicast_interface` to the socket layer, so RTPS
multicast follows the OS default route regardless of `interface_address`. On a laptop
that is usually Wi-Fi, so the subscriber's SPDP announcements never reach a board on a
second NIC. Discovery then fails in one direction only — you receive the board's
announcements, it never sees yours, nothing matches, and you get silence with no error.

`managed_components/` is gitignored and re-fetched by the component manager, so
**re-apply the patch after any component update.** It affects the host build only; the
firmware is unchanged.

## Run

```sh
./build/telem_sub <interface-ip> <seconds> <mode>
```

- `<interface-ip>` — the host NIC facing the board, e.g. `192.168.50.1`
- `<mode>` — `N` prints the first N samples decoded; `-1` prints one machine-readable
  `S <board_seconds> <host_epoch_seconds>` line per sample, which is what
  `../bench-runs/*/scripts/sweep_delivered.py` parses

The board needs `eth` then `rtps` on its console first.

## Gotcha: stale reader proxies

The topic is BEST_EFFORT with no liveliness timeout, so the board keeps the
`ReaderProxy` of every subscriber that has ever matched — including dead ones — and goes
on transmitting to them. A second run therefore receives duplicates of every sample.
This produced an impossible "18.3 Hz delivered from a 10 Hz request" before it was
spotted.

**Reboot the board between subscriber lifetimes**, or keep one subscriber alive for the
whole measurement (which is what `sweep_delivered.py` does).
