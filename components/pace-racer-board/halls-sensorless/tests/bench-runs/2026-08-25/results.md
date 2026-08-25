# Results — bench run 2026-08-24/25

Firmware `v0.1.0-30-ga3da403`, ESP-IDF v6.0.0. ESP32-S3 at 48 V, W5500 on the SPI
expansion header, point-to-point to a Mac at 192.168.50.1. Figures in
[`report.html`](report.html).

## 1. Delivered rate — the headline

One subscriber for the whole sweep, fresh boot, motor idle, 1 writer / 0 readers.
`published` is the board's `pub_ok` counter; `delivered` is counted by `telem_sub`. Both
cover an identical window (counter reset and read are bracketed with the same command
latency offset, so it cancels instead of appearing as loss).

| Requested (Hz) | Published (/s) | Delivered (/s) | Loss (%) | pub_us mean (µs) | overrun | late% | coalesce |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 10 | 10.0 | 10.0 | 0.0 | 569 | 0 | 0.4 | 1.00 |
| 25 | 25.0 | 25.0 | 0.0 | 495 | 0 | 0.9 | 1.00 |
| 50 | 50.0 | 50.0 | 0.0 | 451 | 0 | 1.5 | 1.00 |
| 100 | 99.6 | 100.0 | 0.0 | 429 | 0 | 2.4 | 1.00 |
| 200 | 200.4 | 200.0 | 0.2 | 408 | 0 | 4.1 | 1.00 |
| 300 | 300.5 | 299.8 | 0.2 | 412 | 15 | 5.7 | 1.00 |
| 500 | 486.4 | **484.6** | 0.4 | 377 | 1094 | 7.5 | 1.00 |
| 1000 | 534.6 | **25.3** | **95.3** | 846 | 4181 | 8.9 | 1.00 |
| 2000 | 733.7 | **0.0** | **100.0** | 815 | 7402 | 9.5 | 1.00 |

Peak delivered **484.6 Hz** (PR documents 303 Hz). `pub_fail = 0` at every rate,
including where nothing is delivered.

Raw data: `data/delivered_sweep.json`, `data/delivered_sweep_samples.log`.

## 2. Control quality while spinning — 100 rpm, 15 s captures

| Condition | Load | rpm mean | rpm sd | iq mean (A) | iq sd (A) | mean \|aerr\| | obs err sd | late% | coalesce | rows/s |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Quiet, no network | free | 100.04 | 0.787 | 1.618 | 0.0921 | 0.07 | 0.976 | 0.4 | 1.00 | 9.9 |
| RTPS publishing 10 Hz | free | 100.05 | 0.743 | 1.601 | 0.0880 | 0.01 | 0.947 | 0.4 | 1.00 | 9.7 |
| RTPS + inbound flood | free | 100.05 | 0.775 | 1.586 | 0.0934 | 0.02 | 1.061 | 2.8 | 1.00 | 8.4 |
| Bidirectional saturation | free | 100.05 | 0.807 | 1.549 | 0.1034 | 0.05 | 1.121 | 4.2 | 1.00 | 7.3 |
| Quiet again (drift control) | free | 99.99 | 0.805 | 1.553 | 0.0868 | 0.09 | 1.043 | 0.4 | 1.00 | 9.9 |
| **Quiet, loaded** | brake | 100.09 | 0.832 | 4.104 | 0.1189 | 0.17 | 1.100 | 0.4 | 1.00 | 9.8 |
| **Bidirectional, loaded** | brake | 100.05 | 0.815 | 3.925 | 0.1109 | 0.05 | 1.099 | 3.9 | 1.00 | 7.4 |

Speed regulation is unchanged by traffic: loaded quiet → saturated is 0.832 → 0.815 rpm
sd, *smaller* than the spread between the two identical free-shaft quiet runs
(0.787 vs 0.805). Mean speed 99.99–100.09 rpm throughout.

`rows/s` falling 9.9 → 7.3 is the priority-0 CSV stream task starving — ~25% of
telemetry rows never emitted at saturation.

PR comparison: it reports rpm sd 0.43–0.48 and iq sd 0.088–0.102. iq sd reproduces
closely; absolute rpm sd here is ~1.8× higher because the hall table was uncalibrated
(`cal=0`) and `rpm_hall` is the averaged quantity.

## 3. Endpoint scaling — 50 Hz requested, no subscriber

| Writers | Readers | pub_ok | pub_us mean (µs) | pub_us max (µs) | late% | Participant |
|---:|---:|---:|---:|---:|---:|---|
| 1 | 0 | 660 | 251 | 1506 | 0.3 | started |
| 2 | 0 | 1318 | 302 | 1717 | 0.4 | started |
| 4 | 0 | 2644 | 592 | 2112 | 0.3 | started |
| 5 | 0 | 0 | — | — | — | **failed to start** |
| 6 | 0 | 0 | — | — | — | **failed to start** |
| 8 | 0 | 0 | — | — | — | **failed to start** |

~114 µs per additional writer. PR claims ~11.4 ms each and linear scaling to 8.

## 4. Inbound flood vs control loop — motor idle

| Condition | Offered (pkts) | Received (pkts) | Delivered (%) | late% | coalesce | cmax (µs) | ISR max (µs) |
|---|---:|---:|---:|---:|---:|---:|---:|
| Quiet baseline | — | — | — | 0.2 | 1.00 | 37 | 17.3 |
| 500 pps × 64 B | 3000 | 3002 | 100 | 0.6 | 1.00 | 35 | 16.8 |
| 2000 pps × 64 B | 12000 | 5956 | 49.6 | 0.7 | 1.00 | 35 | 21.6 |
| 5000 pps × 64 B | 30000 | 7481 | 24.9 | 0.8 | 1.00 | 36 | 20.5 |
| 2000 pps × 1400 B | 12000 | 825 | 6.9 | 0.4 | 1.00 | 35 | 15.5 |

RX saturates ~1247 pps at 64 B and ~138 pps at 1400 B, against 1457 pps outbound.
Disturbance tracks packet *rate*, not bandwidth — hence the 1400 B row looking better.
(The first row slightly exceeds its offered count because the measurement's own probe
datagram is included.)

## 5. Transport throughput

| Path | Firmware-side | Host-side | Errors | PR documented |
|---|---|---|---|---|
| UDP transmit, 64 B | 1457 pps · 91.1 kB/s | 8742 / 8742 pkts | 0 | 795 pps |
| UDP transmit, 1400 B | 797 pps · 1090.3 kB/s | 4785 / 4785 pkts | 0 | 539 pps |
| USB console, 64 B lines | 3745 lines/s · 234.0 kB/s | 237.2 kB/s | 0 | — |
| ICMP ping | — | 2.05 / 2.74 / 3.99 ms | 0% loss | 1.3 ms mean |

## 6. SPDP discovery — documented vs measured

| Field | PR description | Measured |
|---|---|---|
| Source port | 7410 | 7400 |
| RTPS version | 2.3 | 2.2 |
| Vendor ID | cafe | 0d25 |
| Announcement size | 240 B | 204 B |
| GUID prefix | `00000000293d…` | `8cfd49a2deb3…` (MAC-derived) |

Discovery works; the PR's screenshot predates the v1.2.0 engine.

## 7. Wire layout

`rtps_sub.py` assumes 15 contiguous floats then a trailing `uint8 state`. Measured on the
pre-port engine, the wire is:

```
off  0  float seconds
off  4  uint8 state (+3 pad)      <-- not trailing
off  8  float id, iq, iqref, vd, vq, aerr, rpm_drive, rpm_est, rpm_hall, flux
off 48  float temps[4]
        64 B total
```

Reading it as declared silently returns `flux` as `temps[0]` and a padding byte as
`state`. Not re-verified on v1.2.0 — `telem_sub` uses the `Sample` struct directly and
lets espp's reflection handle it, which sidesteps the question.

## Notes on measurement validity

- An earlier pass ran against **the wrong firmware** (the board arrived carrying an older
  build; the reflash inside `sweep_endpoints.py` installed the PR build partway through).
  Caught because `rtpstat` printed `parts=`/`eps=`, strings absent from the PR binary.
  Those results were discarded.
- An earlier "publishing got 9× cheaper" claim was measured **with no matched reader**, so
  `publish()` never transmitted. With a subscriber attached the improvement is ~6.7×
  (3839 → 569 µs at 10 Hz).
- `late%` absolute values are not comparable across reboots. See the README.
