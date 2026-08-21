---
title: Ethernet / FOC Coexistence Testing
date: 2026-08-03
branch: feat/eth-foc-coexist
hardware: PACE RACER (ESP32-S3) + WIZnet W5500 + DRV8353
status: complete
tags:
  - pace-racer
  - ethernet
  - w5500
  - foc
  - motor-control
  - esp32
  - rtps
---

# Ethernet / FOC Coexistence Testing

> [!abstract] One-line answer
> **Ethernet and FOC coexist fine.** Speed regulation is unaffected by network
> traffic at any load tested. What traffic costs is current-sample quality and
> telemetry throughput.

> [!danger] REVISED 2026-08-04 — the original transport ceilings were CPU-bound, not hardware
> This note originally concluded that the transmit ceiling was a flat ~250
> packets/s at every payload size, and that this was "a property of the W5500
> path, not of system load." **That was wrong.** It was an instruction-cache
> limit.
>
> Raising the S3 instruction cache from 16 KB to 32 KB and moving the FOC task
> into IRAM lifted the raw UDP transmit ceiling by 2.2–3.0x and the RTPS
> telemetry ceiling by 3.6x, with no change to the W5500 or its SPI clock:
>
> | payload | original | after cache+IRAM | gain |
> | --- | --- | --- | --- |
> | 64 B | 262 pps / 16.4 kB/s | **795 pps / 49.7 kB/s** | 3.0x |
> | 512 B | 254 pps / 127.0 kB/s | **683 pps / 341.3 kB/s** | 2.7x |
> | 1400 B | 240 pps / 328.1 kB/s | **539 pps / 736.7 kB/s** | 2.2x |
>
> Note the shape change: packet rate is no longer flat across payload size
> (795 -> 539), which is what you expect once the wire rather than the CPU is the
> constraint. **The "budget packets, not bytes" guidance in
> [[#6 Packet rate is the constraint not bandwidth]] was therefore derived from a
> CPU-bound regime and is too conservative.** Batching still helps bandwidth
> substantially (737 vs 50 kB/s), but the packet ceiling is 2-3x higher than
> originally reported.
>
> The sections below are preserved as originally measured; treat their absolute
> ceilings as a floor, and see [[#14 Cache and IRAM revision]] for what changed.

Test session 2026-08-03. All firmware on branch `feat/eth-foc-coexist`.
Motor: 15 pole-pair BLDC, 48 V bus, coupled to a magnetic brake for the final runs.

---

## Contents

- [[#1 What was tested]]
- [[#2 W5500 bring-up]]
- [[#3 The measurement instrument]]
- [[#4 Does ethernet disturb the control loop]]
- [[#5 Control quality under load]]
- [[#6 Packet rate is the constraint not bandwidth]]
- [[#7 Transmit ceiling ethernet vs USB]]
- [[#8 The CPU0 finding and the lwIP A B C]]
- [[#9 The encoder thread]]
- [[#10 Network setup traps]]
- [[#11 Conclusions and design guidance]]
- [[#12 What is NOT established]]
- [[#13 Errors made during testing]]
- [[#14 Cache and IRAM revision]]

---

## 1 What was tested

Three questions, in order:

1. Can the board talk over the W5500 at all?
2. Does carrying network traffic disturb the 20 kHz FOC control loop?
3. Can ethernet carry telemetry, and how does it compare to the existing USB console?

The motivating end goal is running RTPS/DDS alongside FOC.

### Test harness

Added to the `halls-sensorless` app so one binary yields every measurement point
without reflashing — **reflashing between runs would perturb the comparison as
much as the variable under test**.

| Command | Effect |
| --- | --- |
| `eth` | bring up W5500 + UDP sink (deliberately NOT run at boot, so a run before it is a true no-ethernet baseline) |
| `etx <hz> <bytes>` | publish UDP at a fixed rate — a stand-in DDS writer |
| `etx max <secs> <bytes>` | bounded flat-out burst, self-reports achieved rate |
| `nstat` | link/IP + rx/tx counters, then reset |
| `usbtx <secs> <bytes>` | flood the USB console to measure that transport |
| `s` | pre-existing: sampler + FOC loop statistics |

Host-side scripts (`udpload.py`, `udprecv.py`, `usbbench.py`, `quality.py`) live
in the session scratchpad, not the repo.

---

## 2 W5500 bring-up

**Result: PASSED.**

| Check | Result |
| --- | --- |
| Driver install on shared SPI2 | OK, SPI host 1 |
| MAC (derived from eFuse `ESP_MAC_ETH`) | `8C:FD:49:A2:DD:C7` |
| Link up | 2.2 s after boot, 100baseTX full-duplex |
| ICMP | 4/4, **1.3 ms avg, 0% loss** (min/avg/max 1.304/1.782/3.053 ms) |
| TCP connect | 1.6 ms |
| TCP echo 16 / 25 / 200 B | byte-exact, 55.3 / 3.4 / 4.2 ms (first includes ARP + slow-start) |
| Board-side logging | confirmed connect / echo / disconnect |

### Pin map

The W5500 shares SPI2 with the DRV8353 but has its own chip select, so there is
no pin conflict:

| Signal | GPIO | Bus |
| --- | --- | --- |
| SPI clock | 12 | SPI2 (shared) |
| SPI MOSI | 11 | SPI2 (shared) |
| SPI MISO | 13 | SPI2 (shared) |
| W5500 CS | 10 | SPI2 |
| W5500 reset | 21 | — |
| W5500 IRQ | 14 | — |
| DRV8353 CS | 39 | SPI2 |
| Encoder SCLK / MISO / CS | 37 / 35 / 38 | **SPI3 (separate)** |

> [!note] The W5500 has no factory-burned MAC
> One must always be assigned. The BSP derives it from the ESP32-S3 eFuse.

---

## 3 The measurement instrument

No new instrumentation was needed — the `s` command already reported the right
things. Understanding what they mean is essential to reading everything below.

| Metric | Meaning |
| --- | --- |
| `late` % | **The sampling ISR fired late or overran its budget, so the current sample landed OUTSIDE the null window** — taken mid-switching, therefore corrupted. Per `foc_sampler.hpp`. |
| `coalesce` | notifications ÷ task_runs. 1.00 = the FOC task never fell behind. >1.00 = it missed ticks. |
| `cmax` | worst-case FOC compute time, µs. |
| `isr` | sampler ISR min/avg/max duration, µs. |
| `imp` / `rej` | glitch-filter counters — implausible samples held, in-band slew rejections. |

> [!important] `late` is a signal-quality metric, not a "loop fell behind" metric
> This distinction drives every interpretation in this note. A rising `late%`
> means more corrupted current samples that the glitch filter then discards. It
> does **not** mean the control loop missed its deadline — that is `coalesce`.

---

## 4 Does ethernet disturb the control loop

### 4.1 Motor idle

| Phase | late % | coalesce |
| --- | --- | --- |
| **A** no ethernet | 4.8 – 5.0 | 1.00 |
| **B** eth up, link up, zero traffic | 4.8 – 5.0 | 1.00 |
| **C** rx 4 Mbit/s | 9.2 | 1.00 |
| **C** rx 24 Mbit/s | 9.1 | 1.00 |
| **C** bidirectional | 11.7 | 1.00 |

### 4.2 Motor spinning, free shaft (2 A, 100 rpm, closed-loop sensorless)

| Phase | late % | coalesce |
| --- | --- | --- |
| **A′** no ethernet | 3.2 – 3.3 | 1.00 |
| **B′** eth up, idle | 3.4 – 3.5 | 1.00 |
| **C′** rx 4 Mbit/s | 8.9 | 1.00 |
| **C′** bidirectional | 11.6 | **1.01** |

### 4.3 Motor spinning, coupled to mag brake (100 rpm)

| Condition | late % | coalesce |
| --- | --- | --- |
| eth idle | 4.8 – 4.9 | 1.01 |
| + TX ceiling 64 B | 7.8 | 1.01 |
| + TX ceiling 512 B | 7.5 | 1.01 |
| + TX ceiling 1400 B | 7.4 | 1.01 |

### Claims supported by this data

> [!success] Idle ethernet is free
> A vs B differ by ≤0.2 pp, both idle and spinning. Installing the W5500 driver
> and holding link up costs nothing measurable. **The original concern — that the
> shared SPI2 bus or the GPIO14 IRQ would disturb the loop at rest — is
> disproved.**

> [!success] Traffic raises `late%` but plateaus
> ~5% → ~9% by 4 Mbit/s, and 6× more bitrate does not worsen it. Bidirectional
> reaches ~11.7%.

> [!success] Control cadence is essentially untouched
> `coalesce` was 1.00 in every idle run and never exceeded 1.01 anywhere,
> including the worst spinning + bidirectional case.

> [!warning] Mechanical load costs more than the network does
> Free-shafted 3.2% → braked 4.8% with no network activity at all. Network TX at
> the ceiling adds a further ~2.6 pp on top of that.

---

## 5 Control quality under load

`late%` is a proxy. These are the numbers that directly answer "did the loop
misbehave" — 15 s telemetry captures, spinning free-shaft at 100 rpm.

| Condition | rpm mean | rpm sd | iq mean | iq sd | mean \|aerr\| | obs err sd | stream rows/s |
| --- | --- | --- | --- | --- | --- | --- | --- |
| quiet 1 | 100.04 | 0.481 | 1.611 | 0.0911 | 0.08 | 1.582 | 8.9 |
| quiet 2 | 100.06 | 0.438 | 1.607 | 0.0875 | 0.05 | 1.687 | 8.9 |
| rx load | 100.04 | 0.468 | 1.640 | **0.1514** | 0.10 | 1.746 | **4.8** |
| bidirectional | 100.07 | 0.428 | 1.619 | 0.1019 | 0.14 | 1.757 | **4.3** |

> [!success] Speed regulation is unaffected
> rpm sd is 0.43–0.48 in **every** condition including the worst, and mean speed
> held 100.0 rpm throughout. This is the strongest single result in the session.

> [!warning] Current-sense noise is what degrades
> `iq` sd rose 0.091 → 0.151 (**+66%**) under rx load, and the glitch-filter
> counters roughly tripled (`imp` 33/24 → 105/94). Exactly consistent with `late`
> counting samples taken outside the null window: they are corrupted, then
> filtered out, and the loop never acts on them.

> [!warning] Telemetry throughput roughly halved
> 8.9 → 4.3 rows/s. See [[#8 The CPU0 finding and the lwIP A B C]] — this is a
> task-priority artifact, not an inherent cost of ethernet.

---

## 6 Packet rate is the constraint not bandwidth

The cleanest result of the session. Two RX runs, identical 11 s window:

| Run | offered | delivered pkts | payload | delivered bytes | late % |
| --- | --- | --- | --- | --- | --- |
| C1 | 1000 pps × 512 B | 3961 | 512 B | 2,028,032 | 9.2 |
| C2 | 3000 pps × 1024 B | 3977 | 1024 B | 4,072,448 | 9.1 |

**Same packet count. Double the bytes. Identical disturbance.**

Delivered packets capped near 360/s regardless of offered load. Confirmed
independently on the transmit path in [[#7 Transmit ceiling ethernet vs USB]],
where pps is flat across a 22× payload range.

> [!tip] Design consequence
> Each packet costs an SPI transaction plus an IRQ; bytes within a packet are
> nearly free. **Batch telemetry into few large datagrams rather than one per
> sample.** Packet rate is the budget, and it is the cheap thing to economize.

---

## 7 Transmit ceiling ethernet vs USB

Board-side, 5 s flat-out bursts. Firmware and host byte counts agreed closely in
both cases, so these measure the end-to-end path.

| payload | eth pps | eth kB/s | USB msg/s | USB kB/s |
| --- | --- | --- | --- | --- |
| 64 B | 262 | 16.4 | **2260** | 141.3 |
| 128 B | 262 | 32.8 | 1272 | 159.0 |
| 256 B | 260 | 64.9 | 714 | 178.4 |
| 512 B | 254 | 127.0 | — | — |
| 1024 B | 248 | 247.8 | — | — |
| 1400 B | 240 | **328.1** | — | — |

Repeated under mechanical load: 271 / 254 / 234 pps at 64 / 512 / 1400 B —
statistically the same, so **the ceiling is a property of the W5500 path, not of
system load**.

> [!important] The two transports fail in opposite ways
> - **Ethernet is packet-rate limited**: ~240–262 pps at *every* payload size.
>   Bandwidth scales linearly while pps stays flat.
> - **USB is bandwidth limited**: ~140–180 kB/s, but happily does 2260 msg/s.

### What this means for telemetry

| Framing | Effective sample rate |
| --- | --- |
| Ethernet, one datagram per sample | **≤ ~260 Hz** — a hard ceiling |
| Ethernet, batched 20 × 64 B samples per 1400 B datagram | ~5000 samples/s |
| USB console, 64 B per message | 2260 samples/s |

- Unbatched small messages: **USB beats ethernet ~8.6×**.
- Batched: **ethernet beats USB ~2.2×** on sample rate and 1.8× on bandwidth.
- Ethernet overtakes USB on raw bandwidth between 512 and 1024 B payloads.

The transport choice is really a **framing** choice.

---

## 8 The CPU0 finding and the lwIP A B C

> [!danger] The most important structural fact, and it is counterintuitive
> **The sampling ISR runs on CPU0, not CPU1.**
> The FOC *task* is pinned to CPU1 (`xTaskCreatePinnedToCore(..., 1)`), but
> `g_sampler.init()` is called from `app_main`, which is pinned to CPU0
> (`CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0`), and **ESP-IDF allocates interrupts on
> the calling core**. So the timing-critical ADC acquisition shares a core with
> the console, stream, and network tasks.

This single fact retroactively explains every result above: network load lives on
CPU0, which is why it always moved `late%` and never `coalesce`.

### Task map (as measured, not assumed)

| Task | Priority | Affinity |
| --- | --- | --- |
| FOC control | 20 | pinned CPU1 |
| lwIP tcpip | 18 | unpinned (IDF default) |
| netrx / nettx | 5 | pinned CPU0 |
| console (`app_main`) | 1 | pinned CPU0 |
| telemetry stream | **0** | unpinned |
| MT6701 encoder | **0** | unpinned |

The stream task is the **lowest-priority thing in the entire system**, which is
why it starves under load. That is a config artifact, not an ethernet cost.

### Controlled A/B/C

Identical builds except the one flag. `sdkconfig` was deleted and regenerated in
**all three arms**, so that confound is common and cancels. 4 × 10 s windows each,
ethernet up and idle, motor idle.

| lwIP affinity | late % (4 runs) | mean | coalesce | cmax µs |
| --- | --- | --- | --- | --- |
| **unpinned (default)** | 2.2 / 2.4 / 2.4 / 2.3 | **2.33** | 1.00 | 200–238 |
| pinned CPU0 | 3.4 / 3.3 / 3.2 / 3.2 | 3.28 | **1.01** | 188–213 |
| pinned CPU1 | 3.1 / 3.2 / 3.0 / 3.0 | 3.08 | 1.00 | **138–164** |

> [!failure] "Keep the network stack off the control core" was wrong here
> Pinning lwIP to CPU0 — the intuitive move — is **worse on both counts**. It
> piles contention onto CPU0, which is where the sampler ISR actually lives.
> Left at the IDF default (unpinned) in `halls-sensorless/sdkconfig.defaults`,
> with the reasoning recorded in-file.

> [!note] Pinning to CPU1 is the alternative worth revisiting
> It was clearly best on `cmax` (138–164 vs 200–238 µs) with `coalesce` at 1.00.
> If FOC compute headroom ever becomes the binding constraint instead of sample
> quality, that is the knob.

> [!info] A confound that was resolved, not hand-waved
> An earlier apparent improvement (4.9% → 2.4%) was initially credited to the
> pinning. The controlled A/B/C shows it was **not** — it came from regenerating
> a drifted `sdkconfig`. The original claim was withdrawn.

---

## 9 The encoder thread

Relevant to the fallback plan of using the magnetic encoder for field-estimated
control instead of the ADCs.

**It was already running during every measurement above.** `init_motor()`
(`sensorless.cpp:741`) unconditionally creates the MT6701 and starts its task
with `run_task = true` (`pace-racer-board.cpp:163-166`).

But it ran in a configuration where its jitter could not matter:

- Reads at **1 kHz** (`core_update_period_us = 1000`).
- **The app never reads it** — zero hits for `bsp.encoder()` in `sensorless.cpp`.
  Halls are the ground truth; the encoder is pure background overhead.
- `espp::Task` defaults: **priority 0, unpinned** — the joint-lowest priority in
  the system.

> [!success] Structural good news for the fallback plan
> The encoder is on **SPI3**, a completely separate bus from SPI2 where the
> DRV8353 and W5500 live. There is **no bus contention path** between the encoder
> and ethernet. At 8 MHz a 16-bit read is ~2 µs of bus time, so the headroom for
> raising its rate is there — it is a scheduling change, not a bandwidth one.

> [!warning] But priority 0 unpinned is a trap if it becomes the angle source
> Today it can float onto CPU0 and be preempted by the network stack, which is
> invisible because the results are discarded. Make it the high-speed angle
> source and that preemption becomes **direct angle error**. It would need
> pinning (to CPU1, per [[#8 The CPU0 finding and the lwIP A B C]]) and a
> priority above the network stack.

> [!note] Encoder angle replaces commutation feedback, not current feedback
> If a torque/current PI is retained, the ADCs stay on the critical path and the
> +66% current-noise finding still applies to them. "ADCs for power management
> only" works cleanly if moving to voltage or speed control.

---

## 10 Network setup traps

Three separate ways the transport bit us. All resolved.

### DHCP can never work on this rig

The board cables directly to the MacBook's USB dock — a point-to-point segment
with **no DHCP server on it**. Both ends wait forever for a server that does not
exist. This presents as a W5500 failure but is purely network config; link-up
already proves the SPI/PHY path is good.

### Tailscale hijacks link-local

```
169.254  link#21  UCS   utun4      <- Tailscale, wins
169.254  link#20  UCSI  en7    !   <- the actual board link
```

Tailscale installs a route for the whole `169.254/16` range that outranks the
physical interface. A perfectly healthy board shows 100% packet loss and no ARP
entry — indistinguishable from dead hardware.

### macOS re-rolls self-assigned addresses

The host's link-local address changed **three times in one session**
(`169.254.171.201` → `169.254.201.167` → absent → back), breaking host scripts
each time.

### Resolution: a dedicated static subnet

| End | Address |
| --- | --- |
| Board | `192.168.50.50 / 255.255.255.0`, no gateway |
| Host `en7` | `192.168.50.1 / 255.255.255.0` |

```sh
sudo ifconfig en7 inet 192.168.50.1 netmask 255.255.255.0
```

> [!danger] Do NOT use `networksetup -setmanual` for this
> `en7` sits at **service order 6, above Wi-Fi at 10**. Configuring a gateway
> there installs a higher-priority default route and **cuts the host's
> internet**. The `ifconfig` form sets the address only, adds no default route,
> and was verified to leave the existing default route on en0 untouched. The
> tradeoff is that it does not persist across reboots.

---

## 11 Conclusions and design guidance

> [!success] Ethernet and FOC can run simultaneously
> Speed regulation was unaffected in every condition tested, up to and including
> saturating bidirectional traffic while spinning under mechanical load.

### For the ATOS / RTPS work

1. **Budget packets, not bytes.** The ceiling is packet-rate-bound, not
   bandwidth-bound: with the 32 KB cache (§14) it is ~795 pps at 64 B, falling
   only to ~539 pps at 1400 B — 22x the payload costs just 1.5x in rate. 1 kHz
   telemetry as one datagram per sample is still out of reach; batched it is
   comfortable. (This item originally quoted a flat ~250 pps; that number
   predated the cache fix — see the correction at the top.)
2. **Batch aggressively.** 20 × 64 B samples per 1400 B datagram gives ~10 000
   samples/s at the measured 539 pps — roughly five times what the USB console
   can do.
3. **Give the telemetry task a real priority.** At priority 0 it is the lowest
   thing in the system and starves under load. Most of the observed halving is
   this, not the transport.
4. **Do not pin lwIP to CPU0.** Leave it at the default, or pin to CPU1. CPU0 is
   where the sampler ISR lives.
5. **RTPS discovery will need attention.** DDS SPDP discovery is UDP multicast;
   on a point-to-point link with no router or multicast querier it generally will
   not work, so expect to configure static/unicast peers. Note also that
   micro-ROS on ESP32 uses Micro XRCE-DDS, which is *not* RTPS on the wire — it
   talks to an Agent that bridges to real DDS.

---

## 12 What is NOT established

> [!caution] Read this before generalizing any number above
> - **All runs were 2–4 A at 100 rpm.** High-power dyno operation puts far more
>   switching noise on the current sense; the +66% `iq` noise could matter more
>   there. Re-check before assuming headroom.
> - **The encoder path was never stressed.** Its thread ran throughout, but at
>   1 kHz, priority 0, with results discarded. Nothing here says encoder-based
>   control would be immune to network load. Testing that requires pinning it,
>   raising its rate, and actually reading the angle.
> - **`cmax` regularly exceeded the 50 µs control period** (200–300 µs) while
>   `coalesce` stayed ~1.00. These are rare outliers over ~250 k ticks, but the
>   cause was not investigated.
> - **The USB sweep stopped at 256 B.** The ethernet/USB bandwidth crossover is
>   bracketed between 512 and 1024 B, not pinned down.
> - **Inbound RTPS processing cost is not established.** RTPS itself HAS been
>   run since this section was first written — discovery, publish-rate sweeps to
>   the 303 Hz ceiling, and endpoint scaling are all real-DDS measurements (§14);
>   the UDP work stands as the transport baseline underneath them. The remaining
>   gap is the receive path: no external discoverable writer exercised the
>   readers (their columns price only idle readers), and no real subscriber
>   drove QoS/acknowledgment traffic.
> - **The static IP is not persistent** on the host across reboots.

---

## 13 Errors made during testing

Recorded because each produced a plausible-looking wrong number, and the same
traps will recur.

| # | Error | How it presented | Caught by |
| --- | --- | --- | --- |
| 1 | Load-generator task polled `recv` with a 1 ms timeout, waking 1000×/s while idle | Phase B looked like **11.9%** — a 2.3× "cost of the driver" that did not exist. True value 5.0% | Suspicion that a driver at rest should not cost that much |
| 2 | zsh does not word-split unquoted `$spec` | Three "under load" runs silently ran with **zero load** | `nstat` showing `rx=0pkt` |
| 3 | Unbounded TX flood at priority 5 on CPU0 starved the console at priority 1 | Board stopped responding to the very `etx 0` that would stop it; needed a reflash | Console silence while the stream (unpinned, escaped to CPU1) kept running |
| 4 | Host receiver bound port 3333, but the board learns its peer from the first datagram — which came from `udpload`'s *ephemeral* port | Every transmitted packet went to a dead socket; "nothing arrived" | Board-side counters showing packets sent |
| 5 | Claimed stream and network tasks "both live on CPU0" | Plausible explanation for the telemetry halving; wrong | Reading the actual task configs |
| 6 | Credited a baseline improvement to the lwIP pinning while also regenerating `sdkconfig` | Two variables changed at once | Flagged at the time, then settled by the controlled A/B/C |
| 7 | Reset the board with a manual DTR/RTS toggle | Chip wedged; USB enumerated but neither app nor ROM bootloader responded. Needed a physical power cycle | esptool failing in all reset modes |

> [!tip] The recurring lesson
> Four of seven (1, 2, 3, 4) produced *silent* wrong answers rather than errors.
> Every load measurement needs an independent confirmation that load was actually
> applied — `nstat` counters on one side, host-side byte counts on the other.

---

## 14 Cache and IRAM revision

Measured 2026-08-04 after the RTPS work. Two changes, applied and measured
separately:

1. **Instruction cache 16 KB -> 32 KB** (`CONFIG_ESP32S3_INSTRUCTION_CACHE_32KB`,
   the S3 maximum). Data cache was already 32 KB.
2. **FOC control task into IRAM** (`IRAM_ATTR` on `foc_task_fn`), plus
   `main/linker.lf` placing the espp adc and bldc_driver archives in internal RAM.

### Effect on the control loop

| config | late% | isr_max | cmax |
| --- | --- | --- | --- |
| 16 KB icache | 4.9 | 19–22 us | 207–239 us |
| 32 KB icache | 4.9 | 12.5–16.3 us | 59–73 us |
| 32 KB icache + IRAM FOC task | 4.9 | 11.3–13.9 us | **36–39 us** |

`cmax` improved ~6x — from 4–5x *over* the 50 us control period to comfortably
inside it.

### Effect on transport throughput

| metric | before | after |
| --- | --- | --- |
| RTPS `pub_us` | ~11,400 us | ~3,200–3,839 us |
| RTPS ceiling | 85 Hz | **303 Hz** |
| raw UDP TX, 1400 B | 240 pps / 328 kB/s | **539 pps / 737 kB/s** |
| margin over the 10 Hz telemetry target | 8.5x | **30x** |

> [!failure] `late%` did not move — not once, across any of it
> It sat at 4.9% through every configuration. Per `foc_sampler.hpp`, `late` fires
> on either an over-budget ISR **or** a late ENTRY (`gap > expected + 5 us`).
> In-ISR duration fell sharply while late% held constant, so **late% is dominated
> by ISR entry latency** — the interrupt being held off before it starts, not
> running slowly once it does. The existing comments about the DRV SPI poll and
> LM75 I2C traffic delaying the sampler already suspected this.
>
> Cache and IRAM make the ISR run faster; they cannot make it start sooner.

**The remaining lever for `late%` is MCPWM interrupt priority.**
`mcpwm_timer_config_t` exposes `intr_priority`, but espp's `BldcDriver` never
sets it, so the sampling ISR is allocated at the lowest priority and loses to
SPI, I2C, and ethernet interrupts. Changing that needs an upstream espp change.

### Why the linker fragment is small

`main/linker.lf` maps only the espp adc and bldc_driver archives. Linker
fragments map whole archives/objects, not individual functions, and the code that
matters is not in a separate archive: `foc_sampler.hpp` is a header and
`BldcDriver::set_pwm()` is a header inline, so both compile *into* `sensorless.o`
— whose text is ~142 KB, most of it cold console and command parsing. Placing it
wholesale would consume more than half the remaining internal RAM to speed up
code that runs once. Hence `IRAM_ATTR` on the hot functions instead.

The sampling ISR itself is deliberately **not** `IRAM_ATTR`: `foc_sampler.hpp:662`
records that doing so mis-orders the literal pool at -O2 and fails to link with
"dangerous relocation".

---

## Appendix: reproducing

```sh
# build + flash
cd components/pace-racer-board/halls-sensorless
idf.py -p /dev/cu.usbmodem2101 flash

# host side (once per boot of the dock interface)
sudo ifconfig en7 inet 192.168.50.1 netmask 255.255.255.0
```

Console sequence for the full A/B/C:

```
s                      # baseline, no ethernet — reset then read after 10 s
eth                    # bring ethernet up
s                      # cost of the driver existing
etx max 5 1400         # bounded TX ceiling burst, self-reports
nstat                  # rx/tx counters
run 4 100 3            # spin (4 A needed for breakaway when brake-coupled)
stop                   # de-energize
```

> [!warning] Brake-coupled breakaway current
> Coupled to the mag brake the motor needs `run 4 100 3`. At 2 A it slips — I/f
> drives the field at 100 rpm while the rotor turns ~13. Known behaviour, not a
> fault; once spinning it settles back to ~1.6 A.
