# Dyno bench — handoff notes (last session 2026-08-21)

Read this before touching the bench. It is the consolidated state of the
characterization campaign: what exists, how to bring the bench up, every
gotcha that cost us time, and the agreed next step (the speed campaign,
planned but NOT yet run).

**Deliverable:** the six-figure characterization report lives at
`../report/figures.html` (regenerate: `python3 ../bench_report.py`) and is
published as a Claude artifact. Source data: `../logs/figs-20260819/`
(committed). The older hall-campaign report is `../report/index.html`
(`../compile_report.py`).

## Hardware map

| Thing | Where / how | Notes |
|---|---|---|
| Board (ESP32-S3 + DRV8353) | USB on **dock hub 3-1 port 1** (uhubctl-switchable, deliberately) | VM-powered; USB port name changes (`bench_lib._find_board_port()`), console 115200 |
| BK MR3K160120 (VM, 48 V) | `/dev/cu.usbmodem615J231251`, SCPI 115200 | **Never touch Vset.** Iset/output OK. Scripts raise Iset to 15 for power runs, restore 8 |
| Rigol DP2031 | pyvisa `USB0::6833::42152::DP2A284M00016::0::INSTR` | CH1 = torque sensor 24 V/0.5 A — **do not change**; healthy draw ≈ 0.13 A. CH2 = brake coil |
| Mag-particle brake | DP2031 CH2, coil ≈ 7.6 Ω | **200 N·m-class unit** — the 1–8 N·m "map" is its bottom curve. Harness allows 24 V / 3 A |
| Torque sensor | 0–10 V analog → scope CH1 | **Scale 200 N·m/V** (10:1 probe on 1× channel — verified vs the sensor's display) |
| Rigol MHO984 scope | **LAN only**: raw socket `169.254.100.100:5555` (static, set on scope) | **Never on USB** — its USB presence correlated with board console wedges. Mac needs a link-local source addr (auto-discovered by `bench_lib.Scope`) |
| Phase-C current clamp | Scope CH2, 100 A clamp, **0.01 V/A** | ~−0.77 A zero offset (zero it); orientation flips sign. Cross-checked board cal within 6% |
| Motor | NineBot S hub, 15 pp, R=0.166 Ω phase (0.332 line-line), L≈395 µH, Kt=0.60–0.62 N·m/A, J≈0.106 kg·m² (drag-incl.) | All fitted in the report §1 |
| MT6701 encoder | SSI via BSP; geared **2.5:1** off the wheel → elec = exactly 6× enc | Low-speed/stall specialist. 1 kHz BSP update ⇒ 36 elec-deg lag at 400 rpm — do NOT use at high speed; use halls |

## Bring-up procedure

1. Rigol: CH1 24.000 V / 0.5 A ON (verify ~0.128 A draw), CH2 0 V + **`:SOUR2:CURR 1.000`** OFF.
2. BK: `OUTP ON` (Vset stays 48). Board USB enumerates only with VM up.
3. `bench_lib.Board()` — auto-finds the port, quiets the stream (`sq 1`).
4. Arm: `arm_limits()` (lim 25 30, **vds 5**, vl 20) + `odg 8`. For power runs: `lim 30 35` (hard ceilings raised to 30/35 in fw, user-authorized).
5. Scope: `bench_lib.Scope()` — verifies CH1; for the clamp enable CH2 + `:MEAS:ITEM VAVG,CHAN2`. If VAVG returns 9.9e37 the channel is clipping — reset scale (CH1 0.2 V/div) and `:MEAS:CLE ALL`, re-add.

## The gotcha list (each of these cost real time)

- **Console wedge**: the USB-JTAG console goes mute (no bytes, esptool can't even reach the ROM). Chip cold boots do NOT clear it; USB re-enumeration does ⇒ stuck macOS driver instance. **Recovery: `bench_lib.usb_replug()`** (uhubctl power-cycles hub 3-1 port 1 — board keeps running, only USB resets). Prevention: keep the stream quiet around opens/closes (Board does this), avoid crashing scripts mid-stream (use try/finally with `stop`).
- **DRV latch vs console wedge are different**: `! DRV8353 fault 0x0620…` needs a **VM cycle** (`vm_recover.py`), not a replug.
- **Encoder cal**: use the **torque-peak sweep** (constant `eiq`, brake locked, step `eofs` 0→360; torque |cos| nulls locate the offset — nothing moves). The rotating I/f cal (`Board.ecal`) reads its own breakaway ring-down (~30° spreads) — don't trust it without steady-state windowing. Known-good offset ≈ 352.5° (changes if the mount is touched).
- **A dead encoder SSI link false-passes**: all-zero frames decode as cnt=0/NORMAL/OK. Always verify cnt moves with the wheel. `enc pu`/`enc pd` = MISO pull test.
- **Brake remanence**: torque at a given voltage rises after high-excitation events; the stall edge moves between runs. Approach edges from below; a stalled wheel may need full brake release to break away again.
- **Brake supply current limit**: Rigol CH2 can revert to 0.1 A and silently CC-clamp (~0.75 V) — the brake starves with no error. `Brake.set()` re-asserts the limit every write.
- **First serial command after open can be mangled** — everything goes through `ack()` with retries.
- **Stream regex**: temps go `nan` if LM75 init fails; a hung LM75/I2C read silently blocks the whole stream task (thermal guard blind!) — VM cycle recovers. Parse with the nan-tolerant regex (see fig6_hold.py).
- **lim is clamped by compile-time hard ceilings** (now 30/35) — read the `#lim` ack, don't assume.
- **Watch item (hardware)**: intermittent VDS faults naming **phase-B high-side** at low current near stress transients; not reproducible cold. A gate-drive scope session is recommended before pushing past today's envelope.
- **Tailscale** hijacks 169.254/16 when it's up — the scope LAN link needs it off (or routes fixed).

## Firmware console quick-reference (beyond the boot banner)

`erun/eiq/ecal/eofs` (encoder drive), `epos/pg` (position), `rl` (R/L ident),
`cap <every> <n> <mode>` + `capdump` (20 kHz ring: mode 0 pos/rpm/iq,
1 iα/iβ/vα, 2 drive-vs-encoder angle), `sq <0|1>` (stream quiet),
`vds <0-5>`, `odg <1|2|4|8>`, `hset`, `hs`, `enc`.

## State of the campaign

Done (all in the report): R/L/J identification + math, velocity & position
autotune, disturbance rejection, sweeping load, lock-rotor thermal
(baseline AND heatsinked — heatsinks now on the FET thermal-via pads;
board-sensor ΔT unchanged, but the 20 A DC hold now plateaus at 48 °C
where it used to VDS-trip), torque linearity (encoder Kt 0.60 flat vs
halls collapsing to 0.29–0.39), and **514 W peak bus at 150 rpm** (>500 W
for ~11 s, hall-commutated, ended cleanly by the 35 A guard).

## NEXT: the speed campaign (planned with the user, NOT yet run)

Goal: absolute limits via **speed, not torque** (protects the motor —
copper loss stays at the ~20 A level). Hall commutation (best at speed;
encoder is worst there). User is willing to exceed 150 rpm now.

1. **Prep (zero risk)**: raise `vl` stepwise 20 → 24 → 26 (SVPWM ceiling
   is 48/√3 ≈ 27.7 V), BK Iset 20, unloaded ladder to 500 rpm watching
   vq margin and hall glitch counters.
2. **Loaded ladder**: brake ≈ 2.5–3 V (8–12 N·m), 200 → 300 → 400 → 500
   rpm, 15 s dwells. Expected: ~760 W bus / ~630 W mech at ~80% efficiency
   around 500 rpm / 24 V.
3. **Endurance dwell** at the best point until FET plateau or 80 °C.
4. Aborts: vq > 0.9·vl, hall glitches, 80 °C, any trip, operator.
   Mind the brake (absorbs 600+ W continuously — FLIR it) and the encoder
   gear (spins 2.5×; 300 rpm wheel killed mount #1 — consider disengaging).

Runner scripts to crib from: `fig6_edge.py` / `fig6_hold.py` (power runs),
`fig4_heatsink.py` (thermal dwell + clamp logging).
