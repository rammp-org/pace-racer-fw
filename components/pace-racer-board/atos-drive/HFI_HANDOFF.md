# Drone-motor sensorless / HFI bench — handoff (2026-10-23)

Read this first in a new session. Everything below was verified on hardware today.
Memory notes with the same facts: `drone-motor-hfi-bench`, `mho984-scope-capture`,
`rigol-dp2031-lan-scpi`.

## Hardware on the bench

| Item | How to reach it | State |
|---|---|---|
| PACE RACER dyno board (ESP32-S3, DRV8353), USB PID 0x1209:0x0d41 "pace-racer-dyno" | CDC console `/dev/cu.usbmodem2101` 115200; reset `tools/usb_reset.py`; flash `python -m espp_ota flash --pid 0x0d41 -q build-drone/pace_racer_atos_drive.bin` (needs pyusb; `/tmp/hfi/venv` had pyusb+pyserial+numpy, recreate with `python3 -m venv` + `pip install pyusb pyserial numpy`) | Runs the build from this branch, boots clean |
| Rigol DP2031 supply | raw SCPI tcp 5555, DHCP client on the bench switch (169.254.50.13 today, was 169.254.146.149 self-assigned; sweep 169.254/16 or run `tools/mock_mib_dhcp.py` if it moved). Series mode `:OUTP:PAIR SER`; in series `:APPL` is ignored, use `:SOUR1:VOLT 12`, `:SOUR1:CURR 3`, same for SOUR2. CH1 on, then CH2. | 12 + 12 = 24 V, 3 A, outputs ON |
| Rigol MHO984 scope, Micsig CP2100A clamp on phase A (0.01 V/A) | static 192.168.1.101:5555, needs `sudo ifconfig en7 alias 192.168.1.50 255.255.255.0` (osascript admin prompt works). Probe ratio clamps at 10x: keep PROB 1, UNIT VOLT, multiply by 100. **`:TIM:MODE MAIN` before any single-shot** (50 ms/div silently enters ROLL and locks the sweep to AUTO). RAW readout: `:STOP`, `:WAV:MODE RAW`, `:WAV:FORM WORD`, chunk 250k with STAR/STOP/DATA?. See `tools/hfi_bench/RIGOL_MHO984_HANDOFF.md` and `tools/hfi_bench/bootcap.py`. | Running |
| Motor: small drone outrunner, no sensors, free rotor (clamped stator) | — | **2 pole pairs** (measured), R 36 mΩ/phase, L ~3 µH/phase, tau 83 µs, dead-time voltage error 0.13 V at 24 V/20 kHz |
| MT6701 encoder, magnet glued to the back of the bell, 1:1, not concentric | SPI3 MISO 35 / CLK 37 / CS 38; `enc` prints cnt/mech/acc/rpm | Works. Reads opposite to the drive direction (`kEncGear = -1`). `ecal 4` -> ofs 316.35 deg, consistency 0.964. `erun 3 300 3` spun it (32 revs in 4.5 s). The `rpm` field of `enc` is ~2.25x the true mech rpm (BSP scaling, not fixed); `acc` (degrees) is right. |

Mac side: Tailscale claims 169.254/16, bind sockets to en7's address (`tools/hfi_bench/scpi.py` does). `/dev/cu.usbmodem201301`, if present, is the M5Stack Tab5 HMI, not a motor board.

## Code state (branch `alexchunlin/high-frequency-injection`, worktree `~/orca/workspaces/pace-racer-fw/high-frequency-injection`)

Based on `feat/atos-drive` + the main checkout's uncommitted v3 diff. UNCOMMITTED. `managed_components/` and `dependencies.lock` were copied from the main checkout: without the lock, `idf.py` upgrades espp to 1.3.10 and tinyusb to 3.0 and `demo_link.hpp`/`eth_link.hpp` stop compiling (DispatcherModuleInfo grew a `protocol` field). Build:

```
bash -c "export IDF_PYTHON_ENV_PATH=$HOME/.espressif/python_env/idf6.0_py3.11_env; source ~/esp/esp-idf/export.sh >/dev/null; idf.py -B build-drone -DATOS_MOTOR=drone build"
```

Changes in `main/`:
- `atos_drive.cpp` `set_duties()`: atomic three-comparator write (counter-position guard + critical section), all nine `set_pwm` call sites use it. **This fixed the boot-time DRV faults 0x0621/0x0624**: espp writes comparators one at a time, a TEZ between writes = one-period duty mismatch = full bus into ~4 µH (13.5 A/us at 48 V, scope-confirmed). Optional `defer_past_tez` makes the write-to-effect latency a fixed 2 periods (used while HFI is on).
- `motor_params.hpp`: `drone` profile (pp 2, R/L measured, Kp 0.006 / Ki 72, vlim 3 V, bus 24 V, maxrpm 3000 mech), `kEncGear` per profile (2.5 on the NineBot profiles, -1 on drone). `main/CMakeLists.txt` maps `-DATOS_MOTOR=drone`.
- `foc_math.hpp` `svpwm(..., comp_abc)`: optional per-phase voltage add (dead-time comp).
- HFI console: `hscan <Vh> [Id]` pulsating scan of the injection frame (36 angles, synchronous pattern averaging, result via `capdump`: angle+1000*rotation, d amp, q amp); `hrot <Vh> [ms]` rotating injection with positive/negative-sequence demod (`capdump` row0 = P re,im,n; row1 = N re,im,Vh; `arg N + arg P + 180 = 2 theta`, latency cancels, `|N|/|P|` = saliency incl. inverter ellipse); `hfi <Vh> [Id] [track]` continuous injection + PLL (PLL UNTESTED); `hfig kp ki lag sign half`; `dtc <V> [band]` dead-time comp (leave at 0, see below). `hfi` alone prints status.

## What was learned (do not re-learn)

1. Pulsating injection at fs/2 (+,-) is invisible to period-centre current sampling: the current triangle crosses its mean at every sample. Use +,+,-,- (fs/4) or the rotating vector.
2. The duty write lands before TEZ on most ticks and after it on slow ticks (FOC task cmax ~130 us), so the effect latency jitters 1..2 periods; `defer_past_tez` fixes it at 2. Demod lag then = 2 (`hfig ... lag 0` = auto picks the largest positive d response).
3. Dead-time compensation with a linear band is positive feedback (0.13 V / 0.3 A = 0.43 V/A, 70x Kp): the loop limit-cycled at ~400 Hz. Keep `dtc 0`. If needed later, do it as feed-forward on the commanded current, not on the measured one.
4. Pinning the rotor with a d-axis bias is useless for saliency work: the dead-time "virtual saliency" follows the bias vector exactly like a rotor would. Measure with zero bias and move the rotor independently (now possible: the encoder gives truth).
5. With R/L = 83 us the electrical response settles within 2 PWM periods; injection must be at 5 kHz or faster.
6. At 6 A I/f the current loop went unstable near 1400 rpm mech and the DRV latched 0x0622 (VDS HA+HC). 3 A I/f was fine to ~1400 rpm, pulled out above. Don't raise current without handling the dead-time relay behaviour.
7. All standstill saliency numbers taken today (inverter ellipse |N|/|P| 0.62 at zero bias, rotor term 1.9 %) were with the rotor at ONE position (the 2 A alignments did not move it: 2 A is about the cogging torque). Saliency is UNRESOLVED, not absent.

## Bench scripts (`tools/hfi_bench/`)

`scpi.py HOST 'CMD'...` raw SCPI bound to en7 | `bootcap.py` arm scope on phase A, reset board, read RAW + bootlog (`--cmd 'rl 0.3 10'` sends a console command instead of resetting) | `rl_run.py 0.1 0.2 0.3 0.4` R/L steps + scope | `hfi_cap.py Vh bias` per-tick i/v capture under pulsating injection + lag correlation | `rot_cap.py Vh bias th` same for rotating injection + complex spectrum | `rot_scan.py Vh bias ms A1 A2 ...` rotating-injection saliency at several rotor angles (bias 0 = align with 2 A then release) + stationary/rotor split | `pin_scan.py`, `hscan_run.py`, `align_scan.py` pulsating variants | `spin_phase.py A rpm ramp` I/f spin + v-i phase fit | `coast_test.py`. All open `/dev/cu.usbmodem2101`; run from the venv.

Console sequence that is known-good at 24 V: `ho 9999 9000 1` (no observer handoff), `vl 6`, `run 3 60 2` (7 Hz electrical = 210 rpm mech) spins reliably; `ecal 4`; `erun 3 300 3`.

## Next steps

1. Saliency with truth: `rot_scan.py 0.12 0 1500 <angles>` but position the rotor with `erun`/`th`+`id 4` then release, and record `enc` elec for each point; fit N(theta_enc). Decide HFI viability from the rotor term vs the fixed inverter ellipse (|Ns| ~0.09 A).
2. If viable: close the `hfi ... track` PLL on the rotating-injection error instead of the pulsating one (demod already exists), compare `th_est` against the encoder in the stream, then I/f-free standstill start.
3. If not viable: I/f start + flux observer handoff is the fallback (`ho`, `mp`, λ to be measured from a coast capture with the encoder as truth).
4. Fix the `enc` rpm scaling; make the reset path coast the DRV before `esp_restart` (OTA reboot showed a 2-7 A transient as the bridge died).
