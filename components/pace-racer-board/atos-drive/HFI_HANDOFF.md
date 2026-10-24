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

## Update 2026-10-23 evening: the measurement chain was the "saliency"

Everything in "What was learned" items 3, 4 and 7 about saliency size is superseded. Two sampler effects were faking a large stator-fixed N:

1. **Median-of-3 in `FocSampler::median_filter`** (NineBot glitch filter). At fs/4 it alone fakes |N|/|P| 0.11-1.0 and eats 29-71 % of |P| (simulated). Now bypassed while any injection runs (`smed 1` = auto, default; `smed 0` always off, `smed 2` force on for A/B). The slew gate widens to 10 A in that mode; the plausibility gate and trips are unchanged.
2. **One phase per period (`m 1`, alternating)**: the other phase in every alpha-beta sample is 50 us stale. At fs/4 that alone fakes |N|/|P| 0.28-0.80; at fs/8 it fakes nothing. `hrot <Vh> [ms] [4|8]` now defaults to fs/8 (2.5 kHz) and also solves P, N from the fresh phase only (capdump rows 2-3; rows 0-1 are still alpha-beta). `m 2` is not an option: the post-hop conversion is corrupt.

Also: `kHfiMaxVolts` 0.25 -> 1.0 (watch `rej`/max|A| in `s`; fs/4 at 0.4 V tripped the 5 A slew gate), and `enc` rpm now prints the 50 ms-filtered rotor rpm (the old 1 ms raw value read ~2x off at random, which was the "2.25x").

**Result with the clean chain** (`tools/hfi_bench/enc_scan.py 0.25 3 800 15 out.json`: align with 3 A at 24 angles, release, read the encoder, `hrot 0.25 800 8`): |P| 2.25 A, rotor 2-theta term |Nr| 0.239 A = **10.6 % saliency**, residual 0.039. After subtracting the stator-fixed Ns and a fixed offset (137.7 deg in 2-theta at 0.25 V, 89.1 deg at 0.15 V: calibrate per Vh) the HFI angle matches the encoder to **3.8 deg rms, 8.1 deg max electrical**. At 0.15 V: 6.5 % saliency, 6.7 deg rms. |P| repeats to 0.5 % at a fixed position.

Caveats: the rotor only rests at 6 cogging detents (60 deg elec apart), where e^{j2theta} and e^{-j4theta} are indistinguishable, so "2 theta" is assumed from physics until checked off-detent. Sign of theta (180 deg ambiguity) still needs a polarity test.

Next: continuous rotating-injection demod + PLL (`hfi ... track` on the fs/8 P/N error, Ns and offset calibrated), compare `th_est` with the encoder while the rotor is turned slowly (this also breaks the detent alias), then polarity detection, then standstill start.

## Update 2026-10-23 night: continuous tracker, closed loop, and four DRV trips explained

**Bench now runs at 12 V** (Rigol 6+6 series) with a 12 V build: `idf.py -B build-drone12 -DATOS_MOTOR=drone -DATOS_VBUS=12 build`. Dead-time error and PWM ripple scale with the bus; at 12 V the same 0.17 V injection gives |P| 2.2 A with a cleaner N (spinning SNR 0.9 -> 1.5). Flashing a 24 V build onto the 12 V bus doubles every commanded voltage: keep build and supply matched.

**Fixes this round**
- `intr_priority = 2` for the MCPWM ISR (BSP-documented, was never set): late samples 4.7 % -> 0.0 %. An unrelated code change had shifted timing enough to expose it.
- `fresh_phase` is read with the currents (was read ~400 lines later; the 50 us period is shorter than the FOC task's cmax).
- hrt math uses an 8-entry phasor table and cached phase-pair constants: with `cosf/polar/complex divide` per tick the FOC task starved the console (looked like a hang).

**`hrt` continuous tracker** (`hrt <Vh> <ns_re> <ns_im> <off>`, `hrtg kp ki lpfP drive lpfN emax_deg`, `hguard <deg>`): fs/8, per-phase one-cycle boxcar, P/N solve, P filtered hard (0.01), N lightly (0.2), PLL in the 2-theta domain with the error clamped (emax 10-20 deg: a one-tick spike can no longer flip the 180 deg branch). Calibrate Ns/off with `tools/hfi_bench/enc_scan.py` + `hrt_cal.py` at the same Vh and bus. Current cal (12 V, 0.17 V): `-0.0176 -0.0427 149.2`.

Results vs the encoder (deg electrical):
- standstill, zero current: 4.8 sd (lpfN 0.01), offset -5.
- under encoder FOC at 30 rpm: 14-16 sd, offset -21. The rotor is not moving smoothly: cogging + the speed loop swing it between -96 and +109 rad/s electrical at a 6 rad/s average. The error is short-correlated noise (gone by 5 ms); it is NOT explained by iq, |iq|, iq^2, theta harmonics, mechanical-angle harmonics (off-centre magnet) or a dead-time relay model of the fundamental current (`model_fit.py`). The -21 deg offset appears whenever torque current flows, either sign.

**Closed loop on HFI**: `xrun <A> <rpm> [ramp]` / `xiq <A>` = HALL machinery with theta source 2 (theta_hat, omega_hat). Polarity is seeded from the encoder (no polarity test yet). `hguard 60` (default) stops the drive if theta_hat is >60 deg from the encoder for 10 ms - keep it on; it turned four would-be runaways into clean stops. Below breakaway (xiq 1 A) theta_hat holds within +/-7 deg. At 2.5 A the rotor snaps off its cogging detent and accelerates faster than the PLL follows (tested kp 300-3000); the branch flips, torque reverses, guard stops it. Next: feed-forward iq -> acceleration into the PLL (measure Kt/J from encoder data first), then load-offset compensation, then a polarity test.

**The DRV trips (0x0606, 0x060a x2, 0x0624)**: three were the console capture dump running while the motor was driven. USB console output holds the sampler ISR off; for ~200 us the duties freeze, the injection vector stops rotating and acts as DC: 0.15 V across 3 uH ramps 2.5 A per period (scope: 0 -> 8.6 A in 4 periods, `faultcap.py`), a little longer reaches the 17-22 A VDS trip. `hrt_log.py`/`xtest.py` now stop the motor before `capdump`. NEVER dump or print bulk console output while driving this motor. The fourth (0x0624) was the first `xrun` before the guard existed: HFI lost the angle and the speed loop ran away. Recovery is the VM cycle (scpi `:OUTP CH2,OFF`/`CH1,OFF`, 4 s, `CH1,ON`/`CH2,ON`) then `tools/usb_reset.py`. The root cause of the console stall (which interrupt-masking path in the USB console) is OPEN and is a safety item for any low-inductance motor.

## Bench scripts (`tools/hfi_bench/`)

`scpi.py HOST 'CMD'...` raw SCPI bound to en7 | `bootcap.py` arm scope on phase A, reset board, read RAW + bootlog (`--cmd 'rl 0.3 10'` sends a console command instead of resetting) | `rl_run.py 0.1 0.2 0.3 0.4` R/L steps + scope | `hfi_cap.py Vh bias` per-tick i/v capture under pulsating injection + lag correlation | `rot_cap.py Vh bias th` same for rotating injection + complex spectrum | `rot_scan.py Vh bias ms A1 A2 ...` rotating-injection saliency at several rotor angles (bias 0 = align with 2 A then release) + stationary/rotor split | `pin_scan.py`, `hscan_run.py`, `align_scan.py` pulsating variants | `enc_scan.py Vh Ialign ms step out.json` encoder-truth saliency scan (fs/8, fresh-phase P/N) | `hsweep.py th|- reps div smed Vh...` repeatability/A-B at one position | `con.py 'cmd[@wait]'...` console one-shots | `hrt_log.py every n out.npy cmd...` (CAPMODE=5/6/7/8) tracker logging, stops before dumping | `xtest.py out every n cmd@wait... ARM ...` closed-loop tests with capture armed before the drive | `faultcap.py lev_A tdiv wait tag cmd...` scope single-shot on phase A around a console sequence | `hrt_cal.py scan.json` -> `ns_re ns_im off` | `model_fit.py` offline N models | `spin_phase.py A rpm ramp` I/f spin + v-i phase fit | `coast_test.py`. All open `/dev/cu.usbmodem2101`; run from the venv.

Console sequence that is known-good at 24 V: `ho 9999 9000 1` (no observer handoff), `vl 6`, `run 3 60 2` (7 Hz electrical = 210 rpm mech) spins reliably; `ecal 4`; `erun 3 300 3`.

## Next steps

1. Saliency with truth: `rot_scan.py 0.12 0 1500 <angles>` but position the rotor with `erun`/`th`+`id 4` then release, and record `enc` elec for each point; fit N(theta_enc). Decide HFI viability from the rotor term vs the fixed inverter ellipse (|Ns| ~0.09 A).
2. If viable: close the `hfi ... track` PLL on the rotating-injection error instead of the pulsating one (demod already exists), compare `th_est` against the encoder in the stream, then I/f-free standstill start.
3. If not viable: I/f start + flux observer handoff is the fallback (`ho`, `mp`, λ to be measured from a coast capture with the encoder as truth).
4. Fix the `enc` rpm scaling; make the reset path coast the DRV before `esp_restart` (OTA reboot showed a 2-7 A transient as the bridge died).
