# HALLS-SENSORLESS VARIANT

This directory is a fork of `sensorless/` (2026-07-30) adding **hall-commutated
FOC** as a low-speed test drive; the sensorless original is frozen upstream and
everything in the base doc below still applies here. What this variant adds:

- `hall_drive.hpp` — control-grade hall commutation: per-sector center-angle
  table **measured by `hcal`** (absorbs wiring order, mounting offset, and
  direction — nothing is assumed), transition-interval speed with stall decay,
  sector-clamped interpolation. Advances every FOC tick; drives in HALL mode.
- Mode `HALL` (stream chars: `L` speed / `W` sine / `Q` torque), all exiting
  through the same gentle STOPPING path; thermal guard covers it.
- Console: `hcal [s]` (during I/f: `ho 9999 45 1` then `run 2 30 5`),
  `hrun <A> <rpm> [ramp]`, `sine <A> <rpm_amp> <period_s>` (speed follows a
  sine through zero/reversal — the wheelchair-load sweep), `hiq <A>` (absolute-
  angle stall torque), `hofs <deg>` trim, `hs` diagnostics (state, sector,
  steps, rpm, glitch counters, table). Hall modes are **gated on hcal**.
- `kHardMaxVolts` raised 24 -> 26 V (SVPWM linear-region edge at 48 V).
- monitor.py: hall mode names + an rpm-tracking plot (est vs hall).

Session recipe: flash -> `z` -> `e 1` -> `ho 9999 45 1` -> `run 2 30 5` ->
`hcal` -> wait `#hcal OK` -> `stop` -> `sine 4 30 10` -> brake up gradually.

---

# PACE RACER Sensorless FOC — Engineering Notes

Firmware: `components/pace-racer-board/sensorless/` · Hardware: ESP32-S3 + DRV8353 gate driver, 1 mΩ low-side shunts, 3 hall sensors (validation only), MT6701 encoder (unused).
Bring-up: 2026-07-21 → 2026-07-26. Status: **closed-loop sensorless FOC with speed regulation, validated against halls.**

---

## 1. What this is

Sensorless field-oriented control for a BLDC motor: the rotor angle comes from a flux-linkage observer fed by measured phase currents and commanded voltages — no encoder, no halls in the control path. The halls on this board are noisy (the reason for going sensorless); they stay wired purely as a ground-truth check.

The bring-up was staged so the observer was **validated against the halls before it ever had control authority**:

0. PWM-synchronized current sampling proven (timing, null vector, ADC-in-ISR)
1. Sampler → control task handoff at 20 kHz
2. dq current loops at a fixed angle, then open-loop I/f rotation
3. Flux observer running in parallel, logged vs halls, zero authority
4. I/f → observer handoff, fail-safe, speed loop  ← **complete**

`current-control/` is the frozen last-known-good wired-pair reference app; never modify it.

---

## 2. Architecture

### Files (`main/`)

| File | Role |
|---|---|
| `sensorless.cpp` | App: DRV arming, FOC control task (CPU1), console, 10 Hz stream, mode state machine, speed loop |
| `foc_sampler.hpp` | `FocSampler`: PWM-synchronized low-side current sampling + the glitch-filter stack (the hard part) |
| `foc_math.hpp` | Forward Clarke/Park + SVPWM (espp has no forward path) |
| `foc_observer.hpp` | `FluxObserver`: flux-linkage observer + PLL (ODrive-style) |
| `hall_sensor.hpp` | Hall reader — ground truth only, never in the control path |
| `adc_fast.c/.h` | C shim for the one ADC LL call that isn't C++-clean |

### Execution model

- **Sampling ISR** (CPU0, fires at MCPWM **TEP** = timer peak): one ADC conversion per 50 µs PWM period, alternating phases A/B each period. Integer-only (no FPU in Xtensa ISRs). Runs the filter stack, the soft-trip check, then notifies the FOC task.
- **FOC task** (CPU1, high priority): woken per sample via `vTaskNotifyGiveFromISR`. All float math: Clarke → Park → 2× PI → inverse Park → SVPWM → `set_pwm`. Also runs the observer, the handoff state machine, and the speed loop. Publishes a coherent snapshot via seqlock.
- **Stream task** (CPU0, 10 Hz): CSV telemetry (silent while disarmed), DRV fault poll (5 s), trip reports, and the Hi-Z coast handshake (the FOC task can't touch SPI).
- **Console** (CPU0): line-based commands over USB-serial-JTAG.

### Why sampling looks the way it does

- **TEP is the null vector on this board** (verified on hardware; the espp docstring names the wrong edge, and espp only exposes `on_empty`, so the timer handle is borrowed via a friend-injection template to register `on_full`).
- Phase currents are read on the low-side shunts, valid only while low sides conduct — the sample must land inside the "null window" centered on TEP. **Late-entry ISRs are rejected** (entry-gap + in-ISR-time checks) because their conversions land outside the window and read garbage.
- Phases are fixed to **A and B** (`p 0 1`), C reconstructed from `ia+ib+ic = 0`. (Phase C's channel was condemned early as "bad hardware" — later evidence says it was ADC conversion corruption like everything else; rehabilitating it and restoring lowest-two-duty selection is queued work.)
- The channel for each conversion is **pre-selected at the end of the previous ISR**, so the mux settles for a full period.

---

## 3. The ADC corruption saga (read before touching the filter)

The single biggest time sink of the bring-up. The ESP32-S3 SAR ADC, driven via a hoisted fast path (`adc_oneshot_hal_convert` directly; the public `adc_oneshot_read_isr()` re-runs REGI2C cal every call, ~14 µs, and trips the interrupt WDT), produces **corrupt conversions**: a continuum from ~3 A errors up to 50 A phantoms on a ≤6 A real signal. Proven not to exist in the copper (current clamp) and refuted by twin conversions 8 µs apart (472/472 spikes disagreed with their twin).

Confirmed mechanisms:

1. **A debug GPIO toggle on an ADC pin.** The ISR's scope pin was GPIO2 = ADC1_CH1, same SAR unit and pin row as the sense inputs (GPIO4/5/6). Toggling it 1–2 µs before the conversion injected charge — the dominant slot-1 corruption. **Never scope-toggle on an ADC-adjacent pin of the unit you're sampling.** (`kScopePin = GPIO_NUM_NC` now.)
2. **PWM switching edges landing mid-conversion.** A conversion spans ~7.2 µs of SAR bit decisions; an edge's supply/ground bounce during them flips bits (early bits → 40 A monsters, late bits → the small-error continuum). At idle all phases switch at TEP±12.5 µs, which is why back-to-back second conversions (spanning +8.5…+15.7 µs) were systematically corrupted; at speed, duty spread walks edges toward the conversion tail near the modulation peaks — the "glitches correlate with rotation / sine peak" mystery.
3. Hall edge bursts on GPIO3/GPIO9 (also ADC1 pins, board routing) are a residual async aggressor.

### The filter stack (`FocSampler::median_filter`), in order

1. **Plausibility gate** (`kMaxPlausibleAmps`, 18 A): a reading beyond any physically possible current is always held, never accepted, never earns escape credit. Must sit **above** the software trip or the trip can never fire.
2. **Slew gate** (`kMaxJumpAmps`, 5 A vs the last *accepted* value) with a **bounded escape**: after 3 consecutive in-band rejections the level steps toward raw by one slew bound (never jumps to raw — a decaying spike burst once walked the filter straight to a phantom trip). The 5 A bound is physics: max real di/dt ≈ vlim / L_line-line per per-phase interval. **Revisit if `kVoltageLimit` is raised.**
3. **Median-of-3** for small noise.

Hard-won filter lessons (do not relearn):
- Hold-forever filters **latch**: real dynamics eventually exceed any gate, and then all real data reads as "glitches" while the loop winds up on the stale value. Every hold needs a bounded escape.
- Run-length rules alone always lose — burst length is unbounded.
- No sample-local gate can separate a corruption continuum that overlaps the real signal range. The fixes that actually worked attacked the *source* (scope pin, pre-select) or used *time* (soft trip) as the discriminator.
- `dbl 1` (verification pair mode) re-converts the same channel per ISR and reports agree/disagree stats in `s` — the tool that cracked the mechanism. Leave OFF for normal operation.

---

## 4. Control pipeline

### Modes (stream `mode` column)

| Char | State | Angle source | Current targets |
|---|---|---|---|
| `H` | HOLD | console `th` | console `id`/`iq` |
| `S` | SPIN: I/f | synthetic ramp | id=0, iq=run amps |
| `V` | SPIN: CONVERGE | synthetic ramp (untouched) | iq **decaying** |
| `C` | SPIN: CLOSED | **observer** | iq from **speed loop** |
| `X` | STOPPING | last frame, still rotating | 0 (PI intact) |

### Startup: I/f
`run <amps> <rpm> <ramp_s>` → DC align at angle 0 (300 ms, Id = amps) → rotate a synthetic angle, ramping to the target electrical frequency with Iq = amps. Current is closed-loop throughout; only the *angle* is open-loop.

### Handoff: current-decay convergence (the part worth understanding)

An angle crossfade was tried first and **failed on hardware**: torque ∝ sin(γ) (γ = rotor-flux→current-vector angle), so rotating the drive angle toward max-torque orientation with iq unchanged multiplies torque mid-blend → acceleration burst → pole slip.

The working method never touches the angle. In `V`, iq decays slowly (default 1 A/s, floor **0.5 A absolute** — see below). Torque balance `T = k·λ·I·sin(γ)` forces γ toward 90° as I falls — the rotor flux walks itself onto the synthetic d-axis. The stream's `aerr` column (`th_est − th_drive`, wrapped) shows it live: ~+90° in light-load I/f → ~0° when converged. When |aerr| < 15° for 20 ms, the angle source switches (a near-no-op) and `C` begins **at the decayed current** — torque is continuous by construction.

**The floor must be absolute, not fractional (dyno lesson, 2026-07-27):** convergence requires decaying iq down to ~the machine's actual drag current (sinγ = T/(kλI)), and drag doesn't scale with commanded amps. The original 25 %-of-run-amps floor made `run 10` unloaded park at aerr ≈ 90° at a 2.5 A floor, hit the stuck-abort every ~8 s, and the then-instant floor→amps iq snap raced the soft trip to the VDS OCP (the "iq sawtooth with OCP spikes"). Now: floor = 0.5 A, and **every abort/revert restore ramps iq back at the `ho` decay rate** — never a step.

Guards: engage needs 50 ms *sustained* `rpm_est ≥ engage` + healthy flux (single-tick triggers chattered on hardware); any revert/abort sets a 500 ms cooldown; stuck at the floor unconverged for 500 ms → abort to I/f (ramped). A `C` revert **with iq pinned at the clamp** (≥95 % of run amps) means the load beat the torque ceiling — I/f can't win that either, so it gentle-stops with `! torque ceiling exceeded` instead of slip-chattering against the brake.

### Speed loop (in `C` only)
Outer PI: rpm setpoint (ramped at the run's ramp rate) → iq, clamped to ±run-amps (**amps = torque ceiling**; negative iq = regen braking allowed). Seeded bumplessly at the `V`→`C` transition (setpoint = actual rpm, integrator = decayed iq). Re-issuing `run` while in `C` retargets the setpoint live without realigning. Few-Hz bandwidth vs the 400 rad/s current loop — decades of separation, tune fearlessly.

### Observer (`foc_observer.hpp`)
Flux-linkage integrator (ψ = ∫(v − R·i)dt, flux = ψ − L·i) with a nonlinear correction pulling |flux| → λpm, angle from atan2, PLL for smoothing. Fed measured i and the **previous tick's commanded** v — so it is only as honest as `kBusVoltage` (SVPWM scaling) and the deadtime error, which is why it has a **low-speed floor: ~30–40 rpm on this motor at 24 V** (measure it per motor/voltage by sweeping `run` speeds and watching where `rpm_est` unhooks from `rpm_drive`).

### Stopping and safety — everything is gentle now

The founding insight (user's): **the old safety responses were themselves the violence.** Braking (comparators→0) shorts the back-EMF (~30 A bang at speed), which fired the hardware OCP and made phantom trips indistinguishable from real events.

| Layer | Trigger | Response |
|---|---|---|
| Plausibility/slew/median | per-sample | filter only, never trips |
| **Soft trip** | filtered \|i\| > 15 A, 3 consecutive ticks (incl. reconstructed C) | record + freeze capture → FOC task enters STOPPING (closed-loop unload to 0 A — works at any angle) → Hi-Z coast + full report. No brake, no disarm. |
| **Flux fail-safe** | flux outside [0.35, 3.0]·λ for 40 ms while observer has authority | same STOPPING → coast path + `!` message |
| **DRV8353 VDS OCP** | ~17–22 A instantaneous | hardware-latched, outputs Hi-Z. **Survives ESP reset — needs a VM power cycle.** `0x0620` = FAULT+VDS_OCP+VDS_HA. |
| PSU current limit + scope | bench | you |

`stop` = the same STOPPING path. The PI is *not* reset while the bridge is live (resetting it discards the integrator's bemf-cancellation and creates the spike); it resets after coast.

---

## 5. Console reference

| Command | Effect |
|---|---|
| `e 1` / `e 0` | Arm on TEP / TEZ edge (always use `e 1`); zeroes targets, clears coast, recovery after trips |
| `f` | Disarm targets |
| `run [A rpm ramp_s]` | I/f start → auto handoff → speed-regulated. Defaults 2/50/2. In `C`: live retarget |
| `stop` | Gentle stop: unload → Hi-Z coast |
| `id x` / `iq x` / `th deg` | HOLD-mode direct targets (clamped ±8 A) |
| `g kp ki` | Current-loop PI gains |
| `sg kp ki` | Speed-loop gains (A/rpm, A/rpm·s) |
| `mp R L_uH lambda` | Observer motor params |
| `og fluxgain pll_kp pll_ki` | Observer/PLL gains |
| `ho engage revert decay` | Handoff: engage rpm, revert rpm, iq decay A/s |
| `lim A trip_A` | Raise/lower console clamp + soft trip live (plausibility follows at trip+3). Hard ceilings 25/30 A |
| `vl volts` | Current-loop voltage clamp (boot 8 V, ceiling 24 V). At 20 A the iq·R term alone is ~6.5 V — raise before loaded runs |
| `tl degC` | Board thermal guard (boot 80 °C): any LM75 over it while driving → gentle STOPPING |
| `vds 0-4` | DRV VDS OCP level 0.06–0.10 V (~17–22 → ~29–37 A cold; derates as FETs heat). 0.20 V step not offered |
| `s` | Stats (ISR timing, late %, per-phase filter counters imp/rej/esc, max\|A\|, dbl verdict) — resets counters |
| `c` | Full 256-sample raw capture dump (pairs adjacent in dbl mode) |
| `dbl 1/0` | Verification pair sampling (diagnostic; OFF for normal op) |
| `z` | Zero-calibrate current sense (braked). Run after supply/motor changes |
| `a` | One-shot async current read via BSP path |
| `r` | Dump DRV8353 registers |
| `p x y` / `p auto` | Sampled phase pair / lowest-two-duty auto select (auto currently off) |
| `m 1/2`, `n N` | Conversions per ISR, decimation (bring-up tools) |

**Stream** (10 Hz, silent when disarmed): `t, mode, id, iq, iqref, vd, vq, aerr, rpm_drive, rpm_est, rpm_hall, flux, t0, t1, t2, t3`.
`vd/vq` are the PI outputs (host derives P = 1.5(vd·id+vq·iq), bus current, |v|/vlim saturation, and the vd/id copper-R proxy); `t0–t3` are the LM75s at 2 Hz (NaN on read failure).
Healthy `C`: id≈0, iq = load demand, aerr ≈ 0, all three rpm columns agreeing, flux ≈ λpm.
Zero-cal also prints a `#range` line: the ADC rail in amps from the measured zero — the soft trip is blind past it.

**Trip report** (~20 lines): cause line (which phase incl. reconstructed, filtered A/B/C vs the tick's raw), per-phase filter counters since arm, 8-window per-phase median trend (real overcurrent ramps into the limit; phantoms have a flat baseline), and the >5 A anomaly list with ISR timing.

---

## 6. Parameter reference (all in `sensorless.cpp` unless noted)

### Motor-specific — MUST change per motor
| Parameter | Current value | How to obtain |
|---|---|---|
| `kPolePairs` | 15 | datasheet / count (NineBot S = 15, same as prior motor) |
| `kObsR` / `mp` | 0.323 Ω | auto-tuner sys-id (current-control app) |
| `kObsL` / `mp` | 336 µH | sys-id |
| `kObsLambda` / `mp` | **0.026** — measured on the NineBot S 2026-07-27 (0.025–0.027 at speed) | read the `flux` column at speed; it settles at the true λpm |
| `kDefaultKp/kKi` / `g` | 0.0568, 54.5 (169 rad/s for R=0.323/L=336µ) | kp = L·ω_bw, ki = R·ω_bw |
| `kHoEngageRpm/RevertRpm` / `ho` | 60 / 45 | 1.5–2× the measured observer floor |

### Bench/supply
| Parameter | Value | Notes |
|---|---|---|
| `kBusVoltage` | **48.0** | MUST equal the real supply: SVPWM scales by it → wrong value miscalibrates the loop 2× AND breaks the observer's voltage model. Re-`z` after changing supplies. |
| `kVoltageLimit` | 8 V boot, `vl` live (≤24) | PI output clamp; caps top speed (ω_max ≈ vlim/λ) and, under load, iq·R headroom. Raising it → revisit `kMaxJumpAmps` (slew physics scales with vlim/L; fine to 24 V at 336 µH). |
| `kMaxTargetAmps` | 8 A boot, `lim` live (≤25) | console clamp = torque ceiling ceiling |
| `kTripAmps` | 15 A boot, `lim` live (≤30) | soft trip; plausibility gate auto-follows |
| `kPlausibleMarginAmps` (`foc_sampler.hpp`) | +3 A above trip | plausibility gate must stay **above** the trip to keep it visible |
| `kMvToA` | 0.05 A/mV | CSA at **GAIN_20** — NOT the BSP's GAIN_5 constant |

### Behavior tuning (safe to experiment)
| Parameter | Value | Meaning |
|---|---|---|
| `kHoDecayAps` / `ho` 3rd arg | 1 A/s | converge decay AND abort-restore ramp rate; slower if `V` aborts, faster if it dawdles |
| `kIqFloorAmps` | 0.5 A absolute | decay floor ≈ this rig's drag current; must NOT scale with run amps |
| `kConvergeDeg` / `kConvergeTicks` | 15° / 20 ms | switch cone |
| `kSpeedKp/Ki` / `sg` | 0.05, 0.2 | speed loop; halve if hunting, raise if soggy under load |
| `kFluxFailLo/Hi`, `kFluxFailTicks` | 0.35/3.0×λ, 40 ms | fail-safe band |
| `kAlignMs`, `kSpinAmps/Rpm/RampS` | 300 ms, 2 A/50 rpm/2 s | I/f defaults |
| `kStopUnloadTicks` | 100 ms | unload time before Hi-Z |
| `kMaxRejectRun`, `kMaxJumpAmps` (`foc_sampler.hpp`) | 3, 5 A | filter escape/slew — see §3 before touching |

---

## 7. Motor swap procedure (e.g., → test-stand motor)

1. **Mechanical/bench**: motor secured; PSU current limit low; VM **on before boot** (always); scope clamp on a phase.
2. **`kPolePairs`** — compile-time; also fixes `kRpmToOmegaE` (all rpm↔ω conversions).
3. **Sys-id R and L** with the auto-tuner in the `current-control` app (its tuning applied to a *series* wired pair — R/L per phase are half the pair values).
4. **Current-loop gains**: `kDefaultKp = L·400`, `kDefaultKi = R·400` as the starting point (or `g` live). Validate in HOLD: `id 1` → id tracks, iq≈0, `vd ≈ id·R` + deadtime offset.
5. **Zero-cal**: `z` (and re-check `#zero … OK` at every boot).
6. **λpm**: spin I/f only (`run <A> <rpm>` below the engage speed, or temporarily `ho 9999 …` to prevent handoff), read where `flux` settles → `mp R L_uH <that>` and update `kObsLambda`. Sanity: λ ≈ V_bemf,peak-phase / ω_e.
7. **Observer floor**: sweep speeds downward, find where `rpm_est` unhooks from `rpm_drive` → set `ho` engage ≈ 1.5–2× that, revert ~25 % below engage.
8. **I/f current**: heavier rotor may need more `run` amps to align and ramp without slip (hunting/clunking = insufficient torque margin).
9. **Handoff test**: `run <A> <target>` → watch `S→V→C`, `aerr` → 0, three rpm columns agreeing. Tune decay via `ho`.
10. **For high power**: raise `kMaxTargetAmps`, `kTripAmps` (+ `kMaxPlausibleAmps` above it), possibly `kVoltageLimit` (→ revisit `kMaxJumpAmps`), PSU limit, and watch the LM75 temps. Ripple ∝ Vbus/(L·f_pwm) — a lower-inductance motor at 48 V rides a lot of ripple.

---

## 8. Known issues / queued work

- **Noise floor** (deferred, worth doing before high-speed runs): tighten late-entry margin 5→2 µs; re-test phase C (`p 0 2` at a gentle spin — its "bad hardware" verdict predates the ADC findings) and restore lowest-two-duty auto selection so conversions stay clear of PWM edges at high modulation.
- **Hall pins are ADC1 pins** (GPIO3/9, board routing) — residual async aggressor; filter backstops cover it. Board-rev consideration.
- **Upstream to BSP** once stable: the sampler, GAIN_20 scale (BSP's `CURRENT_SENSE_MV_TO_A = 0.2` is a GAIN_5 placeholder), normal-mode zero-cal. Upstream an `on_full` callback to espp and drop the timer borrow.
- **EKF / model-predicted gate**: considered, deliberately deferred. A 4–5-state EKF at 20 kHz is feasible on the S3 but mainly buys low-speed estimation; the pragmatic intermediate is centering the ISR's gate on a model-predicted current (posted as integers by the FOC task) if measurement quality ever limits again. Remember: Kalman-style blending alone makes outlier glitches *worse* — the value is in the innovation gate.
- Torque-sensor serial capture: `monitor.py`'s `SerialTorqueSource` is a stub awaiting the sensor's cable/protocol; CSV/report columns already exist.

## 9. Gotchas (cost real time — do not relearn)

- **No FPU in Xtensa ISRs** (coprocessor exception). Sampler ISR is integer-only; all float FOC math lives in the CPU1 task.
- `adc_oneshot_read_isr()` is unusable at PWM rates (re-runs REGI2C cal, ~14 µs). Requires `CONFIG_ADC_ONESHOT_CTRL_FUNC_IN_IRAM=y`, `MCPWM_ISR_IRAM_SAFE=n`.
- **Never toggle a debug GPIO on an ADC-adjacent pin** of the ADC unit you sample (see §3).
- espp `BldcDriver`: only exposes `on_empty`, docstring names the wrong edge, fixed 20 kHz. TEP is the null vector here.
- espp `disable()` = **brake** (stops at peak, low sides on), not coast. True coast = DRV8353 `set_coast(true)` (Hi-Z), which also idles the body diodes since bus ≫ bemf.
- Never reset the current-loop PI while the bridge is live and spinning — the integrator holds the bemf cancellation.
- **VM on before boot**; a `BAD` `#zero` line means the offsets are garbage — fix power, re-`z`.
- DRV latched faults (e.g., `0x0620`) survive ESP resets; VM power cycle to clear.
- 20 kHz PWM is near-optimal for 94 µH (ripple already ~3 A p-p at 24 V, ~6–7 at 48 V); slower doubles ripple, faster shrinks the sampling window.
- clangd in-editor errors (`sys/features.h`, `-mlongcalls`) are stale-index noise — trust `idf.py build`.
- The stream's `rpm_hall` is windowed sector-counting; the old last-interval estimator exploded on hall noise (±14,000 rpm readings).

---

## 10. Motor-swap / tuning log

### 2026-07-27 — λpm measured; high-power test instrumentation
- λpm measured on the bench (I/f spin, `flux` column): **0.025–0.027** → `kObsLambda` = 0.026.
- Stream extended (`vd, vq, iqref, t0–t3`), LM75s initialized in this app, thermal guard (`tl`, boot 80 °C) wired to the gentle-STOPPING path, runtime limit commands `lim`/`vl`/`vds` added, `#range` ADC-rail line at zero-cal, plausibility gate now trip+3 A.
- `monitor.py` added: textual TUI (live panels + temp/current/power plots, escape = stop, dT/dt watchdog) and per-run recorder (`logs/run-*/`: raw.log, data.csv, events.log, report.md). See §11.
- First dyno session found two handoff failure modes (iq sawtooth → OCP at high-amps unloaded handoff; slip oscillation when the brake beat the torque ceiling). Fixed: absolute 0.5 A converge floor, ramped (never stepped) abort/revert iq restore, and clamp-pinned `C` revert now gentle-stops with `! torque ceiling exceeded`. Details in §4.

### 2026-07-26 — NineBot S motor
New motor connected. Re-ran the `current-control` auto-tuner (`tune.py`, 48 V bus, `/dev/cu.usbmodem101`); converged slightly-overdamped (2 % overshoot, ~9 ms settle) at wc = 169 rad/s. Logs in `current-control/tune_logs/`.

**Plant ID** (tuner drives the A→B series pair; per-phase = half):
- R = 0.646 Ω series → **0.323 Ω/phase**
- L = 673 µH series → **336 µH/phase**
- A→B PI: kp = 0.1135, ki = 109

**Applied to `sensorless.cpp`:**
- `kDefaultKp` 0.0376 → **0.0568** V/A, `kDefaultKi` 60 → **54.5** V/(A·s) (per-phase = A→B ÷ 2, wc = 169 rad/s)
- `kObsR` 0.115 → **0.323** Ω, `kObsL` 94 µH → **336 µH** (per-phase sys-id)

**Decisions / caveats:**
- `kPolePairs` unchanged at 15 — NineBot S is the same 15 pp (30-magnet hub motor) as the previous motor. Confirmed, no code change.
- Bandwidth dropped 400 → 169 rad/s. The tuner picked 169 as the verified-overdamped crossover on the *current-control* loop (5 kHz, 200 µs dt). Sensorless runs at 20 kHz (50 µs) and could support higher bandwidth — the pole-zero ratio `ki/kp = R/L` holds at any wc — but 169 is the value actually validated, so it's the safe starting point. Push wc up later only with re-verification in HOLD (`id 1` → id tracks, iq≈0).
- `kObsR` is best validated against the stage-2a `vd = id·R` slope on the new motor (that's how the old 0.115 was obtained, distinct from sys-id). 0.323 is the sys-id starting value.
- `kObsLambda` **NOT** updated — flux linkage isn't measured by the current tuner. Still the old 0.016 default; must re-measure (spin I/f, read the `flux` column at speed) before trusting the observer on this motor (§7 step 6).
- Side effect: 336 µH is ~3.5× the old 94 µH → much lower current ripple at 20 kHz. The §9 "94 µH ripple" gotcha and the incidental 94 µH / 0.15 Ω references in code comments are superseded for this motor.

---

## 11. High-power dyno testing (2026-07-27 →)

Bench: magnetic particle brake (separate supply) + rotary torque sensor on the test stand, FLIR on the FETs, scope current clamp, 48 V PSU. The board stressor is **phase current** (FET conduction loss ∝ I²·Rdson), which ≈ iq regardless of speed — so the staircase is: hold a speed setpoint, step the *brake* up, watch iq rise to hold it.

**Protocol per step:**
1. Hand off at LOW amps, then raise the ceiling live: `run 3 100 3`, wait for `C`, then `run <amps> 100 3` — re-issuing `run` in `C` retargets without redoing the handoff. (High-amps handoffs converge slowly and used to sawtooth; see §4.)
2. Step the brake; iq rises. Hold to thermal plateau (dT/dt → 0 on all LM75s) or until the `tl` guard ends the run.
3. Log the torque display (`t= <Nm>` in the TUI) and PSU watts (`p= <W>`): `p_w − τω` = motor losses, `PSU − p_w` = board losses.
4. Escalate only between runs, keeping the layer order **target < trip < trip+3 < VDS OCP < ADC rail** (`#range` line): `lim`, then `vl` when |v|/vlim saturates, then `vds` one notch, then `tl`.

**Watch for:** |v|/vlim pinned (loop out of authority — raise `vl` or back off), vd/id creeping up at fixed id (copper heating), accelerating dT/dt at constant power (runaway — the TUI auto-stops at 2 °C/s), imp/rej filter counters growing with temperature (ADC corruption worsening — periodic `s` polls land in events.log), and late-run VDS OCP trips at lower current (Rdson derating — expected physics, not an anomaly).

Every safety response in the chain is the gentle unload → Hi-Z coast; a run that hits any guard ends with the stream intact — that's the point.

---

## 12. Hall-drive dyno bring-up log (2026-07-30) — lessons

Working end of day: `hcal` (bidirectional) → `hrun 5 30` self-starts under load,
settles ~1.5–2 A at 30 rpm. The path there, so nobody relearns it:

1. **Any-edge hall interrupts wedged CPU0 on first wire-up** (noisy line =
   interrupt storm; console dead before the banner, esptool soft-reset dead
   too — flashing only worked in the ~1.5 s post-reset window). Fix: halls are
   **polled in the FOC task** (`HallPoller`), interrupts never enabled. A bad
   line is now a counter (`illegal`), not a dead chip.
2. **Boot failures must never be silent**: the DRV config check now retries
   with a 2 s console message instead of returning into a mute console —
   "board totally dead" cost two long detours before this.
3. **Hall edge bounce → speed-estimate garbage → current pops.** Slow edges on
   the weak internal pull-ups (~45 kΩ) bounce N↔N+1 through the debounce; a
   bounce pair reads as an impossible ±1500 rpm interval and the speed loop
   slams iq to the rail (audible pops, scope spikes). Fixes: 200 µs debounce,
   bounce-undo (reversal within 5 ms), reversals force omega through zero, and
   the hall speed-loop error is clamped ±30 rpm. **Board rev: real 1–4.7 kΩ
   external pull-ups on the hall lines.**
4. **`hcal` must be bidirectional**: the rotor lags the I/f drive by the load
   angle (γ ≈ 25–37° on this rig — measurable drag!), biasing a one-direction
   table. Equal ticks per direction cancel it. Budgets only count while the
   wheel actually rotates (pre-breakaway/hand-start stall time smears a full
   drive sweep into one parked sector), and the table must pass a 60°±25°
   spacing check before it's committed.
5. **The +90° convention (the day's boss fight):** the cal's circular mean is
   the *drive* angle per sector, but under I/f the rotor d-axis aligns with
   the *current vector* = drive+90°. Without adding 90°, the table is exactly
   rotor−90° and commanded "iq" lands on the d-axis: zero torque, perfect
   detent lock. Diagnostic signature worth remembering: **the better the
   calibration got, the harder it locked** — cleaner-but-wronger behavior
   means a reference-frame error, not noise.

**Open items:** (a) the chip **wedges under sustained stall-level DC current**
(three occurrences: boot-enable, `run` align, stalled sine at 5 A) — survives
nothing but a reset button; electrical suspicion, needs a scope session on
3.3 V + hall lines during a deliberate `hiq 5` stall; (b) hall pull-ups (see
3); (c) monitor.py now shows firmware messages in a console pane — use it
instead of a bare terminal so runs keep getting recorded.
