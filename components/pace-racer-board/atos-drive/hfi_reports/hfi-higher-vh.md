# hfi-higher-vh: report (2026-10-23/24)

Branch `alexchunlin/hfi-higher-vh`. The worktree came up on main (17dafad); I reset it to f8743a5 and copied
rammp-rtps, managed_components and dependencies.lock. Build: `idf.py -B build-hvh12-fresh -DATOS_MOTOR=drone
-DATOS_VBUS=12 build` (fresh dir, builds clean at the final commit). Data are in `/tmp/hfi/hvh/`: mode-9 runs `*.npy`,
scope records `scope*.npz`, scans `scan020.json`/`scan025.json`, calibrations `cal020.txt`/`cal025.txt`, and the mode-8
sets `m8_020_e*.npy`.

## Outcome: FAILED (acceptance not met; my firmware fix never ran on hardware)

- Forward `xrun 3 30 1` at 0.20 V held 10 s **once in 4 tries** (trips at 3.0 and 6.7 s, plus one 1.4 s trip with a
  different hrtn). `xrun 3 60 1` tripped at 1.3 s.
- Reverse still passes at 0.20 V (2 of 2 runs, 10 s each).
- The 0.20 V data put every forward loss back at the known stick spot. The data say why: there the measurement follows
  the current vector, so I wrote a fix, zero-current HFI windows (`hrtb`).
- I could not test it. Every image built from this branch rolled back after OTA: the coordinator's known boot panic,
  `esp_intr_alloc` from `FocSampler::set_edge`. Then the board dropped off USB entirely (see Safety / bench state).
  The fix has only an open-loop replay behind it. That replay is optimistic by construction, so treat it as a hint,
  not evidence.

## 1. Merged build validation (f8743a5, unmodified): PASS

`cs_meas.py` (3000-row capdump, all sinks, drone 12 V):

| state | late | ISR max gap | FOC max gap | cavg |
|---|---|---|---|---|
| idle | 8 / 71k (0.01 %) | 95 us | 132 us | 14.6 us |
| HOLD id 0, idle | 7 / 73k (0.01 %) | 100 us | 150 us | 14.7 us |
| HOLD id 0 + capdump | 280 / 193k (**0.14 %**) | 96 us | **149 us** | 14.7 us |

console-stall reported 0.9 % late during the dump, with a FOC gap of 150-156 us. On this build, late is lower and the
FOC gap equals idle.

Regression check: `hrt 0.17` + `xrun 3 -30 1` with the smooth-hfi-drive set **held 10 s**: hat-enc -1.9 / 11.4 / 58.1
(mean / sd / max, deg elec), |I| p50 1.35 A, p95 2.55 A, travel -2898 deg mech.

## 2. Higher Vh: calibration and scope

Encoder-truth scans (`enc_scan.py <Vh> 3 800 15`, no drive current) followed by `hrt_cal.py`:

| Vh | \|P\| | rotor \|Nr\| | saliency | residual rms | Ns re, im / off (`hrt` args) |
|---|---|---|---|---|---|
| 0.17 (previous) | ~2.2-2.4 A | ~0.28 | ~12 % | - | -0.0176 -0.0427 149.2 |
| 0.20 | 2.92 A | 0.337 | 11.5 % | 0.051 | **-0.0313 -0.0782 151.4** |
| 0.25 | 4.00 A | 0.488 | 12.2 % | 0.018 | **-0.0031 -0.0997 163.2** |

At zero current, every detent the scans visited matches the encoder within 1-3 deg (0.25 V within 0-2.7 deg). The
scans only visit one mechanical half-revolution (mech 9-163 deg).

Scope (`scopecap.py`, new tool): peak-detect single shot on phase A, triggered when the injection starts. Caveat: the
MHO984 RAW readout returns only 250k points, so each record covers 2.5 s (hrt start, then the xrun start and about
1.7 s of driving), not the whole 10 s run. All runs were reverse `xrun 3 -30 1` with a 3 A ceiling.

| Vh | phase-A peak / min | p99.9 \|I\| while driving | 1 ms envelope p99 | ramps |
|---|---|---|---|---|
| 0.17 | +4.6 / -7.2 A | 6.5 A | 6.5 A | none (largest 1 ms rise = injection start) |
| 0.20 | +5.3 / -5.1 A | 4.2 A | 4.3 A | none |
| 0.25 | +7.8 / **-9.9 A** | 8.5 A | 8.1 A | none, but 140 samples above 8 A |

**0.25 V with a 3 A ceiling sits at the 10 A limit, so I ran no further drive at 0.25 V.** It would need a lower
ceiling (about 2-2.5 A) or a full-run scope check. 0.20 V has 2x margin and became the operating point. There were no
DRV faults in the whole task.

## 3. 0.20 V drive results (mode 9, 8 ms rows, `hvh.sh` = smooth-hfi-drive set with Vh/cal/hrtn as parameters)

| run | hrtn | held | hat-enc mean / sd / max | \|I\| p50 / p95 | travel (deg mech) |
|---|---|---|---|---|---|
| xrun 3 -30 1 | 0 | **10 s ok** | +4.5 / 8.2 / 46.7 | 0.64 / 2.54 | -3343 |
| xrun 3 30 1 | 0 | 3.87 s, TRIP -71 | -3.4 / 12.0 / 77 | 1.06 / 1.82 | +359 |
| xrun 3 30 1 (cq 0) | 0 | 1.37 s, TRIP -141 | +0.6 / 20.0 / 153 | 1.25 / 2.05 | +340 |
| xrun 3 30 1 (cq 0, "hrtz 5")* | 0 | 1.58 s, TRIP -167 | +3.4 / 17.4 / 172 | 0.86 / 1.90 | +182 |
| xrun 3 -30 1 (old hrtn, 0.17 V fit) | 0.17 fit | 3-4 s, TRIP 142 (scope run) | - | - | - |
| **xrun 3 30 1** | 0.17 fit x1.2 | **10 s ok** | +6.9 / 23.7 / 58.1 | 1.30 / 2.75 | **+3421** |
| xrun 3 -30 1 | x1.2 | **10 s ok** | +5.5 / 9.5 / 52.0 | 0.84 / 2.67 | -4864 |
| xrun 3 30 1 #2 | x1.2 | 3.04 s, TRIP 170 | +16.1 / 16.5 / 169 | 0.13 / 1.62 | -209 |
| xrun 3 30 1 #3 | x1.2 | 6.70 s, TRIP 176 | -1.0 / 21.3 / 177 | 0.90 / 2.78 | +2703 |
| xrun 3 60 1 | x1.2 | 1.34 s, TRIP -172 | +3.1 / 28.7 / 171 | 0.97 / 2.58 | +180 |

\* The `hrtz` row ran on the merged image. The OTA of my build had silently rolled back (section 5), so the gate was
not active and the console answered "unknown command". The flash tool reported "Update complete", and I found the
rollback only afterwards.

hrtn x1.2 = `hrtn -0.0054 0.0485 -0.0484 0.0493`: the 0.17 V K scaled by Vh, since the rotor term scaled by about
1.2-1.27x. A direct refit on encoder-driven 0.20 V data (`hrt_nfit2.py` on `m8_020_e±30`) gave a near-zero K, because
those runs carry little current (|If| p50 0.85 A). Raw error there was -2.9/9.8 (fwd) and +1.7/10.4 (rev).

## 4. The forward stick spot: what the data show

Of the forward failures at 0.20 V, 3 of 4 (all three with hrtn x1.2) end at enc 175-178 deg elec, the same spot as at
0.17 V. The rotor is stuck (encoder still) for 0.3-0.8 s under |I| ~1.45 A. theta_hat leads by +40..+50 deg, and
omega_hat sits at +30..+50 rad/s even though the rotor is still. The current lands at rotor-frame angle ~135 deg
(id -1.0, iq +1.05). Then the branch flips: theta_hat - meas reaches 90 deg and the 2-theta error wraps.

Pooled stuck samples (|w_enc| < 3 rad/s, |I| > 1 A), meas-enc circular mean / sd:

| spot / current angle (rotor frame) | 0.17 V | 0.20 V |
|---|---|---|
| enc 165-190, 60-90 deg (theta_hat lags) | -10.7 / 8.7 | -19.3 / 9.6 |
| enc 165-190, 120-150 deg (theta_hat leads) | -31.6 / **47.4** | **+27.6 / 26.0** |
| elsewhere, 60-90 | -10.5 / 8.5 | -6.4 / 9.1 |
| elsewhere, 90-120 | +5.5 / 7.0 | +8.0 / 7.8 |

- **Higher Vh helped the noise but not the cause.** At the spot with -d current, the scatter drops from 47 to 26 deg.
  But the measurement now follows theta_hat's error: it reads about +28 when theta_hat leads by 30-60 deg and about -19
  when it lags. That is a local gain of about 0.8 of positive feedback, so only about 20 % of the HFI restoring
  stiffness is left there.
- **The model then carries theta_hat.** It evaluates the cogging term at a theta_hat that is 40 deg off, which is 240
  deg wrong in 6-theta phase. That term predicts about +5000 rad/s^2 forward, where the true cogging at 177 deg is about
  -2300. It is what keeps omega_hat at +38 while the rotor is stuck.
- **The xsl loop then pulls torque back.** Its kd term brakes on that false omega_hat, and cff at the wrong angle
  removes more torque. The rotor stays stuck until the flip.
- **The rocking-spot noise is local.** A passive quadrant map (`qmap.py`, encoder drive, 0.20 V) at the 350/355 deg
  detent shows the same Q2 current (id -1, iq +0.9) reading -2.6 / 2.3 deg there: clean. Unknown: whether the spot
  depends on mechanical position. Alignment never landed on the 173 deg detent, and mode-9 has no mechanical angle.

## 5. Firmware changes (commits on `alexchunlin/hfi-higher-vh`; all new knobs default off)

- `3cf35dc` `hrtz ge_deg [tau_ms]` is a model gate. When the observer's model acceleration pushes against the
  low-passed PLL error, it is scaled by max(0, 1 - |e_lp|/ge). The `hrt` status gains `zgated=<gated>/<armed>` ticks.
  `xsl ... [eback]` is an asymmetric reference clamp: how far the rotor may run ahead before the reference is dragged,
  aimed at the 1.7x overspeed. Bench tools: `hvh.sh`/`hvhset.sh` (smooth-hfi-drive set with VH/CAL/HN/EXTRA),
  `scopecap.py`, `qmap.py`, `hrt_nfit2.py`.
- `ef94d8e` `hrtb per_ms wid_ms settle_ms [boost] [cout] [wmax]` adds zero-current HFI windows, in the HFI drive
  (HALL, theta source 2) only. Every per_ms, the iq and id commands go to 0 for wid_ms. After settle_ms the PLL error is
  weighted by `boost`. Outside that clean part it is weighted by `cout` x the i0 confidence. With wmax > 0 the windows
  run only below that |omega_hat|.
- Final commit: replay, acceptance and viewer tools (`hvh_replay.py`, `hvh_accept.sh`, `tail9.py`).

**None of the firmware changes has run on hardware.** Images from this branch (`/tmp/hfi/hvh/hvh_v1.bin`,
`hvh_v2.bin` = ef94d8e, `hvh_v3.bin/.elf` = ef94d8e with a 6 KB FOC stack) roll back after OTA. Per the coordinator,
this is the latent boot panic in `esp_intr_alloc` from `FocSampler::set_edge`. The device reported `core_dump: true`
afterwards. Bumping the FOC stack did not help. I built two .bss-padded variants (`hvh_p1024.bin`, `hvh_p4096.bin`)
but never flashed them: the board was gone by then. The padding is not in the branch.

The OTA tool hides the rollback: it prints "Update complete; the running image is already confirmed" with the old
version. Always check a new command after flashing.

### Offline replay (open loop, `hvh_replay.py`)

The replay runs the observer on captured meas and currents. hrtb windows are synthetic: iq is 0 inside, and the
measurement is the encoder plus 3 deg noise, which is optimistic. The replay base does NOT reproduce even the
successful capture (sd 85 vs 23.6 captured), because meas is held for 8 ms. So the replay can only rank the options:

| file | replay base | hrtz 5 20 | hrtb 20 3 1.5 3 0.5 | hrtb 10 3 1.5 3 0.5 |
|---|---|---|---|---|
| V020_f30_k12_2 (trip) | sd 95, >60 at 1.9 s | sd 71, 1.9 s | sd 12, max 83 | **sd 8.0, max 55, no >60** |
| V020_f30_k12_3 (trip) | sd 59, 3.2 s | sd 60, 3.2 s | sd 15, 3.3 s | **sd 8.7, max 43** |
| V020_f60_k12 (trip) | sd 57, 1.1 s | sd 55, 1.1 s | sd 14, max 57 | **sd 8.5, max 37** |
| V020_f30_k12 (held) | sd 85, 1.2 s | sd 96 | sd 34 | **sd 10.7, max 59** |
| ACC30_3 (0.17 V trip) | sd 46, 2.8 s | sd 75 | sd 47 | sd 9.2, max 61 at 5.8 s |

- **hrtz alone does nothing useful** in replay; drop it unless the bench shows otherwise.
- **10 ms windows are what keep theta_hat in range** in replay, but they zero the torque 30 % of the time. Expect
  higher |I|, or stalls at the high-friction spot; this must be tried on hardware.

## 6. To finish acceptance once a booting image is on USB

1. Flash this branch (or the boot-panic fix merged into it). Verify `hrtb` answers on the console.
2. Regression at 0.17 V: `tools/hfi_bench/hvh.sh R017 'xrun 3 -30 1'` (defaults = smooth-hfi-drive set).
3. Scope check at 0.20 V with windows on (current steps every window):
   `source hvhset.sh` with `VH=0.20 CAL="$(cat /tmp/hfi/hvh/cal020.txt)" HN="hrtn -0.0054 0.0485 -0.0484 0.0493"
   EXTRA="hrtb 10 3 1.5 3 0.5"`, then `scopecap.py 3 1 hvh/scope020_hrtb "${SET[@]}" 'xrun 3 30 1@10.3'`.
   Require a peak <= 10 A and no multi-ms ramps.
4. Acceptance: `tools/hfi_bench/hvh_accept.sh` runs 3x `xrun 3 30 1`, then `xrun 3 60 1`, `xrun 3 -30 1` and
   `xrun 3 -60 1` at 0.20 V, hrtn x1.2, `hrtb 10 3 1.5 3 0.5`, hguard 60. Each prints hat-enc mean/sd/max, |I|
   p50/p95, travel and the guard result. If the windows stall the rotor, try `hrtb 20 3 1.5 3 0.5`, a 2 ms width, or
   `wmax 30`.
5. If it still fails at the spot: map the current-angle dependence at the 173 deg detent (`qmap.py`, after getting
   the rotor there) to decide between a spot-specific correction and a polarity/position-dependent model.

## Safety / bench state

- No DRV faults. Every session ended with `stop`, en=1, PSU 6+6 V on, and all new knobs at default on the board.
- **Lock-rule slip (no board contact).** At 00:18 a background job of mine ran `espp_ota flash` and `con.py` after its
  lock acquire had timed out: the script chained with `;` instead of `&&`. It reached nothing, because the board was
  not on USB. I killed it, fixed the scripts and reported it to the coordinator.
- **Board absent from USB at 00:23.** When I got the lock, the PACE board (1209:0d41) was not enumerated. The PSU was
  on at 6 V, 78 mA. One VM cycle did not bring it back, so I released the lock and escalated. It needs a physical check
  (cable, or the boot-panic work leaving it in download mode).
