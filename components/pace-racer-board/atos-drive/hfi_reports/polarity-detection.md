# polarity-detection: encoder-free polarity test for hrt (report)

Branch `alexchunlin/hfi-polarity-detection` (worktree `~/orca/workspaces/pace-racer-fw/hfi-polarity-detection`),
based on f764371. **Acceptance NOT met**: ≤ 3 A gives ~81–85 % correct (target ≥ 95 %). The
hpol-seeded `xiq 1` start held 8 of 12 times; the other 4 were wrong-polarity starts that hguard
stopped cleanly.

## Result in one paragraph

On this motor, at the allowed currents, the saturation polarity signal is only 1–2.5 % of the HF
admittance. The inverter puts a stator-fixed term of up to ±4 % on top of it: dead-time knee,
current-sensor offset, and an unexplained 3rd harmonic in the frame angle. Run-to-run noise is
2–3 % sd. The best method (biased pulsating d injection, Y metric) gets **66/78 = 84.6 %** right
uncalibrated over 6 detents × 2 approach directions × 2 runs, or **42/52 = 80.8 %** counting only
decisions where the rotor moved < 2° mech. Adding an encoder-fitted stator table and
cross-validating gives **82.7 % leave-one-detent-out** and **78.8 % on the approach split**, so the
table doesn't help out of sample. Short 4 A pulses (option A from the coordinator) passed the scope
check. The 4 A scan then latched DRV fault **0x0628**, and I dropped everything above 3 A as
instructed. Its 24 decisions were 54 % correct.

## What hpol does (firmware, `main/atos_drive.cpp`)

- `hseed enc|pol|none`: how `hrt` picks the 180° branch at its first estimate. `enc` is the
  default and the old behaviour. With `pol`, the polarity test runs automatically at the first
  estimate (HOLD only), and `xrun`/`xiq` stay locked until it decides. `none` takes whatever
  atan2 gives.
- `hpol [Vp] [bias] [ms] [blocks] [div] [ysign] [metric] [psi]`: run the test now (`hrt` running,
  HOLD, standstill). `hpol` alone prints the last result.
  - **Sequence:** the rotating injection pauses and theta_hat is frozen. Along it, i_d is held at
    +b, then −b, in ABBA order over the blocks, with a small pulsating voltage `Vp cos(phi)` on
    top. The HF is 1.25/2.5/5 kHz for div 16/8/4; 2.5 kHz works best. iq stays 0.
  - **Y:** `|I_hf(+b)| / |I_hf(−b)| − 1`, in %. When +b points at north, the incremental Ld drops
    and Y > 0.
  - **K:** the 2nd-harmonic ratio `k = I2 conj(I1)^2 / |I1|^3`, projected on psi (reported only).
  - **Decision:** `ysign·(Y − Ystat(theta))`; if negative, theta_hat += 180°.
  - **Printout:** margin %, SNR across blocks, Y, K, k, I1, peak |id|, encoder movement during
    the test (deg elec), th0, hat, enc, and err vs the encoder.
- `hpolt <gap_ms> <settle_ticks> <ramp_ticks>`: segment timing. gap 0 is continuous mode. With
  gap > 0, the test runs as pulses from 0 A with a ramp up and a ramp down. Above 3 A the console
  enforces the coordinator's limits: pulse (settle + measure) ≤ 3 ms, gap ≥ 20 ms, train
  ≤ 100 ms, bias ≤ 5.5 A. The FOC side aborts on any d sample above bias + 2.5 A.
- `hpolcal c1 s1 c3 s3`: stator-fixed Y table, subtracted before the decision (option B). See the
  build problem below.
- The `hrt` filters and PLL freeze during the test and for 20 ms afterwards; `hrt` restarts cleanly.

The defaults at HEAD are the B settings: Vp 0.03 V, b 2.0 A, 60 ms × 6 blocks, div 8, ysign +1.

## What was tried and measured (12 V bench, `hrt 0.17 -0.0176 -0.0427 149.2`)

| # | Method | Result |
|---|---|---|
| v1 | ±Id pulses (2–3 A) under the rotating `hrt` injection, compare \|P\| and the rotor N | Per-cycle scatter ±10 % of \|P\|; \|P\| moves 10 % between 2 and 3 A bias (dead time); polarity term ≤ 3 %. Abandoned. |
| v2 | Zero-mean pulsating d injection at 1.25 kHz, 2nd harmonic (no bias) | k 1–2 %, but at the angle of the DC-offset × dead-time artefact my model predicts (−27…−40°), not the magnetic one (±142°/37°). i_d crosses zero. |
| v3 | Bias ±1.5–2 A plus pulsating 1.25 kHz | Y ±1–4 %, inconsistent between sessions: the impedance is R-dominated there (ωL 24 mΩ vs R 36 mΩ). 5 kHz (div 4) is garbage (aliases with one-phase-per-period sampling). |
| v3 at 2.5 kHz | Bias ±2.2 A, Vp 0.03 (I1 0.6 A), 6 × 60 ms | **Best: see B below.** K had no usable magnet term (46/78). |
| A | 4 A pulses: 3 ms (0.5 ms ramp + 1 ms settle + 1.5 ms measure), 20 ms gaps, 4 pulses = 72 ms | Scope: clean 0.5 ms ramp, flat plateau, no ringing. Plateau 4.9 A on the clamp; the clamp reads 1.20× the board (static `id 3` = 3.61 A on the scope), so 4.08 A in board units. Raw peak 7.15 A on the clamp = 5.96 A board ≈ 1.49× command, all PWM ripple (static 3 A already shows a 1.61× raw peak). Scan: 13/24 = 54 % (3 detents); rotor slipped 30° mech once; **DRV fault 0x0628** latched later in the scan, and many conditions after X = 105 got no result. Recovered by VM cycle; boot faults all 0x0000. Escalation to 6 A was not done. |
| B | Same as v3 at 2.5 kHz, ≤ 3 A, both approach directions | Table below |

**B scan** (`pol_scan.py ... '0.03 2.2 60 6 8 1 0 37' 15 out.json 2 3.0 1 'hpolt 0 208 100'`,
data `/tmp/hfi/pol_scanB1.json`, log `/tmp/hfi/pol_scanB1.log`):

| Detent (enc deg elec) | Correct | Signed margin median / min |
|---|---|---|
| 350 | 8/16 | −0.18 / −6.1 % |
| 51 | 6/6 | +0.88 / +0.11 % |
| 111 | 18/18 | +4.2 / +0.23 % |
| 167 | 15/16 | +4.4 / −7.8 % |
| 232 | 8/8 | +3.1 / +0.32 % |
| 300 | 11/14 | +1.8 / −10.4 % |
| **All** | **66/78 = 84.6 %** | p5 −5.2, p25 +0.7, median +3.0 %; \|SNR\| median 6.5 |

- **Movement:** median 1.9° elec (≈ 1° mech). 26 of 78 decisions moved more than 2° mech, and
  several flipped the rotor a full pole (180° elec): −b on north above the ~2 A cogging is an
  unstable push. Counting only decisions with move < 2° mech: 42/52 = 80.8 %.
- **Stator fit** (`pol_analyze.py`): Y = m·s + c1 cos θ + s1 sin θ + c3 cos 3θ + s3 sin 3θ.
  m = +1.9 % (textbook sign: north gives larger HF current at +b), stator coefficients
  −4.2 / +1.4 / −1.3 / −4.2 %, residual sd 2.0 %. Cross-validated: leave-one-detent-out 43/52
  (82.7 %), approach split 41/52 (78.8 %).

**hpol-seeded start, then `xiq 1` with hguard 60** (`pol_xiq.py 0.17 ... 1 2 <12 positions>`, log
`/tmp/hfi/pol_xiq2.log`; b 2.0, Vp 0.03, 6 × 60 ms, div 8, no stator table):

- **8/12 correct:** no trip over 2 s; the rotor crept 2–5° mech under the 1 A torque.
- **4/12 wrong:** hguard tripped cleanly (last_trip_err ≈ ±175°), no DRV fault. All 4 were on
  the 167/350° detent axis, the weak one in the scan.
- Margins of the wrong ones: +0.13, +0.43, +1.38, −1.34 %. The right ones were mostly |3–14| %.
  A margin threshold (refuse to seed below ~2 %) would turn most wrong starts into "undecided";
  not implemented.

## Problems found on the bench (for the coordinator)

1. **The encoder offset got lost twice.** `enc` showed `cal=0 ofs=0` after other agents' flashes
   or the VM cycle. I restored it each time with `eofs 316.35` ("saved"); `th 0` then aligns to
   350.6° as before. The 4 A scan ran with ofs = 0; I re-scored it offline with +316.35°. Any
   guard test run while `cal=0` gives false trips.
2. **HEAD of this branch panics at boot (rolled back by OTA). The cause looks pre-existing,
   not hpol.** The core dump, read over the WebUSB CoreDumpService (module 4, GET_SUMMARY;
   `/tmp/hfi/coredump_summary.py`), shows a PANIC in task `main`:
   `esp_intr_alloc_intrstatus` -> `find_desc_for_source` (intr_alloc.c:190, walking the vector
   descriptor list hits a corrupted node) <- `mcpwm_timer_register_event_callbacks`
   <- `FocSampler::set_edge` (foc_sampler.hpp:193) <- `app_main` (atos_drive.cpp:2033).
   - None of the hpol code runs before that point: the FOC task idles until the sampler notifies,
     and hpol only allocates statics. The trigger is the last commit (`hpolcal`: 4 atomics plus a
     few lines). Its predecessor 0495597, built the same way, boots, so this looks like heap
     corruption earlier in boot that the new memory layout exposes.
   - Padding the image (to rule out a 512-byte OTA edge case) did not help.
   - Recommendation: build the merged tree with `CONFIG_HEAP_POISONING_COMPREHENSIVE` and check
     the reset reason with `tools/usb_reset.py --info` after every flash. The OTA tool's
     "running image is already confirmed" line can mean it silently rolled back.
   - The firmware tested last on the bench (A/B scans and the `xiq` test) is 0495597 plus
     runtime settings: everything at HEAD except `hpolcal` and the new defaults.
3. **DRV fault 0x0628** during the 4 A pulse scan; cause not analysed. The scope showed no
   overshoot on the pulses I captured. One rotor slip of 30° mech happened shortly before the
   failures began.

## What changed (commits on the branch)

- `main/atos_drive.cpp` (HEAD panics at boot on this layout, see problem 2; 0495597 boots and
  was the firmware tested): `hseed`, `hpol`, `hpolt`, `hpolcal`; hooks in the `hrt` block (filter/PLL
  freeze, seed branch selection) and a self-contained polarity section after the `hrt` injection.
- `tools/hfi_bench/pol_scan.py`: position scan with approach directions and per-detent stats.
- `tools/hfi_bench/pol_xiq.py`: hpol-seeded start + `xiq` + guard check.
- `tools/hfi_bench/pol_analyze.py`: accuracy, stator fit, cross-validation.
- Notes from v1/v2/v3: `/tmp/hfi/reports/polarity-detection.dcbias.txt`.

## Still open

- Polarity ≥ 95 % needs a larger magnet signal than ≤ 3 A gives on this motor, or a different
  principle:
  - **(a)** Larger d pulses (5–6 A), after the 0x0628 trip is understood.
  - **(b)** Micro-motion: a small ±iq torque pulse, read the deflection direction from `hrt`.
    This moves the rotor, against the task rule.
  - **(c)** Accept ~85 %, add a margin threshold, and keep hguard (or the first-motion check)
    as the safety net.
- Characterise the stator-fixed Y term: the 3rd harmonic suggests dead-time knees in the phase
  currents, which a larger bias or dead-time feed-forward could shrink.
- At b > ~2 A, −b on north can flip the rotor; lower bias costs signal.
