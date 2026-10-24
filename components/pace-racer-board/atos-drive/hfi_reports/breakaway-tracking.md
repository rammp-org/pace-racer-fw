# breakaway-tracking: report (2026-10-23)

Branch `alexchunlin/hfi-breakaway-tracking` (worktree reset to f764371 first; it had been created at main).
Commits 73f2887..ad33bff, all in `components/pace-racer-board/atos-drive` (`main/atos_drive.cpp`, `HFI_HANDOFF.md`, `tools/hfi_bench/`).
Raw data: `/tmp/hfi/reports/breakaway-tracking-data/` (mode 9 = [enc, meas, hat], [ifa, ifb, w_hat]; mode 8 = N/P/If).

## Outcome: ACCEPTANCE NOT MET (documented negative result)

Best settings (`tools/hfi_bench/xaccept.sh`): `hrtg 600 100000 0.01 0 0.2 20`, `hrtm 4000 8 0 1300 1.5`, `hrtk -3000 -3100`,
`hrtn -0.0046 0.0412 -0.0484 0.0493`, `hrtc 0 0 0`, `sg 0.02 0.5`, `hrt 0.17 -0.0176 -0.0427 149.2`, hguard 60.

| test | result | hat-enc (deg elec) |
|---|---|---|
| xrun 3 30 1, baseline (plain PLL, default sg) | guard trip at 0.29 s | sd 33, max 133 |
| xrun 3 30 1, best (4 runs) | trips at 9.49 / 1.89 / 0.86 / 0.33 s | 9.5 s run: mean +8.8, sd 8.5 until the trip |
| xrun 3 -30 1, best | **held 11 s, no trip** | mean +2.7, sd 9.7, max 45 (only 100 deg mech of travel: mostly stick/rock) |
| xrun 3 60 1, best | trip at 1.56 s | sd 20.5 |
| xiq 2.5 (governed 100 / 200 rpm) | about 0.35 s at 2.5 A, then trip | sd 17-19 |
| xiq 2.5 ungoverned (model only) | tracked breakaway + spin-up to ~650 rad/s elec (hat-enc +20..27), lost lock at ~750 rad/s, DRV 0x0624 | - |
| xiq 1 (model + comp) | held (several 0.5 s holds) | +1..+14 |

## What was found

1. **The PLL was not the bottleneck.** At the 2.5 A step the *raw* HFI angle jumped -28 deg before the rotor moved (C1). hat followed it, the true-frame current gained +d, and the measurement shifted about 1.7 deg per deg of hat error, so the branch flipped within 12 deg of travel. Under encoder FOC (true current on q), the raw measurement stays within +/-35 deg (mean -10, sd 15) at +/-150 rad/s (A2).
2. **Drive-current saliency** (mode-8 fit, `hrt_nfit.py`): w/|P| = e^{j2th} A (1 + b|If|^2) + K If^2, with K = -0.0046+0.0412j, b = -0.0484+0.0493j /A^2, |A| = 0.28. At 2.5 A the current term (0.26) is larger than the rotor term (0.21). Correcting it without theta (`hrtn`) moves load-offset means from -23.5/-20.3/+36.6 to -2.1/+1.0/-4.6 deg (e30/em30/I-f; the I-f set was not in the fit). The sd under motion is still 9-26 deg. This is the "-21 deg offset whenever torque current flows": it is sign-symmetric because it goes as If^2.
3. **Mechanical model** (A2, erun 2.5 A, R^2 0.92): Kt/J = 4000 rad/s^2/A (elec), B/J ~8 1/s, Coulomb 1300 rad/s^2 (~0.33 A), cogging -3000 cos6th - 3100 sin6th (4300 rad/s^2, ~1.1 A). The predicted detents (-7 + k*60 deg elec) match every rest angle seen. Breakaway is about 1.4 A. With the model, omega_hat follows the snap-off (79 vs 105 rad/s), and replay of the breakaway captures holds lock where the plain PLL flips.
4. **What still fails:** every remaining trip is a breakaway or a +/-80 rad/s rocking event in a detent with |I| > ~2 A. In those events the residual current-saliency error and the 6-theta cogging term (wrong once hat is 20 deg off) let hat walk away. The confidence scaling (i0 1.5) helps: 0.3 s became up to 9.5 s at 30 rpm, but the result is not reliable.
5. **Static map** (B3, encoder FOC, rotor in detent): |iq| <= 1.1 A gives offsets within the +/-7 noise; id +2 A gives +7, id -2 A gives +20..+48.
6. **FOC CPU / console stall (safety-relevant):** espp `fast_sin` uses double `round()`. Adding per-tick trig to the tracker made console replies lag 1.5-3 s, so commands such as `stop`, `cap` and `erun` landed late, and the first erun ended in DRV 0x0609. Fixed: trig runs once per injection cycle, and the legacy pulsating demod (which ran every tick whenever hrt was on) is gated off. Latency is back to ~55 ms and cavg is 13.7 us. Note that even on f764371 the console lagged in HALL+hrt (prior logs show replies arriving one command late).

## What changed (console)

- `hrtm ktj bj l3 [coul] [i0]`: observer feed-forward Kt/J*iq_hat - B/J*w - coul*sat(w) + cogging + l3 integrator, plus HFI confidence 1/(1+|If|^2/i0^2). `hrtk ca cb`: cogging. All 0 = the old PLL.
- `hrtn kr ki br bi`: drive-current saliency correction. `hrtc c_abs c_q2 c_d`: load-offset comp in theta_hat-frame currents (superseded by hrtn; leave at 0).
- `xiq <A> [vmax_rpm]`: speed governor. With it, the HALL ceiling is |A| (it was the board max: the governor braked with -6.7 A). `hid <A>`: HALL d-current for offset maps (reset by stop).
- Capture mode 9. `hrt` status adds `errmax`. `s` adds `cavg`.
- Tools: `otest.py` (mode 5/9 runner with stats, trims the frozen tail), `xaccept.sh`, `xtest8.py`, `hrt_nfit.py`, `hrt_sim8.py`, `hrt_sim9.py`, `cap9_show.py`.

## Safety notes

Three DRV trips this session, each recovered with a VM cycle + usb_reset; the bench was left stopped, en=1, PSU 6+6 V on:
- 0x0609: console stall from my per-tick trig (fixed).
- 0x0624: ungoverned xiq 2.5 runaway.
- One at eiq 0.9 + hid -2: ungoverned eiq runaway to thousands of rpm.

Do not run eiq/xiq above breakaway without a governor on this free rotor.

## Open

- A static saliency map at 2-3 A in arbitrary current directions (needs a locked rotor or a position loop) to refine K/b. Possibly a rotor-frame (theta_hat) term.
- A speed loop for xrun that does not bang the 3 A ceiling. HFI rpm noise (+/-20 rad/s at standstill) makes kp > 0.02 A/rpm bang-bang, and softer gains stick in the detent.
- Cogging feed-forward into the torque command (smooth motion instead of rocking).
- A higher Vh relative to drive current (rules cap Vh at 0.17 V with current).
- The polarity test is still missing: the branch is seeded from the encoder.

---

## smooth-hfi-drive (2026-10-23, same branch, commits after ad33bff)

### Outcome: ACCEPTANCE NOT MET (documented negative result; reverse direction passes)

Forward `xrun 3 30 1` / `xrun 3 60 1` still trip the guard. The same settings run **reverse** (`xrun 3 -30 1` x3, `xrun 3 -60 1` x1) for the full 10 s with real motion (8.5-11 rev) and no trip.
I never got |I| p95 below ~2.5 A. The plant itself needs about 1 A mean (running friction) plus about 1.2 A of cogging peak to turn at 30 rpm.

Best settings (`tools/hfi_bench/xshd.sh` defaults): `sg 0.02 0.5`, `hguard 60`, `hrtg 600 100000 0.01 0 0.2 20`,
`hrtm 4000 8 0 4000 1.5 4000`, `hrtk -3600 -3500`, `hrtc 0 0 0 6`, `hrtn -0.0046 0.0412 -0.0484 0.0493`, `cff 1 1`,
`xsl 0.6 0.015 4 0 30 1`, `hrt 0.17 -0.0176 -0.0427 149.2`. Default xrun governor: 200 rpm.

Acceptance runs (mode 9, 8 ms rows; hat-enc in deg elec; |I| = stationary fundamental; travel in deg mech):

| run | held | hat-enc mean / sd / max | \|I\| p50 / p95 | travel | note |
|---|---|---|---|---|---|
| xrun 3 30 1 #1 | 3.32 s, TRIP | +5.3 / 21.6 / 171 (to the trip) | 1.08 / 2.56 | +692 | |
| xrun 3 30 1 #2 | 5.88 s, TRIP | -4.5 / 19.3 / 156 | 1.23 / 2.78 | +2370 | |
| xrun 3 30 1 #3 | 9.28 s, TRIP | -2.4 / 17.7 / 175 | 1.15 / 2.76 | +3032 | |
| xrun 3 60 1 | 4.54 s, TRIP | -2.9 / 23.6 / 164 | 1.41 / 2.84 | +2882 | |
| xrun 3 -30 1 (Vr) | **10 s ok** | -1.5 / 11.8 / 46 | 1.02 / 2.38 | -3121 | |
| xrun 3 -30 1 #1 | **10 s ok** | -1.1 / 11.8 / 42 | 1.15 / 2.57 | -3251 | errmax 47.7 |
| xrun 3 -30 1 #2 | **10 s ok** | -5.0 / 13.5 / 46 | 1.69 / 2.62 | -3067 | |
| xrun 3 -60 1 | **10 s ok** | -6.2 / 13.7 / 44 | 1.53 / 2.57 | -3989 | errmax 51.8 |
| xrun 3 30 1, previous best (no FF, sg PI) | 10 s ok | -6.3 / 7.5 / 25 | 0.35 / 1.54 | **+25** | rotor stuck in a detent: not a pass |

Speed: the runs average about 1.7x the set-point (for example -30 rpm runs at about -52 rpm). The xsl reference is clamped to +/-1 rad of the rotor, so every forward snap-off drags the reference along.

### What changed (console, all off by default; `atos_drive.cpp` HALL branch + hrt observer only)

- `cff <cog> [coul]`: torque feed-forward from the hrtm/hrtk model. iq += -cog * a_cog(theta)/(Kt/J), computed once per 8 ticks (no per-tick trig). In xsl speed mode it also adds coul * Coulomb/(Kt/J) in the set-point direction. It uses theta_hat on HFI and the encoder on erun. **Note: cff divides by the hrtm Kt/J, so changing Kt/J rescales the feed-forward.**
- `xsl kp kd ki [rate] [tau_ms] [emax]`: position-tracking speed loop for erun/xrun, replacing the rpm PI when kp > 0. The ramped set-point is integrated into a reference angle; iq = kp(ref - theta) + kd(w_set - w_lpf) + ki*integral. The integrator is clamped to +/-0.5 of the ceiling, the reference is held within emax of the rotor, and there is an optional iq rate limit (feed-forward excluded). w = omega_hat on HFI, d theta/dt on the encoder (enc_rpm_f lags 50 ms).
- `hrtc ... [c_q]`: signed theta_hat-frame iq offset term (deg/A). Fit: +5..+8 deg/A residual with hrtn on, both directions, 15-60 rpm.
- `hrtm ... [stic]`: Karnopp static friction in the observer model. Below 5 rad/s it cancels up to stic of the drive acceleration, so a rotor stuck in a detent is not predicted to accelerate.
- `xrun <A> <rpm> [ramp] [vmax]`: the governor now defaults to max(4|rpm|, 200) rpm (it used to inherit whatever the last xiq/eiq had left). xrun/xiq also reset `last_trip_err`.
- Tools: `xshd.py` (otest-style runner: guard result, hat-enc and |I| p50/p95 over the driven window, encoder speed ripple) and `xshd.sh` (best set). Offline scripts are in `/tmp/hfi/reports/smooth-hfi-drive-data/` (`csim.py` closed-loop sim, `fit.py` friction/cogging fit, `err.py`/`err2.py` residual fits, `tail.py`/`ts.py` viewers, `replay.py`).

### What was found

1. **Cogging FF works on the encoder.** With `erun 3 30 1`, rpm PI to FF + xsl: speed sd 33.7 to 14.4 rad/s elec, and the backward rocking is gone (w min -95 to -10). The |I| p95 stays at about 2.5 A.
2. **The friction model was off by 3x.** An encoder fit of ktj*iq - acc over 30 rpm runs (E2/E3) gives running friction of about 4400 rad/s^2 (about 1.1 A at Kt/J 4000), not 1300. Cogging is -3670 cos6th - 3500 sin6th (same phase as before, 18 % larger). Friction varies 0.2-1.7 A along the mechanical revolution (0.5 s bins), which is not observable sensorlessly (HFI sees only the electrical angle mod 180).
3. **Passive HFI under a perfect (encoder) smooth drive** (P_* sets, hrt observing only): hat-enc sd 13-17 deg, with brief excursions >60 deg at 1.3-4.3 s, and P_-30/P_-30b lost the branch. The residual meas-enc is +5..+8 deg per A of signed rotor-frame iq (corr 0.4-0.65, sharp at lag 0) plus about +/-4.5 deg by direction. That is the `c_q` term. The pooled fit only takes sd from 13.6 to 11.9 deg. One loss was a ~30 ms burst of outliers (meas jumped ~100 deg) while |iq| rose through 1.2-1.6 A.
4. **Speed term on omega_hat limit-cycles.** Without stiction in the model, omega_hat swings +/-120 rad/s while the rotor moves +/-20: the model accelerates a rotor that is stuck in a detent. kd*omega_hat (~1.8 A) then drives a ~25 Hz iq limit cycle (X_* runs). `stic` 4000 plus tau 30 ms did NOT clearly fix the omega_hat error: |w_hat - w_enc| p95 is 63 rad/s (X run, old model), 60-62 (final forward ACC30_2/3) and 40 (reverse ACCm30_1). The obvious 25 Hz chatter is gone from the final runs, but omega_hat is still a poor speed signal at these speeds.
5. **The forward/reverse asymmetry has a single location.** With the final config, 11 of 12 forward losses start at enc 174-188 deg elec (just past the detent at 173 deg), with theta_hat already +26..+30 deg ahead. The rotor sticks there under 1.1-1.3 A (a high-friction spot), the measurement reads +20..+46 deg ahead, and omega_hat creeps up past the 5 rad/s stiction band. The model then carries theta_hat forward, and the drive current follows theta_hat. Passive moving data shows no such bias at that angle (-5.5 deg), so it is a stuck-with-current effect. Reverse never stalls there long enough. The previous agent's static map (id -2 A gives +20..+48, id +2 A gives +7) suggests positive feedback once theta_hat leads.
6. **Things tried that did not fix the forward case:**
   - partial FF (cff 0.6-0.8)
   - softer and stiffer xsl gains, kd 0, tau 5/30 ms
   - plain PLL (hrtm 0): trips at 0.7 s
   - i0 1.0 / 3.0
   - hrtg 300 30000
   - stic 6000/8000: worse, trips at 1.1-1.9 s with theta_hat lagging the snap-off
   - hid 0.3/0.5/0.7/1.0: 0.5 gave 7.5 / 6.0 / 0.8 s; 1.0 sticks in the detent
   - ceiling 2 A: stuck, no motion
7. Item (3) of the task, a static saliency map at 2-3 A under an encoder position loop, was **not done**. A held rotor only draws its holding torque (cogging, at most about 1.2 A of iq), so a 2-3 A q map needs a locked rotor or an opposing load. id maps exist from the previous agent (B3).

### Safety

No DRV trips in three sessions (15, 12 and 2 min). The bench was left stopped, en=1, PSU untouched (6+6 V on), all new knobs reset to 0 on the board (cff 0 0, xsl 0, hrtc 0 0 0 0, hrtm stic 0). The guard stopped every forward loss cleanly.

### Next

- Keep theta_hat from creeping while the rotor is stuck. Possibly gate the model with HFI-confirmed motion (for example, freeze the model acceleration while the xsl integrator is rising and |meas - hat| is growing), or use a speed-dependent i0.
- A locked-rotor (iq, theta) saliency map at 1-3 A around 170-190 deg elec to see whether the +30 deg there is a mechanical-position effect.
- An asymmetric xsl reference clamp, so forward snaps are not banked (this fixes the 1.7x overspeed).
- A higher Vh-to-current ratio, if the rules allow it (rotor term 0.28 |P| vs the 1-2.5 A that this motor needs to turn).
