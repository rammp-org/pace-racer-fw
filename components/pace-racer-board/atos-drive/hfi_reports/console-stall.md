# console-stall: why bulk console output froze the PWM duties

Branch `alexchunlin/hfi-console-stall` (worktree `~/orca/workspaces/pace-racer-fw/hfi-console-stall`),
based on `f764371`. Files: `main/demo_link.hpp`, `main/foc_sampler.hpp`, `main/atos_drive.cpp`,
`main/linker.lf`, `main/CMakeLists.txt`, top `CMakeLists.txt` (adds `esp_driver_gptimer` for the probe).

Setup note: the worktree started on `main` (17dafad), not f764371. I reset my own clean branch to
f764371 and copied `external/rammp-rtps` (submodule, 6e36aec) plus `managed_components/` and
`dependencies.lock` from the high-frequency-injection worktree.

**Verdict:** the duty freeze during console output is gone (max freeze during a capdump = idle
value, 150 us), and capdump under `id 0` + hrt 0.10 V shows no current excursion above +-3.5 A.
`late` is NOT back to idle: 2.1 % -> 0.9 %, against 0.02 % idle. The ISR max gap went up,
66 -> 104 us. Both are explained below.

## Root cause (measured)

The handoff's "ISR held off ~200 us" was not quite right. The sampler ISR is never held off that
long (max entry gap 66 us before the fix). What happens:

1. **Where things run** (`cores` command): sampler ISR on CPU0 (registered from `app_main`), FOC
   task on CPU1, **TinyUSB task and the USB OTG ISR on CPU1** (esp_tinyusb's default affinity is
   core 1 on dual-core parts; espp `UsbDevice` uses `TINYUSB_DEFAULT_CONFIG()`), console loop =
   `app_main` on CPU0.
2. Every `fmt::print` goes through DemoLink's stdout VFS, which called `write_cdc()` ->
   `tud_cdc_n_write_flush()` -> `dcd_edpt_xfer_fifo()` **from the printing task (CPU0)**. TinyUSB
   0.21 runs that inside `usbd_spin_lock` = `portENTER_CRITICAL` (masks level <= 3 on CPU0, so
   the level-2 MCPWM ISR) while it copies packets into the DWC2 TX FIFO, and that spinlock is shared
   with the TinyUSB task / ISR on CPU1 (CPU0 spins with interrupts masked when CPU1 holds it).
3. That makes the sampler ISR enter 5-20 us late. Any sample > 5 us late (or > 10 us of in-ISR
   time) is **discarded without waking the FOC task** (foc_sampler.hpp late-sample policy), so a
   run of late samples = no duty update for N periods. That is the freeze: FOC wake-to-wake gap
   (= how long the bridge holds one set of duties) reached 218-227 us during a capdump.
4. With HFI injection on, a frozen rotating vector is DC across 3 uH -> the scope ramps.

A/B with the stdout sink mask (`cmir`), 3000-row capdump, motor coasting, original layout:

| sinks | late | ISR max gap | FOC max gap |
|---|---|---|---|
| all (UART+CDC) | 2246 / 193k (1.2 %) | 94 us | 217 us |
| UART only (no CDC) | 115 / 252k (0.05 %, = idle) | 79 us | 208 us |
| CDC only | 322 in a 0.6 s dump | 105 us | 199 us |
| none | 70 / 252k (0.03 %) | 94 us | 159 us |

Moving only the sampler ISR to CPU1 (`-DATOS_SAMPLER_CORE=1`, `set_edge_on_core`) made it worse:
late 2.9 % (it then shares the core with the TinyUSB task and ISR).

The `pcprobe` (assist-debug PC recorder sampled from a 50 kHz GPTimer ISR on the other core)
caught CPU0 in `xPortEnterCriticalTimeout`/`spinlock_acquire` and in the sampler's ADC wait during
stalls; it mostly confirmed that stalls were late-discard runs, not a single long hold-off.

## Changes

1. **Root fix, `demo_link.hpp`**: stdout no longer touches TinyUSB from the printing task. The VFS
   write tees to the UART inline (as before) and pushes the bytes into an 8 KB FreeRTOS stream buffer;
   a `con_tx` task pinned to TinyUSB's own core (looked up by task name) drains it into CDC and the
   WebUSB LOG frames. Printers block at most 20 ms for room, then drop (best-effort as before).
   `cmir 31` restores the old inline path for A/B.
2. **`linker.lf`**: the sampler ISR methods (`isr_trampoline`, `on_sample`, `median_filter`,
   `apply_fallback`) are placed in IRAM by symbol. IRAM_ATTR on them fails to link ("dangerous
   relocation"); the symbol mapping links fine. Under console + HFI load the flash-resident ISR ran
   past its 10 us budget (`late_code`).
3. **Defence in depth, `foc_sampler.hpp` + FOC task (`hwd <n>`, default 2)**: see below.
4. Diagnostics: `s` now prints `gap isr=<max ISR entry gap> long=<entries > 1.5 periods> foc=<max
   FOC wake-to-wake gap> late_code=<lates from in-ISR time>` and, when the fallback is armed,
   `#injfb applied held_off stale1 stale_max`. `cores` prints where the ISR/tasks run. `cmir <mask>`
   selects stdout sinks. `pcprobe 1|0|(report)`.

### Defence in depth: the sampler ISR removes the injection itself

Every FOC tick that injects (hscan/hfi pulsating `vinj` or hrot/hrt rotating `vrot`) also computes
the duties WITHOUT the injection (same svpwm on the pre-injection alpha-beta voltage) and publishes
their compare values to the sampler (`set_fallback`, under the existing `g_duty_mux`). Every duty
write bumps a sequence under that lock. The ISR writes the injection-free compare values when
- the sample is late or discarded (the task will not get it, so the bridge would keep the last
  injection vector another period), or
- `hwd` consecutive samples passed without any duty write (task held off while samples arrive).

Why this design:
- The ISR is the first code to run after any hold-off, and it runs at TEP, half a period from the
  TEZ shadow load, so three comparator writes there are atomic on the bridge (plus the same TEZ
  guard as `set_duties` for the slow path). Integer-only, no FPU in the ISR.
- It keeps the fundamental voltage. Centering the duties would also remove the back-EMF
  compensation and short the motor through the zero vector (bad while spinning on a 3 uH motor);
  the frozen fundamental is the pre-existing, non-HFI behaviour.
- Disarmed whenever no injection runs, so the non-HFI paths are unchanged.
- A hardware alternative was rejected: the S3 MCPWM has no comparator watchdog/ETM and its fault
  path only brakes or Hi-Zs (same cost as above, and a Hi-Z during HFI tracking loses the estimate);
  a second watchdog ISR on the other core would race the FOC task's writes without the ISR's
  natural TEP timing.
- Cost: on the bench with hrt 0.10 V and no console load it fires on ~0.2 % of ticks (late samples
  that happen anyway at that load); each one drops one injection step, which the hrt PLL's error
  clamp absorbs.

## Measurements

All on the 12 V bench, drone profile build (`-DATOS_MOTOR=drone -DATOS_VBUS=12`), one 3000-row
`capdump` per row (`cap 1 3000 0` then `capdump`), stats reset right before each dump (`s`),
read right after. Script: `/tmp/hfi/cs_meas.py`, raw results `/tmp/hfi/cs_*.json`.
"late" = sampler late count over the dump window (~10 s incl. the 8.4 s UART-throttled dump).
"ISR gap" = max ISR entry-to-entry gap. "FOC gap" = max FOC-task wake-to-wake gap (how long the
bridge held one set of duties).

**Acceptance A: HOLD, `id 0`, no injection (motor enabled but harmless). hrt NOT running.**

| build | late during capdump | ISR max gap | FOC max gap | in-ISR lates |
|---|---|---|---|---|
| before (`f764371` + counters only, `probe.bin`) | 4130 / 197.7k = **2.1 %** (x2: 4086) | 66 us | **218-227 us** | n/a |
| fix1 (stdout drain on TinyUSB's core) | 943 / 189k = 0.5 % (x2: 0.4 %) | 104-108 us | 162-206 us | 1/3 of lates |
| **fix2 (+ ISR methods in IRAM), final** | 1752 / 197.6k = **0.9 %** (x2: 0.8 %) | 103-105 us | **150-156 us** | **0** |
| idle, same build, no dump | 15 / 72k = 0.02 % | 79-103 us | 104-150 us | 0 |

fix2: the ISR's own time is now 7.4-8.1 us max (was up to 27 us), and the duty-freeze max during a
dump equals the no-console idle value. The remaining 0.8 % are entry-lates that come with USB
traffic itself, not with who calls TinyUSB: on fix2 the old inline path (`cmir 31`) gives the
same 0.8 %, UART-only printing 0.1 %, no sinks 0.02 %. They are now isolated single lates
(FOC gap = idle), so they no longer chain into multi-period freezes.

The ISR max gap went UP (66 -> ~104 us; `long` 0 -> 12-18 per dump, also 1-2 per 3 s at idle).
That is one sample period plus ~4 us late, i.e. the "long" entries are mostly isolated late samples
that follow a discarded one, but I did not pin down why fix1/fix2 show more of them than before. It
does not show up in the FOC gap.

**Acceptance B: scope, `id 0` + `hrt 0.10 -0.0176 -0.0427 149.2`, 3000-row capdump while injecting**
(`tools/hfi_bench/faultcap.py`, Micsig clamp on phase A, single shot, 12 s wait):

| build | trigger | result |
|---|---|---|
| fix1 | +6 A | **NO TRIGGER** |
| fix2 | +6 A | **NO TRIGGER** |
| fix2 | +3.5 A | **NO TRIGGER** |
| fix2 | -3.5 A (falling edge, `/tmp/hfi/cs_faultcap_neg.py`) | **NO TRIGGER** |
| fix2, positive control (no dump, trigger +0.5 A, hrt starting) | +0.5 A | triggered |

Board-side numbers for the same load on fix2 (`/tmp/hfi/cs_after2_hrt.json`, hrt 0.10 V running):

| | late | FOC max gap | fallback applied | stale_max |
|---|---|---|---|---|
| hrt, no dump (3 s) | 62 / 246k = 0.03 % | 170 us | 10 | 2 |
| hrt + 3000-row capdump | 1953 / 226k = 0.9 % | 169 us | 1357 (1267 on late samples) | 4 |

Before the fix this sequence is the known DRV trip (0 -> 8.6 A in 4 periods at 0.15 V, handoff).
The rows above are with hrt running, so with breakaway-tracking's FOC CPU-budget issue still
present on this branch (cmax 100-170 us). Without their fix, hrt alone already gives
0.2 % lates and FOC gaps up to 289 us on fix1 (flash ISR).

Caveat on the scope: the positive-control record that faultcap read back covers only the
pre-trigger 2 ms (t = -2.5...-0.5 ms, +-0.2 A baseline), so the waveform itself was not saved. The
control shows only that the trigger fires on >= 0.5 A injection current.


## Open

- Residual 0.8 % entry-lates during heavy CDC output (vs 0.02 % idle), the same whichever core
  calls TinyUSB. Best guess (not proven): cross-core contention with the TinyUSB task/ISR on
  CPU1, e.g. on the FreeRTOS kernel lock or the global intr-alloc spinlock that
  `shared_intr_isr` takes (the MCPWM interrupt is allocated `ESP_INTR_FLAG_SHARED`). Next probe:
  record the interrupted PC from the level-2 exception frame on entry-late samples, or move
  TinyUSB to CPU0 (needs an espp `UsbDevice` config knob for `tinyusb_config_t.task.xCoreID`;
  today it hard-codes `TINYUSB_DEFAULT_CONFIG()` = core 1) with the ISR on CPU1 and compare.
- The late-sample policy (discard, do not wake the FOC task) is what turns lateness into frozen
  duties. With injection on, the fallback now covers it. A cleaner long-term answer would be to
  wake the task on late samples with a "stale current" flag, so it advances the injection and
  holds the current loop. Not done: it changes control semantics the other branches depend on.
- Not tested: the fallback against a real long FOC-task stall (`stale >= hwd` path). The bench
  only hit the late-sample path (stale_max 4, with lates in it). `hwd 0` + capdump + injection
  (fallback off) was deliberately NOT run: it is the known trip.
- WebUSB vendor writes (telemetry, dashboard replies) still run on whatever task calls them, so a
  dashboard session can still put TinyUSB critical sections on CPU0. Only stdout goes through the
  drain.
- `sdkconfig.defaults`'s MCPWM_ISR_IRAM_SAFE comment and `linker.lf`'s "late% is not limited by
  cache" note predate this. The in-ISR part of late% WAS cache-limited under this load. I appended
  a section and left their text alone.
- HFI_HANDOFF.md not edited (to keep the merge simple). Its "root cause OPEN" line is superseded
  by this report.
- Keep the "never capdump while driving" rule for anything beyond hrt 0.10 V / id 0 until
  acceptance B is re-run at that operating point.

## Commits (branch `alexchunlin/hfi-console-stall`, on f764371)

- `28ebbf4` diag(atos-drive): ISR/FOC gap stats, 'cores', 'cmir' stdout sink mask
- `f17d291` feat(atos-drive): ISR injection fallback + 'pcprobe' stall probe
- `9d16f1a` fix(atos-drive): drain stdout to USB on TinyUSB's core; fallback on late samples
- `529057e` perf(atos-drive): sampler ISR methods in IRAM; fallback write after the conversion

## Commands added

`cores` | `cmir <mask>` (1 UART, 2 CDC, 4 WebUSB LOG, 8 eth, +16 old inline USB path) |
`hwd <n>` (fallback threshold, 0 = off, default 2) | `pcprobe 1|0|` | extra `s` fields.
Build option `-DATOS_SAMPLER_CORE=1` (default 0; measured worse, kept for A/B).
Binaries: `/tmp/hfi/cs_bins/{probe,c1,fix1,fix2}.{bin,elf}`. The board was left running fix2.
