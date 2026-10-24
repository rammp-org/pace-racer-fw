# boot-panic: report (2026-10-24)

Branch `alexchunlin/hfi-boot-panic` (not pushed):
- `562a9f5`: merge of `alexchunlin/hfi-polarity-detection` into f8743a5. One file conflicted
  (`atos_drive.cpp`) and I kept both sides: the hrtm/hrtk/hrtn/cff/xsl atomics and the hpol block,
  `d_hat` together with `pol_left/pol_hold`, the `hseed` seed branch with `d_hat = 0`, and one help
  line that lists the commands from both branches.
- `7713764`: the fix.

## Root cause: the main task overflowed its 8 KB stack (not heap corruption)

`app_main` runs in the main task (CONFIG_ESP_MAIN_TASK_STACK_SIZE=8192). Its own frame is
`entry a1, 0x1200`, which is **4.6 KB**: every hook, config struct and lambda lives in it. Boot
peaks at **~8.33 KB** inside `FocSampler::zero_calibrate` (foc_sampler.hpp:580). That is a
`fmt::print` with 9 floats, and fmt's float formatter needs a deep stack. So the task ran
**~140 B below the bottom of its stack**, into the heap block underneath. Whichever object the heap
layout put there got overwritten. Any change to .bss/.data/heap use moves the victim, so the crash
"follows" unrelated commits:

| image | victim / symptom |
|---|---|
| polarity ce252c1 (their report) | intr_alloc vector-descriptor list → panic in `find_desc_for_source` ← `mcpwm_timer_register_event_callbacks` ← `set_edge` |
| merged tree, plain (my repro 1) | partition list → `abort()` in `esp_ota_get_running_partition` ("partition table corrupt") ← `DemoLink::start`; OTA rollback |
| merged tree, heap poisoning on | boots, but the ota_1 partition descriptor is trashed (size=0, `flash_chip`=0x101xx) → **every OTA onto it panics in `esp_ota_begin` → `esp_flash_erase_region`** (LoadProhibited 0x10119) |
| merged tree + checkpoint code | boots, but the running partition's label reads `"ota_\xa8\xa4\xcb?"`: a main-stack pointer (0x3fcba4a8) written over the label |

### Evidence
1. **The writer, caught directly.** I put a hardware store-watchpoint (both cores) on the label
   word of the running partition that had been corrupted. It fired with `DebugException` in task
   `main`, in `fmt::...write_fixed` ← `vprint_buffered` ← `FocSampler::zero_calibrate`
   (foc_sampler.hpp:580) ← `app_main` (atos_drive.cpp:2406). At that moment **SP = 0x3fcba520 and
   the main TCB's `pxStack` = 0x3fcba5ac**, so SP was 0x8c (140 B) *below* the stack.
2. **The size matches.** With a 16 KB stack the real low-water mark is 8052 B never used:
   16384 − 8052 = 8332 B peak = 8192 + 140, the same overshoot as the SP above.
3. **The heap checks agree.** `heap_caps_check_integrity_all()` passed at all 16 checkpoints
   through app_main, with and without COMPREHENSIVE poisoning. Every block header and canary was
   intact; only the data inside a live block (the one under the stack) was overwritten. That is why
   poisoning never flagged it.
4. **Why it hid.** The stack high-water mark never showed it. At 8 KB it read 292–376 B free
   throughout boot, because the overflowing frame skips the bottom bytes of the stack (big frames,
   partly untouched buffers). The FreeRTOS end-of-stack canary misses it for the same reason.
   espcoredump's "USED/FREE" column is also misleading here (it showed 1364 B free).

Ruled out along the way:
- The friend-injection borrows: they just read `timer_`/`comparators_`.
- MCPWM double registration: `set_edge` registers once; BldcDriver's own `register_pwm_sample_callback` is never called.
- `g_drv_checks`: its index is bounded.
- The FOC task stack: its frame is 0x260, and it used 1264/4096 in the core dump.

## Fix (7713764)
- `sdkconfig.defaults`: `CONFIG_ESP_MAIN_TASK_STACK_SIZE=16384`. That leaves 8 KB of margin, and the
  heap has room for it (internal free ~144 KB after boot).
- `atos_drive.cpp`: a `static_assert(CONFIG_ESP_MAIN_TASK_STACK_SIZE >= 16384, ...)` with the
  explanation. **The per-worktree `atos-drive/sdkconfig` is untracked and keeps 8192.** After merging,
  a stale sdkconfig fails the build with a message saying to delete it, instead of silently booting a
  corrupting image. I checked that it fires. Other worktrees: delete `atos-drive/sdkconfig`, or build
  with a fresh `-B dir -DSDKCONFIG=<dir>/sdkconfig`.
- `bootlog` now also prints the true main-task stack margin, the heap integrity, and the NVS init
  result (plus whether `axiscfg::load` wiped NVS).
- I did not shrink app_main's 4.6 KB frame. It would be worthwhile but isn't needed. Moving the
  hook and config setup into helpers, or the console loop into its own task, would do it.

## Acceptance
I built f8743a5 + polarity + fix with `-DATOS_MOTOR=drone -DATOS_VBUS=12`, in fresh build dirs with
their own sdkconfig. Every cycle was: OTA flash, then check the running image (partition flips
ota_0↔ota_1, reset_reason SW, `core_dump=false` after I erased the old dump), then USB reset, then
check again plus `bootlog` and `enc`.
- **Heap poisoning off (build-fx): 5/5.** Every boot showed "main task stack: 8052 B never used of
  16384 | heap intact" and `cal=1`.
- **Heap poisoning COMPREHENSIVE (build-fxp): 5/5.** 8048 B never used, heap intact, `cal=1`.
- Reading the version string alone was not enough: every dirty build reports
  `v0.1.0-48-g562a9f5-dirty`, and the `built` field comes from a TU that doesn't always recompile.
  I identified the image by the partition flip plus the new `bootlog` line, which only the fix
  image prints.
- Caveat: the final commit adds the `nvs init` bootlog line *after* those cycles. The committed
  build compiles, but that exact binary has not been on the board. The tested binary differs only
  by that print.

## Secondary: encoder offset lost (enc cal=0)
- **Probable cause: the same overflow.** The NVS partition descriptor sits in the same partition
  list that the overflow was trashing.
  - If its label or fields get hit, `nvs_flash_init()` fails for that boot. `load()` returns early
    and you get `cal=0 ofs=0`, the symptom polarity saw.
  - If it returns NO_FREE_PAGES/NEW_VERSION, `axiscfg::load` **erases NVS** and the offset is gone
    for good.
- I did not reproduce the loss itself: the offset survived all of my boots, including the crashing
  ones.
- **Across a flash:** `cal=1` across 11 flash+reset cycles of the fixed image.
- **Across a VM cycle: NOT verified** (see below). The new `bootlog` "nvs init" line will tell the
  two cases apart if it happens again.

## OPEN: the board dropped off USB during a VM cycle
- After the acceptance runs I did a BENCH_RULES VM cycle (`:OUTP CH2,OFF`/`CH1,OFF`, 4 s,
  `CH1,ON`/`CH2,ON`) to test the encoder offset. The board vanished from USB: not in ioreg, no
  `/dev/cu.usbmodem*`. It did not come back after a second VM cycle (10 s off) or after
  power-cycling the empty ports of hubs 32-1.4 (2,4) and 32-1 (1,2,3).
- Per the coordinator, the board is on a port we cannot control, so **it needs a person to replug
  it or press reset.**
- PSU left at 6+6 V series, outputs ON, ~76 mA. Motor stopped; no drive in any of my tests.
- Image in the board before the drop: the fix, plain (build-fx), in ota_0, verified running with
  `cal=1`. ota_1 holds the same fix with poisoning. The bench lock is released, with a note in
  `/tmp/hfi/BENCH_NOTE_boot-panic.txt`.
- I could not see the MCU, so these are only candidates:
  - (a) The MCU reset during VM-off, then hung before USB comes up. The boot path has two silent
    exits that run before `DemoLink::start`: the DRV8353 config retry loop (it prints to a console
    that isn't up yet) and the `zero_calibrate` failure `return`. Either leaves the board
    mute on USB. The DRV gotcha "VM must be on before the ESP resets" fits this.
  - (b) A crash on DRV SPI failure while VM is off, followed by (a).
  - (c) A USB-side wedge.
  - Nothing in the fix touches VM-, DRV- or USB-dependent paths; the only difference is the
    larger main stack.
  - Suggestion: start `DemoLink` (USB) before the DRV retry loop and before the zero-cal check,
    or make those paths retry instead of `return`, so a board that boots without VM stays
    reachable.

## Coordination notes
- Between ~23:50 and 00:10 the board ran my heap-poisoned diagnostic image. Its OTA path panics
  (row 3 of the table), so **any OTA attempted onto it failed and left that image running with
  `core_dump=true`**. hfi-higher-vh's "f8743a5 + small changes rolls back" report from that window
  is probably this. I recovered the board with `espp_ota rollback` to f8743a5 (ota_1). Even so,
  their tree has the same latent overflow, and the fix applies to it too.
- Tools I wrote:
  - `/tmp/hfi/bp/cd_fetch.py` fetches the full core dump over WebUSB, for `esp-coredump info_corefile -t raw`.
  - `/tmp/hfi/bp/cd_erase.py` erases the stored core dump.
  - `/tmp/hfi/bp/info_raw.py` prints the raw INFO JSON; `usb_reset.py --info` crashes on a corrupt label.
  - `/tmp/hfi/bp/cycle.sh <builddir> N` runs the flash/reset acceptance loop.
- Core dumps and ELFs: `/tmp/hfi/bp/cd{1,2,3}.bin`, `/tmp/hfi/bp/wp1.elf` (the watchpoint build).
