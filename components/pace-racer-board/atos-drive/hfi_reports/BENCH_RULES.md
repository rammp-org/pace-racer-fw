# Shared HFI bench rules (read fully before touching hardware)

ONE physical bench is shared by several agents. Every command that touches the board
(/dev/cu.usbmodem2101: console, flash, usb_reset), the Rigol PSU (169.254.50.13) or the
scope (192.168.1.101) MUST happen while you hold the bench lock:

    /tmp/hfi/bench_lock.sh acquire <your-name>     # blocks until free (polls every 20 s)
    ... hardware session: flash YOUR build, run tests ...
    /tmp/hfi/bench_lock.sh release <your-name>

- Keep sessions short (<= 25 min). Plan offline first, then grab the lock, test, release.
- You may flash your own build while you hold the lock. The next holder flashes theirs;
  never assume the board runs your firmware when you (re)acquire.
- Before release: motor stopped (`stop`), no DRV fault (`s` shows en=1), PSU at 6+6 V series
  outputs ON. If a DRV fault latched ("! DRV8353 fault ... power-cycle VM"), recover:
    python3 <atos-drive>/tools/hfi_bench/scpi.py 169.254.50.13 ':OUTP CH2,OFF' ':OUTP CH1,OFF'
    sleep 4; python3 .../scpi.py 169.254.50.13 ':OUTP CH1,ON' ':OUTP CH2,ON'; sleep 1.5
    /tmp/hfi/venv/bin/python <atos-drive>/tools/usb_reset.py ; sleep 6
- Python with pyserial/pyusb/numpy: /tmp/hfi/venv/bin/python. Console one-shots:
  /tmp/hfi/venv/bin/python /tmp/hfi/con.py 'sq 1@0.3' 's@0.5' ...  (cmd@wait_s)

## Safety rules (this motor: 3 uH, 36 mOhm, 2 pole pairs, strong cogging; 12 V bus)
- Bus is 12 V (Rigol 6+6 series). Only flash builds made with `-DATOS_VBUS=12`:
    bash -c "export IDF_PYTHON_ENV_PATH=$HOME/.espressif/python_env/idf6.0_py3.11_env; source ~/esp/esp-idf/export.sh >/dev/null; idf.py -B build-drone12 -DATOS_MOTOR=drone -DATOS_VBUS=12 build"
  flash: cd atos-drive && PYTHONPATH=managed_components/espp__ota/python /tmp/hfi/venv/bin/python -m espp_ota flash --pid 0x0d41 -q build-drone12/pace_racer_atos_drive.bin
- NEVER `capdump` or print bulk console output while the motor is driven (HOLD/SPIN/HALL with
  current or injection): console output stalls the sampler ISR ~200 us, duties freeze and the
  injection acts as DC into 3 uH -> DRV VDS trip. Always `stop` before `capdump`.
- Injection: Vh <= 0.17 V when combined with any drive current; drive current ceilings <= 3 A.
- Keep `hguard 60` (default) on for any xrun/xiq (HFI-driven) test.
- Do not change PSU voltage, DRV OCP/VDS settings, or PWM dead time. Do not touch the BK supply.
- Encoder offset is stored (cal=1); do not run `ecal` unless `enc` shows cal=0.

## Context
Read components/pace-racer-board/atos-drive/HFI_HANDOFF.md (whole file, especially the
"Update 2026-10-23" sections). Bench scripts: atos-drive/tools/hfi_bench/ (hrt_log.py,
xtest.py, faultcap.py, enc_scan.py, hrt_cal.py, model_fit.py). Data from earlier runs:
/tmp/hfi/*.npy, *.json, *.npz. Current 12 V hrt calibration at Vh 0.17: `-0.0176 -0.0427 149.2`
(/tmp/hfi/cal12_017.txt).

## New worktree setup
Your worktree lacks the untracked managed_components/ and dependencies.lock; copy them first:
  SRC=/Users/alxchunlin/orca/workspaces/pace-racer-fw/high-frequency-injection/components/pace-racer-board/atos-drive
  cp -R $SRC/managed_components $SRC/dependencies.lock <your-worktree>/components/pace-racer-board/atos-drive/
Without the lock file idf.py upgrades espp/tinyusb and the build breaks.

## Update 22:25 (coordinator)
- The console-stall fix (console output no longer stalls the sampler ISR; ISR removes injection on
  late samples) is merged into alexchunlin/high-frequency-injection (merge of alexchunlin/hfi-console-stall).
  Do NOT merge it into your branch mid-task; the coordinator merges at the end. Keep obeying
  "never capdump while driving" on your own builds.
- Build gotcha: COMPONENTS in atos-drive/CMakeLists.txt is a CMake CACHE variable; after a change
  to it (that branch adds esp_driver_gptimer) an existing build dir keeps the old list. Use a fresh -B dir.
