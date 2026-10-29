# atos-drive

The PACE RACER board as an ATOS motor controller: `MotorCommand` in,
`MotorState` out, over RTPS, per `rammp-rtps` `motor_message.hpp`. The contract
and the reasoning behind it are in `docs/ATOS_MOTOR_API.md` at the repo root.

Fork of `halls-sensorless` (2026-09-16). The FOC sampler, current loop, hall and
encoder commutation, guards and console are the parent's. Added:

| file | role |
| --- | --- |
| `main/motor_params.hpp` | pole pairs, Kt, speed/current/accel ceilings, hold current; three profiles, selected at build time |
| `main/axis_config.hpp` | NVS: `axis_id`, feedback source, encoder offset, hall table |
| `main/atos_link.hpp` | DHCP ethernet, RTPS participant, the two endpoints, 50 Hz publish |
| `main/drive_supervisor.hpp` | the state machine, watchdog, limits, and the MotorCommand -> FOC mapping |
| `main/hall_drive.hpp` | hall commutation; the speed for the loop comes from a 5 Hz tracking loop on the step count |
| `main/atos_drive.cpp` | the parent app plus hooks, the supervisor task, feedback guards, and the ATOS console commands |
| `tests/host_mib/` | host-side MIB stand-in: a native RTPS participant that streams commands and prints state |
| `tools/provision.py` | plug in a board over USB: flash, set axis and feedback, hall-calibrate, report |

## Build

```
git submodule update --init                       # external/rammp-rtps
idf.py build                                      # NineBot on the dyno, encoder rig
idf.py -B build-flatbot -DATOS_MOTOR=flatbot build # NineBot with office ceilings (60 rpm, 10 A, ramped)
idf.py -B build-hub20 -DATOS_MOTOR=hub20 build    # wheelchair hub motor (measured R/L/lambda/gains, 2026-09-24)
idf.py -p PORT flash monitor
```

`ATOS_MOTOR` is a CMake cache variable: use a separate `-B` directory per
profile or delete `build/` when switching. The profile is compile-time on
purpose. A `MotorCommand` can lower the limits, never raise them.

## Provision a board

With the board on USB and `build-flatbot` built:

```
tools/provision.py            # finds the board by MAC, flashes, sets axis + fb hall, hcal if needed
tools/provision.py --list     # what is plugged in
```

The axis comes from the `ROSTER` table at the top of the script (MAC -> wheel);
`--axis N` overrides it. Boards in `BLOCKLIST` are never touched. The hall
calibration step asks for the wheel to be free and the supply on, then spins
it 30 rpm each way for ~15 s and checks the table saved.

By hand, once per board, on the console:

```
axis 2          # 0 front_caster_left, 1 front_caster_right, 2 drive_left, 3 drive_right
fb hall         # what the drive commutates on: hall (default, the chair) or enc (the dyno)
hcal 6          # hall table: ho 9999 45 1, then run 3 30 5 / stop / run 3 -30 5 / stop; saved
ecal 4          # encoder offset, only for fb enc; saved
```

Reboot. The board takes a DHCP lease from the MIB, joins
`rammp/mib/motor_command/<axis>` and `rammp/<axis>/motor_state`, and publishes
`MotorState` at 50 Hz. `axis` shows what is saved, `fb` shows the source and
its gains, `atos` shows the link and the state machine. Arming is refused
(`FAULT(ENCODER)`) until the selected source is calibrated. An unprovisioned
board (`axis_id` 255) brings ethernet up but joins no topic.

## Ethernet service stream (2026-10-15)

A board inside a wheel has no reachable USB port, so everything the WebUSB
vendor stream carries for commissioning is also served over TCP
(`main/eth_link.hpp`): OTA (module 0, a second `espp::OtaService` on the same
engine), the console (0x10, lines in, stdout out), reset and info (0x20) and
discovery (0xFF), same frames as USB. Ports: **tcp :5555** service stream, one
client at a time (a newcomer replaces a stale session); **udp :5556**
discovery, `PACE?` in (broadcast is fine), one JSON line back by unicast with
axis, feedback, calibration state, motor profile, firmware, partition, base
MAC and the service port. Ethernet comes up whether or not the axis is
assigned.

```
tools/eth_bridge.py                         # provisioning dashboard at http://localhost:8765/ (also WebUSB: open tools/dashboard.html)
tools/eth_link.py list                      # who answers on the LAN
tools/eth_link.py drive_left info           # by axis name, number, IP or 'auto'
tools/eth_link.py auto flash                # OTA build-hub20, wait for it back, mark valid
tools/eth_link.py auto cmd "axis"           # one console line
tools/eth_link.py auto console              # interactive
tools/eth_link.py auto reset | status | mark-valid | rollback
tools/provision.py --host auto --motor hub20   # the whole provisioning flow over ethernet
```

Rollback is gated on the network, not on uptime. A new image boots
PENDING_VERIFY; the USB link no longer self-confirms after 30 s. It confirms
itself only once it holds a lease and the service stream is accepting, 20 s
later, and the host tool confirms explicitly as soon as it reconnects. An
image that never gets that far stays pending and the bootloader boots the
previous one at the next power cycle, which is the only recovery a sealed
board has. First flash of a board still has to go over USB: the firmware on
it has to have this server before ethernet can carry the next one.

Known shape of the LAN: the bench switch drops board-originated multicast,
which is why discovery is a host question with a unicast answer rather than
mDNS. On this Mac bind to the dock interface for 169.254 targets (the tool
does) or Tailscale swallows the traffic.

## Driving it

The MIB, or `tests/host_mib/mib_stub`, streams `MotorCommand` at 10..50 Hz.
200 ms without a command: safe stop, then hold, then `FAULT(WATCHDOG)`.

Limits: a `torque_limit` / `vel_limit` of 0 is the ceiling in `motor_params.hpp`.
`accel_limit` 0 is a step, unless the profile has an acceleration ceiling
(flatbot: 12.6 rad/s^2), in which case 0 and anything larger mean the ceiling.
`vel_limit` holds in every mode: in TORQUE a governor rolls torque off across
the top 20 % of the limit. The board reports the values in force in
`MotorState.*_limit_eff`.

Faults the drive raises on its own while commutating on halls: `HALL` (an
illegal 000/111 input for 50 ms) and `STALL` (speed or position mode, speed
integrator past half the torque cap, no net hall steps for 0.5 s). Both unload
and coast before the supervisor names them. Verified 2026-09-25 on the bench:
a pulled hall line faulted within one sample; a hand-held wheel faulted about
2 s after the grab at 5.6 A peak; a hall line left out gave a legal-but-wrong
pattern that the STALL guard caught in 3 s.

`vel_limit` is a hard cap in every mode, not just a setpoint clamp: above it
the drive rolls torque off and then brakes, so a wound-up integrator released
from a stall cannot launch the wheel (measured 18.5 rad/s against a 6.3 limit
before this, 4.1 after).

## Tuning over ATOS (2026-10-09, API v2)

A third topic per axis, `rammp/mib/motor_tuning/<axis>` (`MotorTuning`:
`tuning_req_id`, `speed_kp` A per rad/s, `speed_ki` A per rad, `hall_pll_hz`,
`persist`). The supervisor acts on a new req_id, clamps into the
`kTune*` bounds in `motor_params.hpp`, and applies on its next tick in any
state, without resetting the integrator. `MotorState` echoes `tuning_ack` and
the three values in force (SI; the console's `sg`/`hpll` are A/rpm and Hz).
API v3 (2026-10-19, D38): a negative `speed_kp`/`speed_ki` sets that gain to
exactly 0 (0 still means unchanged), and with ki = 0 both speed loops hold
the integrator at 0, so it empties instead of freezing (`sg <kp> 0` too).
`persist` = the stream task writes the live values to NVS (`spd_kp`,
`spd_ki`, `pll_hz`), restored at boot over the per-source defaults. Console:

| command | meaning |
| --- | --- |
| `tune` | live tune in both unit systems, last ack, and what NVS holds |
| `tune save` | keep the live values across reboots |
| `tune clear` | compiled defaults at the next boot |

`mib_stub` sends one with `tune <kp> <ki> <pll_hz> [save]`. Not yet run on a
board: built clean (flatbot), host stand-in built; bench next.

## Native USB (WebUSB) link (2026-09-28)

The USB-C port is now a composite native USB device (`main/demo_link.hpp`,
shared with the test-bench repo): a **WebUSB vendor** stream carrying OTA
(module 0), telemetry (3), the flash core dump (4), a console bridge (0x10:
text commands in, stdout out) and a system module (0x20: reset, info), plus a
**CDC** console. USB-Serial/JTAG shares the PHY and is gone while the app runs:
flash with `test-bench/tools/ota.py racer` (espp OTA, auto-confirm), reset
over the system module, and hold BOOT while plugging in only if an image
cannot bring USB up. Primary console is UART0 (the tx/rx header). The
partition table has two 3 MB OTA slots, rollback and a core dump partition
(16 MB flash). Before an OTA the drive is unloaded and the sampler disabled:
flash writes disable the cache and the sampling ISR is not IRAM-resident.
Telemetry (20 Hz: `rpm_set rpm_hall iq iqref id vd vq t0..t3 mode`) and the
stdout mirror on the vendor stream are quiet until a host sends SET_STREAM.

## Characterization hooks (2026-09-28)

`test-bench/web/characterize.html` runs the motor-identification sequence
(R/L, pole pairs, hcal, ecal, λ, K<sub>t</sub> against the stand's torque
sensor, J, speed-loop pole placement, current-loop gains) from the browser
using this app's existing primitives. Two things were added for it:

- vendor-stream module **0x30, capture readout**: `0x01 META → 0x81 [n u32][every u32][mode u8][want u32][fs_hz f32]`,
  `0x02 READ [off u32][count u16] → 0x82 [off u32][3 × f32 per row]` (≤ 340 rows per reply). Same ring as
  `cap`/`capdump`, but binary and request/reply, because the stdout mirror on the vendor stream is
  best-effort and would drop rows of a 3000-row dump.
- console `mi`: one `#mi name= pp= gear= kt= r= l_uh= lambda= vbus= kp= ki= vlim= maxrpm= maxamps= spd_kp= spd_ki= enc_cal= hall_cal=`
  line with the compiled-in and live constants.

`hv` now commutates on the encoder when its offset is known (the `erun` path,
with the encoder speed gains), falling back to halls otherwise; `hv` with no
argument reports which.

## Driving it from a browser or script (2026-09-28)

For a host that streams small setpoint steps over the USB console (the dyno
demo page in `test-bench/web/demo.html`):

| command | meaning |
| --- | --- |
| `hv <rpm> [A]` | hall speed setpoint; starts the speed loop if it is not running, otherwise only the target moves (50 rpm/s ramp, 100 rpm cap, default 10 A ceiling) |
| `wd <ms>` | console heartbeat watchdog, 0 = off. No console line for `ms` while the drive is live: unload, Hi-Z coast, one `! watchdog` line. One-shot; the next command re-arms nothing, send `wd` again |
| `hb` | heartbeat, prints nothing |

The stream gained a 17th column, `rpm_set`: the ramped reference the speed
loop is tracking, 0 outside a speed loop. Verified on drive_left: `hv 20` at
0.65 A, watchdog fired 1591 ms after the last `hb`.

## Bench notes (2026-09-25)

- The bench switch does not forward the board's multicast to the Mac, so
  discovery needs the forged SPDP trick in the memory notes. The stand-in
  works once the board's participant is injected.
- Halls only, NineBot, free wheel: velocity within 2 % at 3 rad/s and above
  (spread 0.17 rad/s); at 1.5 rad/s the motion itself is cogging-jerky
  (0.8 rad/s spread, confirmed on the encoder). Position hold sits at 0.05 A.
- Speed gains follow the feedback source: 0.05/0.2 on the encoder, 0.025/0.1
  on halls (`fb` prints them, `sg` overrides, `hpll` sets the tracking loop).

## Not yet done

- A hall line shorted high (legal-but-wrong pattern at speed) was not tried;
  accepted risk (Alex, 2026-09-25): halls are assumed not to fail high. The
  STALL guard is the backstop if one does.
- Two boards on one network; induced VDS/thermal/sampler faults; any load.
- The hub motor: quadrature encoder source (PCNT) with hall-edge sync, R/L
  measurement, current-loop retune. `ATOS_MOTOR=hub20` only swaps constants.
- `fw_version` is a constant in `motor_params.hpp` (0.2.0 with the tuning topic, 0.3.0 with true-zero gains, 0.3.2 with the hub20 15 A boot cap and halved speed Kp, 0.4.0 with the hub20 30 A peak budget, VDS 0.20 V and 50 Hz MotorState); no git stamp yet.
- espp on the board keeps reader proxies of dead host participants (one extra
  copy of every sample per MIB restart). Needs a fix in espp itself.
