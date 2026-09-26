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
| `main/atos_link.hpp` | DHCP ethernet, RTPS participant, the two endpoints, 20 Hz publish |
| `main/drive_supervisor.hpp` | the state machine, watchdog, limits, and the MotorCommand -> FOC mapping |
| `main/hall_drive.hpp` | hall commutation; the speed for the loop comes from a 5 Hz tracking loop on the step count |
| `main/atos_drive.cpp` | the parent app plus hooks, the supervisor task, feedback guards, and the ATOS console commands |
| `tests/host_mib/` | host-side MIB stand-in: a native RTPS participant that streams commands and prints state |

## Build

```
git submodule update --init                       # external/rammp-rtps
idf.py build                                      # NineBot on the dyno, encoder rig
idf.py -B build-flatbot -DATOS_MOTOR=flatbot build # NineBot with office ceilings (60 rpm, 10 A, ramped)
idf.py -DATOS_MOTOR=hub20 build                   # wheelchair hub motor
idf.py -p PORT flash monitor
```

`ATOS_MOTOR` is a CMake cache variable: use a separate `-B` directory per
profile or delete `build/` when switching. The profile is compile-time on
purpose. A `MotorCommand` can lower the limits, never raise them.

## Provision a board

Once per board, on the console:

```
axis 2          # 0 front_caster_left, 1 front_caster_right, 2 drive_left, 3 drive_right
fb hall         # what the drive commutates on: hall (default, the chair) or enc (the dyno)
hcal 6          # hall table: ho 9999 45 1, then run 3 30 5 / stop / run 3 -30 5 / stop; saved
ecal 4          # encoder offset, only for fb enc; saved
```

Reboot. The board takes a DHCP lease from the MIB, joins
`rammp/mib/motor_command/<axis>` and `rammp/<axis>/motor_state`, and publishes
`MotorState` at 20 Hz. `axis` shows what is saved, `fb` shows the source and
its gains, `atos` shows the link and the state machine. Arming is refused
(`FAULT(ENCODER)`) until the selected source is calibrated. An unprovisioned
board (`axis_id` 255) stays off the network.

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
- `fw_version` is a constant in `motor_params.hpp`; no git stamp yet.
- espp on the board keeps reader proxies of dead host participants (one extra
  copy of every sample per MIB restart). Needs a fix in espp itself.
