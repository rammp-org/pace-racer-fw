# atos-drive

The PACE RACER board as an ATOS motor controller: `MotorCommand` in,
`MotorState` out, over RTPS, per `rammp-rtps` `motor_message.hpp`. The contract
and the reasoning behind it are in `docs/ATOS_MOTOR_API.md` at the repo root.

Fork of `halls-sensorless` (2026-09-16). The FOC sampler, current loop, hall and
encoder commutation, guards and console are the parent's, unchanged. Added:

| file | role |
| --- | --- |
| `main/motor_params.hpp` | pole pairs, Kt, speed/current ceilings, hold current; two motors, selected at build time |
| `main/axis_config.hpp` | NVS: `axis_id`, encoder offset, hall table |
| `main/atos_link.hpp` | DHCP ethernet, RTPS participant, the two endpoints, 20 Hz publish |
| `main/drive_supervisor.hpp` | the state machine, watchdog, limits, and the MotorCommand -> FOC mapping |
| `main/atos_drive.cpp` | the parent app plus hooks, the supervisor task, and the `axis` / `atos` console commands |

## Build

```
git submodule update --init            # external/rammp-rtps
idf.py build                           # NineBot (dyno) parameters
idf.py -DATOS_MOTOR=hub20 build        # wheelchair hub motor parameters
idf.py -p PORT flash monitor
```

## Provision a board

Once per board, on the console:

```
axis 2          # 0 front_caster_left, 1 front_caster_right, 2 drive_left, 3 drive_right
ecal 4          # encoder offset (or eofs <deg> to restore a known one); saved
hcal / hset ... # hall table; saved
```

Reboot. The board takes a DHCP lease from the MIB, joins
`rammp/mib/motor_command/<axis>` and `rammp/<axis>/motor_state`, and publishes
`MotorState` at 20 Hz. `axis` shows what is saved, `atos` shows the link and the
state machine. An unprovisioned board (`axis_id` 255) brings ethernet up and
stops there.

## Driving it

The MIB, or the Python stand-in in `tools/`, streams `MotorCommand` at 10..50 Hz.
While that stream is live the console refuses motion commands (`run`, `erun`,
`epos`, ...); `stop`, `f`, limits and provisioning still work. 200 ms without a
command: safe stop, then hold, then `FAULT(WATCHDOG)`.

Limits: a `torque_limit` / `vel_limit` of 0 is the ceiling in `motor_params.hpp`
(further clamped by the console `lim`); `accel_limit` 0 is a step. The board
reports the values in force in `MotorState.*_limit_eff`.

## Not yet done

- Bench validation on the dyno (the API has not been exercised on the wire).
- The hub motor: quadrature encoder source (PCNT) with hall-edge sync, R/L
  measurement, current-loop retune. `ATOS_MOTOR=hub20` only swaps constants.
- `fw_version` is a constant in `motor_params.hpp`; no git stamp yet.
