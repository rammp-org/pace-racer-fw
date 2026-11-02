# PACE RACER motor-controller API over ATOS: design record

Status: **design settled 2026-09-16; Phase 1 merged (rammp-rtps PR #5); Phase 2 firmware bench-verified 2026-09-25; tuning topic added 2026-10-09 (D37, API v2, merged in rammp-rtps PR #7 2026-10-16); true-zero gains 2026-10-19 (D38, API v3); hub20 peak budget + 50 Hz state 2026-10-29 (D39, fw 0.4.0); hub20 accel ceiling + iq slew 2026-11-01 (D40, fw 0.4.1); hub20 noise-robust soft trip 2026-11-01 (D41, fw 0.4.2); hub20 VDS 0.30 V 2026-11-01 (D42, fw 0.4.3); rest hold settles + railed fast trip 2026-11-01 (D43, fw 0.4.4)** (branch `feat/atos-drive`) (grilling session 2026-09-15/16, 39 questions,
frontier empty). Sections: Context, Facts, Decisions (D1-D32), Wire types,
Behavior contract, Implementation plan, Host-side MIB stand-in, Bench
verification, Risks. Not yet implemented.

## Context

- Integrating board: ESP32-P4 speaking ATOS (in-house RTPS participant). It is the MIB on the wire (`rammp/mib/...`, rammp-rtps PR #4); the joystick header calls the same board MCB.
- 2 PACE RACER boards initially, up to 4 on the chair. A Jetson is also on the
  network.
- Board side: ESP32-S3 + W5500, espp/rtps >= 1.2.0 (embeddedRTPS engine, typed
  `espp::Publisher<T>` / `espp::Subscriber<T>`, XCDR1 wire format derived from
  struct member order). Existing telemetry proof: `pace_racer/telemetry`,
  16 floats, 10 Hz, best-effort (`halls-sensorless/main/rtps_telem.hpp`).

## Facts (looked up, not decided)

| Fact | Source |
| --- | --- |
| RTPS publish saturates ~70-100 samples/s board-side; raw link ~250 pps | memory `rtps-telemetry-results`, `ETHERNET_FOC_COEXISTENCE.md` |
| Binary size hurts the sampling ISR more than RTPS runtime does | same |
| ROS 2 topic/type name mangling NOT implemented in espp/rtps | `rtps_telem.hpp` comment |
| Control modes that exist today (console): torque `eiq`, velocity `erun` (clamp +/-520 rpm), position P(D) `epos`/`pg` | `halls-sensorless/main/sensorless.cpp` |
| Runtime limits `lim`/`vl`/`tl`/`vds` with compile-time hard ceilings; every guard exits via gentle unload -> Hi-Z coast | `SENSORLESS_FOC.md` |
| No bus-voltage sensing: `kBusVoltage = 48.0f` is a compile-time constant in the drive app; the BSP exposes no VM divider ADC channel | `sensorless.cpp:92`, `pace-racer-board.hpp` |
| MT6701 is geared 2.5x: absolute electrical angle, NOT absolute over an output rev; multiturn must be counted from boot | memory `sensorless-foc-plan` |
| AksIM-2 (17-bit + 16-bit multiturn, SSI/RS422) exists on the bench, not integrated into the drive | memory `aksim2-encoder-is-ssi` |
| `pace-abc/ATOS-Bridge-Comms` has only bringup apps; no motor messages anywhere yet | filesystem |
| Shared message repo: `rammp-org/rammp-rtps`: header-only C++20 structs ARE the XCDR1 wire layout, `enum class X : uintN_t`, `Topic<Message>` typed handles, naming `rammp/<publisher>/<message>` / `rammp/msg/<Message>`, best-effort + no durability + state resent periodically, consumed as a git submodule | rammp-rtps README + `joystick_message.hpp` |
| rammp-rtps already defines joystick <-> MIB messages (XYTwist, ActuatorCommand w/ `req_id` ack pattern, McbStatus, ActuatorState, Diagnostics w/ `seq`) | same |
| rammp-rtps `actions/` and `services/` directories exist but are empty placeholders | same |

## Decisions

| # | Decision | Why |
| --- | --- | --- |
| D1 | The P4 (MIB) is the sole commander. The Jetson only consumes telemetry; anything it wants goes through the MIB. | The board does not arbitrate between masters; no source-ID / ownership concept needed. |
| D2 | Trajectories = streamed setpoints from the MIB at an as-yet-unknown rate (velocity control primarily). No spline upload. | Joystick-style driving; board tracks latest setpoint. |
| D3 | Control modes: COAST, TORQUE, VELOCITY, POSITION, HOLD. No PROFILED mode (removed round 2); the MIB's planner generates trajectories and streams setpoints. | Keeps the board a dumb, fast tracker; planning lives in one place. |
| D4 | Safety is level-triggered: every command carries a `requested_state` in {DISARMED, ARMED, SAFE_STOP, ESTOP}; the board converges to it. CLEAR_FAULT is an explicit command (round 3: reinstated; an engineer should not have to know that DISARM->ARM clears a fault). Mechanism: `clear_fault_req_id` counter (D25). | Best-effort transport + periodic resend means a lost level costs nothing; an explicit clear is clearer than an implied edge. |
| D5 | Units: SI at the motor output shaft (rad, rad/s, N.m). Board hides encoder gearing/cal; MIB does chair kinematics. Torque = Kt*iq, labelled estimated. | |
| D6 | Telemetry rates are hard-coded, target 20-50 Hz. Fewer publishers carrying more data; RTPS endpoints are a fixed resource on the S3. | |
| D7 | All wire types live in `rammp-org/rammp-rtps` (PR there), following its conventions above. No local message definitions. | One source of truth for MIB, boards, Jetson. |
| D8 | Network: MIB runs a DHCP server; PACE RACER is a DHCP client. No static IPs -> board identity cannot come from IP. | |
| D9 | No config / PID over the network in v1. Per-command `torque_limit` / `vel_limit` / `accel_limit` ARE allowed (they are setpoints, not config); 0 = firmware default; board clamps to compile-time hard ceilings and reports the effective values. RTPS messages carry setpoints and safety only; tuning stays console + NVS. | |
| D10 | Axis identity from NVS (`axis_id`, console-provisioned). Factory default is UNASSIGNED (not one of the four) so an unprovisioned board acts on no one's commands. Axes: FRONT_CASTER_LEFT, FRONT_CASTER_RIGHT, DRIVE_LEFT, DRIVE_RIGHT. | DHCP means no identity from IP; unassigned default is fail-safe. |
| D11 | One topic per wheel in each direction. A board subscribes only to its own command topic and does not deserialize another board's commands. Same *type* for all wheels. | Isolation; per-topic matching does the filtering in the RTPS layer. |
| D12 | Command watchdog: 200 ms (firmware constant; the MIB must therefore stream commands at >= 10 Hz even when idle). No wired e-stop on the chair; the MIB carries e-stop over the network and the watchdog is the backstop. | |
| D13 | Telemetry: one `MotorState` per board at 20 Hz, includes `fw_version`. | D6 + 4 boards x 20 Hz = 80 pps into the MIB. |
| D14 | Setpoint semantics: no feedforward fields. VELOCITY: setpoint goes straight into the velocity controller (drive to it as fast as the limits allow). POSITION: drive to the latest received position each setpoint. | MIB-side planner owns smoothness. |
| D15 | Casters are not steered. All four axes are drive axes (forward/back + hold). Position mode is for holding/creeping, so **position reference = multiturn from boot** is sufficient (A1 promoted to a decision). No absolute encoder, no homing. | |
| D16 | All faults latch into `FAULT` (watchdog included) until cleared by CLEAR_FAULT. | Uniform; nothing auto-recovers into motion. |
| D17 | SAFE_STOP -> ramp to zero velocity -> HOLD (active position hold under a reduced torque cap). Watchdog timeout takes the same path, then latches FAULT(WATCHDOG) while still holding. Coast only on ESTOP or DISARMED. | A chair that coasts on a hill after a safe stop rolls away; a held chair is recoverable by power-off. |
| D18 | Board state machine: DISARMED (Hi-Z) / ARMED (runs `mode`) / SAFE_STOPPING (transient, visible in telemetry) / HOLDING / FAULT. ESTOP = immediate Hi-Z, held while requested. `mode` + setpoints are ignored unless ARMED. | |
| D19 | Limit value 0 = the firmware hard ceiling, not a tuned soft value. `accel_limit = 0` = no ramp at all (velocity reference steps; the current limit does the rest). Anything softer is the MIB planner's job. | |
| D20 | Topic naming from one X-macro axis table: commands `rammp/mib/motor_command/<axis>` (type `rammp/msg/MotorCommand`), telemetry `rammp/<axis>/motor_state` (type `rammp/msg/MotorState`), `kAxes[]` / `axis(AxisId)` typed handles. (`mib`, not `mcb`: matches PR #4's `rammp/mib/status`.) | Adding an axis = one table row. |
| D21 | Sequencing: `uint8_t seq` on both messages (joystick convention); `MotorState` also carries `uptime_ms`, `last_cmd_seq`, `cmd_age_ms`. No synchronized clock in v1. | |
| D22 | Faults on the wire: `fault_code : uint8` enum (NONE, WATCHDOG, OVERCURRENT, VDS_OCP, OVERTEMP, BUS_OVERVOLT, BUS_UNDERVOLT, ENCODER, DRV_FAULT, UNASSIGNED_AXIS, ...) + `drv_status : uint16` raw DRV8353 bits + `temps[4]`. | Enum for machine decisions, raw bits for humans. |
| D23 | Jetson reads the same topics through espp/rtps itself (it builds on Linux: POSIX sockets, `std::thread`) or its Python bindings, including `motor_message.hpp` directly. No IDL. Not ROS 2-visible in v1. | One serializer for every peer; espp/rtps lacks ROS 2 name mangling. Revised 2026-09-16: the IDL twin was dropped from PR #5. |
| D24 | Firmware home: new app `components/pace-racer-board/atos-drive/`, forked from `halls-sensorless` (encoder-commutated drive path), `rammp-rtps` as a git submodule, console kept for provisioning/cal/tuning. | |
| D25 | CLEAR_FAULT = `clear_fault_req_id : uint8` in `MotorCommand` (+1 per request, wraps); the board clears on a *new* value and echoes it as `MotorState.clear_fault_ack`. The `ActuatorCommand.req_id` idiom. | Survives packet loss, one clear per request, MIB can confirm, cannot mask a recurring fault. |
| D26 | Sign convention: CCW at the shaft is positive, right-hand rule, for position, velocity and torque on every axis. CCW as seen from outside the chair looking at the wheel face, right-hand thumb pointing outboard along the axle. Driving forward the left wheel is positive and the right wheel negative. No per-board direction flag; the MIB negates mirrored wheels. | One physical rule, no hidden provisioning. |
| D27 | Named setpoint fields `position` / `velocity` / `torque`; unused ones are zero. | Engineer clarity over 8 bytes. |
| D28 | HOLD = position hold at the position at HOLD entry, under `torque_limit`. Mode changes while ARMED take effect immediately, with controller integrators reset; no bumpless transfer in v1. | |
| D29 | `MotorState` also carries **`ibus_est`** (bus current estimate) and **`api_version`** (from `RAMMP_MOTOR_API_VERSION` in the header). | Per-wheel power for the Jetson; schema mismatch visible in telemetry rather than as wrong floats. |
| D30 | MIB command rate contract: 10 Hz <= rate <= 50 Hz. Lower bound is the 200 ms watchdog; upper bound is the S3 packet budget (~100 pps total incl. 20 Hz telemetry). Ceiling to be measured on the bench with the stand-in before the MIB team fixes its loop rate. | |
| D31 | A host-side Python MIB stand-in is an explicit, separate deliverable (publishes `MotorCommand`, decodes `MotorState`, in the `tests/rtps_sub.py` style). | Testable on the dyno before the P4 exists; reference for the MIB engineer. |
| D32 | `vbus` / `ibus_est` stay in `MotorState`; `vbus` reports the compiled constant until hardware gains a VM divider, flagged by `vbus_measured = 0`. No BUS_OVERVOLT/UNDERVOLT fault codes in v1. | Wire format stable across the hardware rev; a consumer cannot mistake the constant for a reading. |
| D33 | Feedback source is per-board NVS (`fb hall|enc`), default hall. The arm gate checks the selected source's calibration. Position on halls is the step accumulator (4 deg on 15 pp); hold has a one-step deadband. | The chair has no encoder; the dyno does. One firmware, one switch, no build flag. |
| D34 | `vel_limit` binds in every mode as a torque governor: in TORQUE the roll-off band is the top 20 % below the limit; in VELOCITY/POSITION/SAFE_STOP the band sits above the limit, so the loop reaches its clamped setpoint but a released stall or wound integrator is cut and braked. | A torque command must never be able to run a wheel away; 20 % is wider than the hall speed noise at chair speeds. |
| D35 | A motor profile may carry an acceleration ceiling (`kMaxAccelRpmS`); where it does, `accel_limit` 0 or larger means the ceiling (D19 still holds where it is 0). The flatbot profile sets 120 rpm/s, 60 rpm, 10 A, 4 A hold. | Office ceilings the MIB cannot raise; raising them is a deliberate reflash. |
| D36 | Feedback guards raise `HALL` (illegal input 50 ms) and `STALL` (speed integrator past half the cap and no net hall steps over 0.5 s, speed/position modes only). Both exit through the unload-then-coast path. Verified physically 2026-09-25. | A lost hall line or a blocked wheel must be a named fault, not a hot motor. |
| D37 | **Revises D9 (2026-10-09).** Outer-loop tuning over the network: a third per-axis topic `rammp/mib/motor_tuning/<axis>` carries `MotorTuning` (`tuning_req_id`, `speed_kp`, `speed_ki`, `hall_pll_hz`, `persist`); the board acts on a new req_id, clamps to compile-time bounds, applies live without an integrator reset, echoes `tuning_ack` and the values in force in `MotorState`, and saves to NVS only when `persist` is set. API version 2. Current loop, ceilings and calibration stay off the wire. | The MIB drove the flatbot from the joystick (Oct 2026) and it vibrates; the gains were console-only, which means a laptop on the robot for every iteration. The per-command limits cannot fix a loop tune. Separate topic so the 50 Hz command stays 28 bytes; req_id so a best-effort resend is harmless; `persist` opt-in so a bad experiment dies at reboot. |
| D38 | **Extends D37 (2026-10-19).** A negative `speed_kp` / `speed_ki` in `MotorTuning` sets that gain to exactly 0; 0 (and NaN) still means unchanged; positive values clamp as before, the Kp floor lowered to 0. With ki = 0 both speed loops hold their integrator at 0 (bumpless seeds included), so switching it off also drops the current it held. Same wire layout; API version 3, fw 0.3.0. | Switching a term off is a basic controller operation, and the v2 floor-by-sentinel left only a tiny positive Ki, which keeps the integrator's stored current forever as an offset. Negative rather than redefining 0, so existing senders that send 0 for "leave as is" don't silently zero a gain; a v2 board already ignored negatives. |
| D39 | **Revises D13 and the hub20 ceilings (2026-10-29).** `MotorState` publishes at 50 Hz (`motor::kStateHz`). hub20: `kMaxAmps` 30 A (46.8 N.m, above the 22 A datasheet peak) as a burst ceiling under an I^2*t budget worth 2 s at 30 A above a 15 A continuous level; the ceiling folds linearly to 15 A over the last quarter of the budget and `torque_limit_eff` reports it live, together with the console `lim` target. Boot target/trip 30/35 A; DRV VDS OCP at 0.20 V (board protection only). Flatbot and ninebot unchanged (no budget, VDS 0.06 V). Same wire layout and API version; fw 0.4.0. | Chair testing: the MIB dashboard needs denser logs to tune the speed loop, and turning in place spiked past 15 A into a VDS OCP latch. Brief spikes deserve the board's full current; sustained current is limited by the bare boards (15 A warm, no heatsinks). A software limit folds back gently where a VDS trip latches the bridge off. 50 Hz x 2 chair boards stays inside the ~70 Hz RTPS ceiling. |
| D40 | **Revises D19 and D35 for hub20 (2026-11-01).** hub20 gets an acceleration ceiling, `kMaxAccelRpmS` 150 rpm/s (15.7 rad/s^2, 2.0 m/s^2 at the rim), so `accel_limit` 0 or larger means 150 rpm/s. New per-profile `kIqSlewAPerMs`: the iq reference out of the hall drive (speed, position and torque sub-modes, after the speed governor) may grow in magnitude by at most that many A/ms; any move toward zero passes at once, a sign change drops to zero and rises from there. hub20 2 A/ms, flatbot and ninebot 0 (off). While the slew holds iq short of the speed/position loop's output, the integrator keeps its previous value whenever the new step would push it further (conditional integration). Console `isl <A/ms>` up to 10 A/ms; `isl 0` (off) only where the profile has none. SAFE_STOP keeps `kSafeStopRampRpmS` 200 rpm/s; HOLDING and POSITION have no setpoint ramp. The stop paths (STOPPING unload after a trip, ESTOP/DISARMED idle and coast, thermal, stall, hall) never pass through the slew. Same wire layout and API version; fw 0.4.1. (D35's flatbot 120 rpm/s ceiling was dropped 2026-09-28, f4746a2: it lagged the planner.) | Chair testing (Nov 1): VDS OCP and software overcurrent trips while turning, at the switch from driving forward into a sharp turn, where the inside wheel's setpoint reverses. The MIB streams VELOCITY at 20 Hz with accel_limit 0, so the speed reference stepped every 50 ms and nothing bounded how fast the speed loop could move iq. The ceiling is 1.28x the MIB NORMAL worst case per wheel (1.0 m/s^2 linear + 2.0 rad/s^2 yaw = 12.3 rad/s^2, 117 rpm/s), so a planned profile is not slowed (each 50 ms step is reached inside its period); only a step the planner did not shape is. The slew: the current loop is first order at 169 rad/s (tau 5.9 ms), so a full 30 A step already rises at 5 A/ms; 2 A/ms (30 A in 15 ms, L di/dt 1.6 V) makes it a ramp the loop follows, and is still far faster than the speed loop and the MIB period. Rising only, because a fast decrease is what unloads the drive: slowing it would hold current longer and delay a brake. Torque mode included because the slew protects the board whoever asks, and a full rise fits inside one command period. |
| D41 | **Soft-trip rule per profile (2026-11-01).** The software overcurrent trip is a `foc::TripRule` (`main/trip_gate.hpp`, integer, in the sampler ISR). Sustained: the filtered current of any phase (A, B or the reconstructed C) over the trip level for `kTripSamples` ISR samples, consecutive or, with `kTripLeaky`, net (+1 per sample over, -1 per clean one). Fast: `kTripFastConversions` consecutive raw conversions of one measured phase past the plausibility gate (trip + 3 A), which the filter otherwise holds and the sustained check can never see. hub20: 20 net samples (1 ms at 20 kHz) and a 4-conversion fast path (400 us); trip level stays 35 A, so the order target 30 < trip 35 < plausibility 38 < ADC rail ~40 A holds; DRV8353 VDS OCP 0.20 V is the hardware backstop. Flatbot and ninebot unchanged: 3 consecutive samples, no fast path. Peak budget, thermal guard and the D40 slew unchanged. Same wire layout and API version; fw 0.4.2. | Chair test (Nov 1): drive_left soft-tripped on the reconstructed phase C (-35.2 A from filtered A +22.3, B +12.9) at ~25 A real, 38 rpm, with the q-axis voltage at its 16 V limit. Phase A's conversions alternated between ~24 A and ~2-6 A on consecutive conversions (not physical: 804 uH gives ~2 A per sample), consistent with the highest-duty phase's low-side window shrinking to a few us so that a late ISR converts outside it and reads ~0 A; the slew-gate filter then holds the wrong level. Three consecutive samples (150 us) cannot ride that out. A trip on the filtered vector magnitude would not help: the same filtered values give 35.6 A. A net 1 ms count rejects bursts and alternating misreads but still trips a real overcurrent within ~1-1.2 ms; the fast path keeps a short above the gate visible. The trip cannot move up: the gate must stay under the ADC rail. The likely root cause (measure the two lowest-duty phases, console `p auto`) is a separate chair test. |
| D42 | **Revises the hub20 VDS level of D39 (2026-11-01).** hub20 boots with the DRV8353 VDS OCP at 0.30 V (level 6), was 0.20 V; deglitch 4 us and latched shutdown unchanged; `vds` still changes it live until the next `clr`. Flatbot and ninebot unchanged (0.06 V). fw 0.4.3. | Across the ISC0802NLS (2.7 typ / 3.6 max mOhm at 25 C, ~1.5x at 100 C, ~1.8x at 150 C), 0.20 V trips at 56-74 A cold but 37-50 A at 100 C and 31-41 A at 150 C, at or under the 35 A soft trip and inside a legitimate 30 A burst, so hot boards could latch off mid-turn. 0.30 V is 83-111 A cold, 56-75 A at 100 C, 46-62 A at 150 C: above the soft trip and the ~40 A ADC rail at every temperature, so the order cap 30 < trip 35 < gate 38 < rail ~40 < VDS holds. Not higher: above the rail the software only starts a ~100 ms closed-loop unload (D41 fast path), so VDS is the only fast cut-off there, and 0.40 V+ widens that band to ~150 A cold without a check of the FETs' pulse rating and SOA. 0.30 V already ran on the dyno (1 kW campaign). |
| D43 | **Revises the hall rest hold (797a07f) and the D41 fast trip (2026-11-01).** Rest hold (`main/rest_hold.hpp`): the anchor is taken only once the speed reads under `kStoppedRpm` AND no hall step has come for 0.25 s; the pull-back is 1 rpm per hall step beyond the +-1 step deadband (clamped at 10 rpm), was `pos_kp` x error = a 10 rpm kick at 2 steps. The slip latch (bleed off, slope current kept) is unchanged. HOLDING/POSITION (`HALL_POS`) is unchanged. Fast trip: 10 consecutive conversions of one measured phase with the raw ADC count within 8 counts (~0.2 A) of 0 or 4095 (railed), was 4 past the plausibility gate. Sustained 1 ms rule (D41), VDS 0.30 V (D42) unchanged. fw 0.4.4. | Chair (Nov 1, 669-675 s): after a stop from a spin the speed loop overshot through zero, the hold anchored at the crossing, the wheel rolled on 2 steps and that latched as a slip; then an on/off pull-back (10 rpm at 2 steps, 0 at 1) acting on a speed that is a 10 Hz tracking loop on 3 deg steps (~0.2 s late, flat +-0.5 rad/s pulses with no position change) limit-cycled at +-0.5 rad/s, +-5 A, 0.65 s until Hi-Z. Waiting for the hall steps to stop keeps the overshoot from counting as a slip; a proportional pull-back removes the relay at the deadband edge while a real slip is still pulled back and its slope current kept. A host sim did not reproduce the exact cycle, so it is not proven until the chair. Fast trip: drive_left tripped on 4 conversions at 38-40 A (gate 38, rail ~40) during a full-speed reverse arc at ~33 A phase current, 5 implausible readings in the whole session; a real short rails the ADC on every conversion, and VDS stays the microsecond backstop. |


## Wire types

Source of truth: `rammp-rtps` `components/rammp_rtps_messages/include/messages/motor_message.hpp`
(PR #5). Every peer, the Jetson included, includes this header; there is no IDL.
`Topic<Message>` moved to `messages/topic.hpp` so both message headers share it.

```cpp
#define RAMMP_MOTOR_API_VERSION 1
// RAMMP_AXIS_TABLE -> AxisId { FRONT_CASTER_LEFT=0, FRONT_CASTER_RIGHT=1, DRIVE_LEFT=2,
//                              DRIVE_RIGHT=3, UNASSIGNED=255 }, kAxes[], axis(AxisId)
// RAMMP_TOPIC_MIB_MOTOR_COMMAND(seg)  "rammp/mib/motor_command/<seg>"  rammp/msg/MotorCommand
// RAMMP_TOPIC_MOTOR_STATE(seg)        "rammp/<seg>/motor_state"        rammp/msg/MotorState

enum class RequestedState : uint8_t { DISARMED, ARMED, SAFE_STOP, ESTOP };
enum class ControlMode    : uint8_t { COAST, TORQUE, VELOCITY, POSITION, HOLD };
enum class BoardState     : uint8_t { DISARMED, ARMED, SAFE_STOPPING, HOLDING, FAULT };
enum class FaultCode      : uint8_t { NONE, WATCHDOG, OVERCURRENT, VDS_OCP, OVERTEMP,   // + HALL = 9, STALL = 10 (2026-09-25)
                                      ENCODER, DRV_FAULT, UNASSIGNED_AXIS };

struct MotorCommand {            // 28 B on the wire, 10..50 Hz
  uint8_t seq; RequestedState requested_state; ControlMode mode; uint8_t clear_fault_req_id;
  float position, velocity, torque;                 // rad, rad/s, N.m
  float torque_limit, vel_limit, accel_limit;       // 0 = ceiling / ceiling / no ramp
};

struct MotorState {              // 80 B on the wire (2 B padding), 50 Hz (D39)
  uint8_t api_version; AxisId axis_id; uint8_t seq, last_cmd_seq;
  BoardState state; ControlMode mode; FaultCode fault_code; uint8_t clear_fault_ack;
  uint8_t vbus_measured, flags;
  uint16_t cmd_age_ms, drv_status;
  uint32_t uptime_ms, fw_version;                   // fw: major<<24 | minor<<16 | patch
  float position, velocity, torque_est, iq, id, vbus, ibus_est;
  float torque_limit_eff, vel_limit_eff, accel_limit_eff;
  std::array<float, 4> temps;
};
```

## Behavior contract

### State machine

```
              requested_state           events
DISARMED  --ARMED-------------------> ARMED
ARMED     --SAFE_STOP---------------> SAFE_STOPPING --(v==0)--> HOLDING
ARMED     --watchdog (200 ms)-------> SAFE_STOPPING --(v==0)--> HOLDING + FAULT(WATCHDOG)
any       --ESTOP-------------------> DISARMED-like Hi-Z, held while ESTOP is requested
any       --DISARMED----------------> DISARMED (Hi-Z coast)
any       --board guard trips-------> FAULT (gentle unload -> Hi-Z, existing path)
FAULT     --new clear_fault_req_id--> DISARMED (then MIB requests ARMED)
HOLDING   --ARMED-------------------> ARMED (resumes running `mode`)
```

- `mode` and setpoints are honored only in ARMED. Mode switches are immediate with
  integrator reset. HOLD captures the position at entry.
- Watchdog FAULT holds position (reduced torque cap) rather than coasting. A chair
  stopped on a hill must not roll (D17).
- A board with `axis_id == UNASSIGNED` subscribes to nothing, publishes `MotorState`
  with `FAULT(UNASSIGNED_AXIS)` on no topic, so it is inert until provisioned.
  (It still logs to the console so an installer can see why.)

### Limits

| field | 0 means | clamp |
| --- | --- | --- |
| `torque_limit` | firmware hard ceiling (Kt x trip current) | compile-time ceiling |
| `vel_limit` | firmware speed clamp (+/-520 rpm today) | compile-time ceiling |
| `accel_limit` | **no ramp** (reference steps), or the profile's accel ceiling where one exists (hub20: 15.7 rad/s^2, D40) | compile-time ceiling |

Effective values are echoed in `MotorState.*_limit_eff`.

### Rates

| stream | rate | QoS |
| --- | --- | --- |
| `MotorCommand` (MIB -> board) | 10 Hz min (watchdog), 50 Hz max | best-effort, keep-last-1 |
| `MotorState` (board -> MIB/Jetson) | 50 Hz fixed (20 Hz before D39) | best-effort, keep-last-1 |

Budget: 4 boards x (50 pub + <=50 sub; was 20 pub before D39) stays under the ~100 pps per-board RTPS ceiling
measured in `rtps-telemetry-results`; verify with the stand-in (see Bench verification).

### Units and frames

SI at the motor output shaft: rad (multiturn from boot), rad/s, N.m (Kt x iq, Kt =
0.60 N.m/A measured). CCW at the shaft is positive by the right-hand rule on every axis (seen from outside the chair, thumb outboard);
the MIB negates mirrored wheels. The board hides the MT6701 2.5x gearing and its
calibration offset.

### Network

Board is a DHCP client (MIB serves DHCP). The RTPS participant starts in `on_got_ip`
with `interface_address` set to the leased IP (espp cannot auto-detect it) and stops on
`on_ip_lost`; command loss then lands in the watchdog path naturally. SPDP/SEDP
multicast discovery on the chair switch is a day-one verification item.

## Implementation plan

### Phase 1: `rammp-rtps` PR (`feat/motor-messages`), drafted 2026-09-16

1. `motor_message.hpp` (wire types above), `topic.hpp` (shared `Topic<>`), `messages.hpp`
   include, README layout + how-to. Compiles clean under clang++ -std=c++20 -Wpedantic with
   static asserts on topic names and sizes; passes the repo's clang-format pre-commit hook.
2. Open as https://github.com/rammp-org/rammp-rtps/pull/5 (commit 088454c). An IDL twin was
   in the first push and removed: espp/rtps runs on the Jetson, so nothing consumes it. Conflicts with
   PR #4 on one include line in `messages.hpp`.

### Phase 2: firmware app `components/pace-racer-board/atos-drive/`, drafted 2026-09-16

Built clean (ESP-IDF v6.0, -Wall -Werror, 0 warnings, 961 KB vs 899 KB parent, 8% partition
free). Not flashed, not on the wire. See the app README. Deviations from the plan below:
`reset_controllers` re-seeds the position derivative only (the current-loop PI is never reset
while driving, per SENSORLESS_FOC.md s9); the coast SPI write happens on the 100 Hz supervisor
tick, not the 10 Hz stream task; `FaultCode::SAMPLER` was added to the wire enum for the
sampler-overrun guard.

#### The chair motor (added 2026-09-16)

Hub motor, direct drive: 20 pole pairs, 48 V, 17 N.m / 7.5 A rated, 51 N.m / 22 A peak,
150 rpm rated / 180 rpm peak (no-load; Ke x 180 rpm = 47 V, so under load the voltage limit
saturates the speed loop below that), R 0.7 ohm, L 1.885 mH, Ke 0.262 V/rpm (+/-10%), Kt
derived 2.3 N.m/A, 11.7 kg. Wheel diameter 358 mm (1.125 m circumference; 150 rpm = 2.8 m/s).
Encoder: 4096 CPR quadrature, A/B differential (RS-422 receiver needed in front of the S3;
PCNT decodes x4 = 16384 counts/rev). Halls present. Consequences: `motor_params.hpp` carries
both motors; ceilings for the hub are 180 rpm / 22 A; kHoldAmps = 10 A (23 N.m per wheel,
holds ~10 deg at 150 kg, inside the 15 A sustained thermal limit). The incremental encoder
has no absolute angle at boot: plan is hall-edge sync (commutate on halls until the first hall
transition, capture the count, switch to encoder angle), which replaces `ecal` for that motor;
the hall table then persists in NVS instead of the encoder offset. Open: which GPIOs, and
whether v1 drives on the encoder at all or halls-only (round-5 questions 1-4).

Original plan:

Fork of `halls-sensorless` (encoder-commutated `erun`/`epos` path is the drive).

1. `rammp-rtps` as a git submodule; include `rtps_interface.hpp`.
2. NVS: `axis_id` (default UNASSIGNED) + console `axis <n>` / `axis` to provision/show.
3. `atos_link.hpp` replaces `rtps_telem.hpp`: one `Subscriber<MotorCommand>` on the
   board's own command topic, one `Publisher<MotorState>` at 50 Hz (D39; was 20) (absolute-deadline
   task pattern from `rtps_telem.hpp`), lifecycle tied to `on_got_ip` / `on_ip_lost`.
4. `drive_supervisor.hpp`: the state machine of the Behavior contract, the 200 ms
   watchdog, `clear_fault_req_id` edge detection, limit clamping, mode dispatch into the
   existing controllers (iq / velocity / position P(D) / hold).
5. Multiturn position accumulation from the MT6701 (unwrap in the 1 kHz encoder task,
   store as int64 counts, convert to rad at publish).
6. `ibus_est`, `torque_est`, `drv_status` sampling; `fw_version` from the build.
7. Console remains for `ecal`, `hset`, limits, `sq`, etc. Console commands that start
   motion are refused while an MIB command stream is live (avoid two masters).

### Phase 3: host-side MIB stand-in (see next section)

### Phase 4: bench verification (see Bench verification)

## Host-side MIB stand-in (Python), a separate deliverable

Lives at `components/pace-racer-board/atos-drive/tools/mcb_standin.py`. Purpose: drive a
board over the real wire format before the P4 exists, and serve as the reference the MIB
engineer diffs their implementation against.

- Publishes `MotorCommand` on `rammp/mib/motor_command/<axis>` at a chosen rate
  (10-50 Hz), subscribes to `rammp/<axis>/motor_state`. Uses the espp Python bindings
  (`espp` module, see espp `python/rtps_rpc_demo.py`) with `motor_message.hpp`, so the
  stand-in and the boards share one serializer. Fallback if the bindings fight us on
  the Jetson or Mac: the hand decoder pattern in `halls-sensorless/tests/rtps_sub.py`.
- CLI/TUI: arm/disarm/estop/safe-stop keys, mode select, live setpoint entry, a
  `clear-fault` key that bumps `clear_fault_req_id`, and a scripted mode (`--script`)
  that replays a setpoint timeline for repeatable dyno runs.
- Displays the decoded `MotorState` at 20 Hz with `cmd_age_ms`, `state`, `fault_code`,
  and flags `api_version` mismatches.
- A `--stall` flag stops publishing to exercise the watchdog path on purpose.
- Records runs the same way `monitor.py` does (`logs/run-*/data.csv`) so dyno results
  compare like for like with the campaign data.

## Bench verification

Run on the dyno rig ([[dyno-bench-rig-control]]) with the stand-in:

1. Discovery: board obtains DHCP lease from a DHCP server on the switch, participant
   comes up, stand-in matches both endpoints (SPDP through the switch).
2. Command ceiling: sweep stand-in rate 10 -> 100 Hz; record `late%` on the sampler
   and delivered `MotorState` rate. Confirm 50 Hz is inside the budget with margin;
   publish the measured ceiling to the MIB team.
3. Watchdog: `--stall` under load -> SAFE_STOPPING -> HOLDING + FAULT(WATCHDOG),
   position held against the brake; clear via `clear_fault_req_id`; re-arm.
4. Each mode under brake load: TORQUE, VELOCITY (accel_limit 0 and non-zero),
   POSITION (step and streamed), HOLD; check `*_limit_eff` clamping at the ceilings.
5. Two boards on one switch (the second bench board), distinct `axis_id`, confirm no
   cross-talk and 2 x 20 Hz telemetry.
6. Binary-size check: `late%` with the participant not started vs started (the
   [[rtps-telemetry-results]] confound).

## Risks / watch items

- Multicast on the chair switch: if SPDP is filtered, discovery fails silently.
  Mitigation: verify on day one; espp/rtps supports unicast peer lists as a fallback.
- The packet budget with 4 boards is per-board, but the MIB receives 80 pps + sends
  up to 200 pps; fine for a P4, but confirm the MIB's RTPS stack handles 8 endpoints.
- No bus-voltage sense: bus faults are invisible to the board; the MIB must own
  battery monitoring. `vbus_measured = 0` makes this explicit on the wire.
- HOLD under a dead master holds indefinitely at the reduced torque cap; thermal
  guard is the only exit besides power-off. The reduced cap must be chosen so an
  indefinite hold at stall is thermally safe (campaign data: 15 A sustained warm).
- Multiturn position resets at boot: any MIB logic that assumes persistence is
  wrong by design (D15).
