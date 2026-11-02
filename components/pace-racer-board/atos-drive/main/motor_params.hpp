#pragma once

// Motor and drive constants the ATOS contract depends on. Everything a
// MotorCommand limit of 0 resolves to, and every unit conversion between the
// wire (rad, rad/s, N.m at the shaft) and the FOC atomics (rpm, amps, rotor
// degrees), reads from here.
//
// Three profiles: the NineBot S on the dyno (default), which the whole FOC
// stack was validated on; the same motor with office ceilings for the flatbot
// rover (ATOS_MOTOR=flatbot); and the wheelchair hub motor the chair ships with
// (ATOS_MOTOR=hub20). The profile is a compile-time choice on purpose: a
// MotorCommand can only lower these limits, never raise them.
//
// Hub motor numbers are from its datasheet (2026-09-16) and NOT yet measured on
// this board: R/L via 'rl', Kt via a torque-sensor sweep, and the current-loop
// gains 'g <kp> <ki>' all have to be redone before the chair drives on them.

#include <cstdint>

namespace motor {

#if defined(ATOS_MOTOR_FLATBOT)
// The office rover: two NineBot S wheels, no encoder, halls only. Same motor
// as the dyno block below but with ceilings the MIB cannot raise: walking
// pace and two thirds of the current. No acceleration ceiling: the planner
// streams setpoints and owns the ramp through accel_limit (0 = step). Raising
// any of these is a reflash on purpose. 60 rpm on the 267 mm wheel is 0.84 m/s.
inline constexpr const char *kName = "flatbot";
inline constexpr int kPolePairs = 15;
inline constexpr float kKt = 0.60f;
inline constexpr float kMaxRpm = 60.0f;   // vel_limit = 0 and hard ceiling
inline constexpr float kMaxAmps = 20.0f;  // torque_limit = 0 and hard ceiling (12 N.m)
inline constexpr float kHoldAmps = 4.0f;  // HOLDING cap, ~2.4 N.m
inline constexpr float kBootTargetAmps = 8.0f; // 'lim' at boot: the drive's real |iq| cap
inline constexpr float kBootTripAmps = 15.0f;  // soft overcurrent trip at boot
inline constexpr float kContAmps = kMaxAmps;   // no peak budget: kMaxAmps is continuous
inline constexpr float kPeakSeconds = 0.0f;
inline constexpr int kBootVdsLevel = 0;        // DRV8353 VDS 0.06 V, ~17-22 A cold
inline constexpr float kSpeedKpEnc = 0.05f;  // speed-loop Kp, A/rpm, encoder feedback
inline constexpr float kSpeedKpHall = 0.25f; // speed-loop Kp, A/rpm, hall feedback
inline constexpr float kSpeedKiHall = 0.5f;  // speed-loop Ki, A/rpm-s, hall feedback
inline constexpr float kHallPllHz = 10.0f;   // hall speed tracking loop, Hz
inline constexpr float kMaxAccelRpmS = 0.0f; // no accel ceiling: accel_limit 0 = step
inline constexpr float kIqSlewAPerMs = 0.0f; // no iq slew limit (D40)
// Soft trip rule (D41): 3 consecutive samples over kBootTripAmps, no fast path.
inline constexpr int kTripSamples = 3;
inline constexpr bool kTripLeaky = false;
inline constexpr int kTripFastConversions = 0;
inline constexpr float kObsR = 0.323f;
inline constexpr float kObsL = 336.0e-6f;
inline constexpr float kObsLambda = 0.026f;
inline constexpr float kBusVoltage = 48.0f;
inline constexpr float kDefaultKp = 0.0568f;
inline constexpr float kDefaultKi = 54.5f;
inline constexpr float kDefaultVoltLimit = 8.0f; // back-EMF at 60 rpm is 2.5 V, 3x margin
#elif defined(ATOS_MOTOR_HUB20)
// Wheelchair hub motor: 48 V, 17 N.m / 7.5 A rated, 51 N.m / 22 A peak, 150 rpm
// rated / 180 rpm peak, 4096 CPR quadrature encoder (A/B differential), halls.
// R, L, lambda and the current-loop gains below are MEASURED on this drive
// (2026-09-24, board 8c:fd:49:a2:de:b0, 48 V): the datasheet's 0.7 ohm /
// 1.885 mH / Ke-derived lambda 0.077 were 30-50 % off, and at 0.077 the
// observer never locked (flux pinned at the parameter). See hub_motor_cal.py
// in halls-sensorless for the procedure.
inline constexpr const char *kName = "hub20";
inline constexpr int kPolePairs = 20;
// N.m/A = 1.5 * pp * lambda. 17 N.m at 10.9 A peak = 7.7 A RMS, which is the
// datasheet's rated 7.5 A (RMS) within 3 %.
inline constexpr float kKt = 1.56f;
inline constexpr float kMaxRpm = 180.0f;  // vel_limit = 0. No-load figure at 48 V.
// torque_limit = 0: the burst ceiling, 46.8 N.m. Above the datasheet's 22 A
// peak on purpose (2026-10-29): turning in place spikes past 15 A and the chair
// wants all the board can give. The board ran 30 A stalls on the dyno.
inline constexpr float kMaxAmps = 30.0f;
// Peak budget (supervisor): an I^2*t budget above kContAmps^2 worth
// kPeakSeconds at kMaxAmps. Full ceiling for the first 3/4 of it, then a
// linear fold to kContAmps: from rest, 30 A for 1.5 s, then under 16 A by
// ~3 s; 22 A holds ~4.5 s, 20 A ~7 s. Refills below kContAmps (full ~6 s
// after stopping). 15 A sustained is
// the measured warm limit of the bare board (no heatsinks on the chair).
inline constexpr float kContAmps = 15.0f;
inline constexpr float kPeakSeconds = 2.0f;
inline constexpr float kHoldAmps = 10.0f; // HOLDING cap: ~16 N.m; 15 A sustained is
                                          // the measured warm thermal limit.
// 'lim' at boot: the ATOS hooks clamp every current to the target, so it
// matches the burst ceiling. The software trip is the real overcurrent guard,
// 5 A over the bursts (the dyno's 35 A guard; plausibility gate at 38 A, inside
// the ADC rail). VDS OCP only protects the board itself (D42): 0.30 V across
// the ISC0802NLS (2.7 typ / 3.6 max mOhm cold, ~1.5x at 100 C, ~1.8x at 150 C)
// is ~83-111 A cold, ~56-75 A at 100 C, ~46-62 A at 150 C, so even hot it sits
// above the 35 A trip and the ~40 A ADC rail. 0.20 V fell to ~31-41 A with hot
// FETs, inside a legitimate 30 A burst. Higher levels would leave a wider band
// above the rail where VDS is the only fast cut-off.
inline constexpr float kBootTargetAmps = 30.0f;
inline constexpr float kBootTripAmps = 35.0f;
inline constexpr int kBootVdsLevel = 6;
// Speed-loop Kp (A/rpm), half the dyno/rover values: those made the chair
// jitter (2026-10-29). Ki is unchanged, it supplies the breakaway current.
inline constexpr float kSpeedKpEnc = 0.025f;
inline constexpr float kSpeedKpHall = 0.125f;
inline constexpr float kSpeedKiHall = 0.5f;
// Hall speed tracking loop, Hz, and the per-wheel hall speed gains the chair drives on
// (2026-11-01). The driver's tune, auto-tuned per wheel and then taken down 15 % when
// 'vl' went to 22 V: at 16 V the voltage clamp had been quietly limiting these gains, and
// at full strength they limit-cycled the loaded chair. 10 Hz tracking with the 0.125 / 0.5
// defaults felt nervous to the driver; 2 Hz with these is what he chose. kSpeedKpHall /
// kSpeedKiHall above stay the fallback for an axis without its own entry.
inline constexpr float kHallPllHz = 2.0f;
inline constexpr float kHallSpeedKpLeft = 0.8011f, kHallSpeedKiLeft = 0.7121f;   // drive_left
inline constexpr float kHallSpeedKpRight = 0.9792f, kHallSpeedKiRight = 0.8011f; // drive_right
inline constexpr float kObsR = 0.51f;     // ohm/phase, 'rl' at 0.6/1.0/1.5 V steps
inline constexpr float kObsL = 804.0e-6f; // H/phase (the sheet's 1.885 mH is ~2x phase)
// Wb. Observer locks 0.045..0.052 (rpm_est = rpm_hall within 0.2 % from 60
// down to 7 rpm), unlocks at 0.060. NOT zero: the observer fail-safe band is
// 0.35*lambda..3*lambda, which with lambda = 0 can never be satisfied.
inline constexpr float kObsLambda = 0.052f;
// Bus, V. SVPWM scales duties by this, so it must be the real bus or every
// commanded volt lands short. 45 V was the bench series supply; the chair's
// 48 V pack sits 46-50 V, re-check with the 'mi' line against the pack.
inline constexpr float kBusVoltage = 45.0f;
// Current-loop PI at the validated 169 rad/s closed-loop bandwidth
// (Kp/L = 169, Ki/Kp = R/L), from the measured R and L.
inline constexpr float kDefaultKp = 0.136f; // V/A
inline constexpr float kDefaultKi = 86.0f;  // V/(A*s)
// 'vl' boot default. Back-EMF is omega_e * lambda = 4.8 V at 30 rpm and 9.6 V
// at 60, so the NineBot's 8 V clamp would stall the I/f spin outright.
// Raised 16 -> 22 V (2026-11-01): at 16 V a wheel held at the 30 A ceiling ran out of
// voltage near 3 rad/s (30 A x 0.51 ohm + back-EMF is ~18-20 V), lost control of the
// current and tripped the 35 A soft trip on hard turns at speed. 'vl' ceiling is
// kMaxPhaseVolts (~24 V at kBusVoltage 45).
inline constexpr float kDefaultVoltLimit = 22.0f;
// Acceleration ceiling (D35, D40): accel_limit 0 or larger means this. The MIB
// streams VELOCITY at 20 Hz with accel_limit 0, so without it the reference
// stepped every 50 ms. It must sit above what the planner asks for or the chair
// lags the stick: the NORMAL worst case per wheel, 1.0 m/s^2 linear plus
// 2.0 rad/s^2 yaw on the 0.558 m track, is 12.3 rad/s^2 on the 0.254 m wheel
// (117 rpm/s). 150 rpm/s (15.7 rad/s^2, 2.0 m/s^2 at the rim) is 1.28x that:
// each 50 ms planner step is reached inside its period, and 0 to 180 rpm takes
// 1.2 s. SAFE_STOP keeps kSafeStopRampRpmS (200 rpm/s): a stop may be firmer.
inline constexpr float kMaxAccelRpmS = 150.0f;
// iq reference slew, A/ms, rising |iq| only (D40). The current loop is first
// order at 169 rad/s (tau 5.9 ms): a full 30 A step starts at 5 A/ms and lands
// in ~18 ms. 2 A/ms turns the step into a ramp the loop follows (0 to 30 A in
// 15 ms, ~2.5 tau; L di/dt = 1.6 V of the 16 V 'vl'), 0.1 A per 20 kHz tick,
// and stays far faster than the few-Hz speed loop and the 50 ms MIB period.
inline constexpr float kIqSlewAPerMs = 2.0f;
// Soft trip rule (D41). The chair tripped (2026-11-01) at ~25 A real on a
// reconstructed phase C of -35 A: at high modulation the highest-duty phase's
// low-side window is a few us, an ISR that enters late converts outside it and
// reads ~0 A, and the slew-gate filter latched onto those reads. Three
// consecutive samples (150 us) is too short to ride that out. So: a net 1 ms
// over the trip (leaky count, 20 samples at 20 kHz, +1 over / -1 clean), or
// at once on 10 consecutive conversions of one measured phase with the ADC
// railed (1 ms: each phase converts every other tick; D43, was 4 past the
// 38 A gate, which noise at a real ~33 A matched). The rail is ~+-40 A at
// GAIN_20 ('#range' at boot prints each board's). The DRV8353 VDS OCP
// (0.30 V, D42) stays the fast hardware backstop.
inline constexpr int kTripSamples = 20;
inline constexpr bool kTripLeaky = true;
inline constexpr int kTripFastConversions = 10;
#else
// NineBot S on the dyno: 15 pp, Kt 0.60 N.m/A measured (report fig 5), 520 rpm
// speed clamp from the 1 kW campaign, 30 A hard target ceiling.
inline constexpr const char *kName = "ninebot";
inline constexpr int kPolePairs = 15;
inline constexpr float kKt = 0.60f;
inline constexpr float kMaxRpm = 520.0f;
inline constexpr float kMaxAmps = 30.0f;
inline constexpr float kHoldAmps = 8.0f; // ~4.8 N.m; a bench number, no slope to hold
inline constexpr float kBootTargetAmps = 8.0f; // 'lim' at boot; raise live for campaigns
inline constexpr float kBootTripAmps = 15.0f;
inline constexpr float kContAmps = kMaxAmps;
inline constexpr float kPeakSeconds = 0.0f;
inline constexpr int kBootVdsLevel = 0;
inline constexpr float kSpeedKpEnc = 0.05f;
inline constexpr float kSpeedKpHall = 0.25f;
inline constexpr float kSpeedKiHall = 0.5f;
inline constexpr float kHallPllHz = 10.0f;
inline constexpr float kObsR = 0.323f;
inline constexpr float kObsL = 336.0e-6f;
inline constexpr float kObsLambda = 0.026f; // measured
inline constexpr float kBusVoltage = 48.0f;
inline constexpr float kDefaultKp = 0.0568f; // V/A, tuned on the dyno
inline constexpr float kDefaultKi = 54.5f;   // V/(A*s)
inline constexpr float kDefaultVoltLimit = 8.0f;
inline constexpr float kMaxAccelRpmS = 0.0f; // no accel ceiling: 0 = step (D24)
inline constexpr float kIqSlewAPerMs = 0.0f; // no iq slew limit (D40)
inline constexpr int kTripSamples = 3; // soft trip: 3 consecutive samples, no fast path (D41)
inline constexpr bool kTripLeaky = false;
inline constexpr int kTripFastConversions = 0;
#endif

/// Hall speed-loop gains (A/rpm, A/rpm-s) for a wheel: hub20 carries each chair wheel's own
/// tune, every other profile and an unassigned axis get the profile's kSpeedKpHall /
/// kSpeedKiHall. Axis numbers are rammp::AxisId (2 = drive_left, 3 = drive_right).
struct HallGains {
  float kp, ki;
};
inline constexpr HallGains hall_gains_for_axis(int axis_id) {
#if defined(ATOS_MOTOR_HUB20)
  if (axis_id == 2)
    return {kHallSpeedKpLeft, kHallSpeedKiLeft};
  if (axis_id == 3)
    return {kHallSpeedKpRight, kHallSpeedKiRight};
#endif
  (void)axis_id;
  return {kSpeedKpHall, kSpeedKiHall};
}

// 'isl' hard ceiling, A/ms. Above ~5 A/ms (169 rad/s x 30 A) the slew no longer
// shapes anything the current loop would not do on its own.
inline constexpr float kMaxIqSlewAPerMs = 10.0f;

// 'vl' hard ceiling: the most phase voltage SVPWM can synthesize from the bus,
// with margin. Derived so it follows kBusVoltage instead of being a stale
// literal from whatever supply was on the bench that day.
inline constexpr float kMaxPhaseVolts = 0.94f * kBusVoltage / 1.7320508f;

// SAFE_STOP ramp when the command's accel_limit is 0. The one place 0 does not
// mean "step": a step to zero rpm under load is a jolt on the stop path.
inline constexpr float kSafeStopRampRpmS = 200.0f;
// Below this the safe stop is over and HOLDING takes the position.
inline constexpr float kStoppedRpm = 2.0f;
// Position hold speed clamp while HOLDING (rpm). Small: it only corrects creep.
inline constexpr float kHoldMaxRpm = 30.0f;

// MotorState publish rate (D13, raised 20 -> 50 Hz 2026-10-29 for tuning
// logs on the MIB dashboard; RTPS saturated ~70 Hz delivered in Aug 2026).
inline constexpr uint32_t kStateHz = 50;

// Command watchdog, ms. The contract: the MIB sends >= 10 Hz, idle or not.
inline constexpr uint32_t kWatchdogMs = 200;

// MotorTuning bounds (D37). The wire carries SI (A per rad/s, A per rad, Hz);
// the loop runs in A/rpm. Anything a MotorTuning asks for is clamped into
// these, so a typo on the MIB cannot make the speed loop unstable or the
// tracking loop useless. The upper bounds are 8x the rover's hall defaults
// (0.25 A/rpm, 0.5 A/rpm-s); the PLL range is HallDrive's own. Both gains go
// to 0: switching a term off is a tune, not a typo (a negative on the wire).
inline constexpr float kTuneKpMinRpm = 0.0f, kTuneKpMaxRpm = 2.0f; // A/rpm
inline constexpr float kTuneKiMinRpm = 0.0f, kTuneKiMaxRpm = 4.0f;   // A/rpm-s
inline constexpr float kTunePllMinHz = 0.5f, kTunePllMaxHz = 50.0f;

// Firmware version reported in MotorState (major<<24 | minor<<16 | patch).
inline constexpr uint32_t kFwVersion = (0u << 24) | (4u << 16) | 5u;

// Conversions.
inline constexpr float kPi = 3.14159265358979f;
inline constexpr float kRpmToRadS = 2.0f * kPi / 60.0f;
inline constexpr float kRadSToRpm = 60.0f / (2.0f * kPi);
inline constexpr float kRadToDeg = 180.0f / kPi;
inline constexpr float kDegToRad = kPi / 180.0f;

} // namespace motor
