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
inline constexpr float kSpeedKpEnc = 0.05f;  // speed-loop Kp, A/rpm, encoder feedback
inline constexpr float kSpeedKpHall = 0.25f; // speed-loop Kp, A/rpm, hall feedback
inline constexpr float kMaxAccelRpmS = 0.0f; // no accel ceiling: accel_limit 0 = step
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
inline constexpr float kMaxAmps = 22.0f;  // torque_limit = 0. Datasheet peak.
inline constexpr float kHoldAmps = 10.0f; // HOLDING cap: ~16 N.m; 15 A sustained is
                                          // the measured warm thermal limit.
// 'lim' at boot. The ATOS hooks clamp every current to this, below kMaxAmps,
// so it is the chair's real torque ceiling: 15 A (23 N.m), the warm thermal
// limit. Trip 3 A above it for speed-loop overshoot; the DRV VDS OCP at boot
// (~17-22 A cold, derates hot) is the hardware backstop.
inline constexpr float kBootTargetAmps = 15.0f;
inline constexpr float kBootTripAmps = 18.0f;
// Speed-loop Kp (A/rpm), half the dyno/rover values: those made the chair
// jitter (2026-10-29). Ki is unchanged, it supplies the breakaway current.
inline constexpr float kSpeedKpEnc = 0.025f;
inline constexpr float kSpeedKpHall = 0.125f;
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
inline constexpr float kDefaultVoltLimit = 16.0f;
inline constexpr float kMaxAccelRpmS = 0.0f; // no accel ceiling: 0 = step (D24)
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
inline constexpr float kSpeedKpEnc = 0.05f;
inline constexpr float kSpeedKpHall = 0.25f;
inline constexpr float kObsR = 0.323f;
inline constexpr float kObsL = 336.0e-6f;
inline constexpr float kObsLambda = 0.026f; // measured
inline constexpr float kBusVoltage = 48.0f;
inline constexpr float kDefaultKp = 0.0568f; // V/A, tuned on the dyno
inline constexpr float kDefaultKi = 54.5f;   // V/(A*s)
inline constexpr float kDefaultVoltLimit = 8.0f;
inline constexpr float kMaxAccelRpmS = 0.0f; // no accel ceiling: 0 = step (D24)
#endif

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
inline constexpr uint32_t kFwVersion = (0u << 24) | (3u << 16) | 2u;

// Conversions.
inline constexpr float kPi = 3.14159265358979f;
inline constexpr float kRpmToRadS = 2.0f * kPi / 60.0f;
inline constexpr float kRadSToRpm = 60.0f / (2.0f * kPi);
inline constexpr float kRadToDeg = 180.0f / kPi;
inline constexpr float kDegToRad = kPi / 180.0f;

} // namespace motor
