#pragma once

// Motor and drive constants the ATOS contract depends on. Everything a
// MotorCommand limit of 0 resolves to, and every unit conversion between the
// wire (rad, rad/s, N.m at the shaft) and the FOC atomics (rpm, amps, rotor
// degrees), reads from here.
//
// Two motors: the NineBot S on the dyno, which the whole FOC stack was validated
// on, and the wheelchair hub motor the chair ships with. Select with the
// ATOS_MOTOR_HUB20 compile definition (CMake option ATOS_MOTOR=hub20); default is
// the dyno motor so the API can be validated on the existing rig first.
//
// Hub motor numbers are from its datasheet (2026-09-16) and NOT yet measured on
// this board: R/L via 'rl', Kt via a torque-sensor sweep, and the current-loop
// gains 'g <kp> <ki>' all have to be redone before the chair drives on them.

#include <cstdint>

namespace motor {

#ifdef ATOS_MOTOR_HUB20
// Wheelchair hub motor: 48 V, 17 N.m / 7.5 A rated, 51 N.m / 22 A peak, 150 rpm
// rated / 180 rpm peak, R 0.7 ohm, L 1.885 mH (+/-10%), Ke 0.262 V/rpm,
// 4096 CPR quadrature encoder (A/B differential), halls.
inline constexpr const char *kName = "hub20";
inline constexpr int kPolePairs = 20;
inline constexpr float kKt = 2.3f; // N.m/A; 17/7.5 = 2.27, 51/22 = 2.32, Ke agrees within 10%
inline constexpr float kMaxRpm = 180.0f;  // vel_limit = 0. No-load figure at 48 V.
inline constexpr float kMaxAmps = 22.0f;  // torque_limit = 0. Datasheet peak.
inline constexpr float kHoldAmps = 10.0f; // HOLDING cap: ~23 N.m, holds a 10 deg slope
                                          // at 150 kg per drive wheel; 15 A sustained is
                                          // the measured warm thermal limit.
inline constexpr float kObsR = 0.7f;      // ohm, datasheet; confirm line-line vs phase
inline constexpr float kObsL = 1.885e-3f; // H, datasheet
inline constexpr float kObsLambda = 0.0f; // Wb, unknown; the observer is unused here
#else
// NineBot S on the dyno: 15 pp, Kt 0.60 N.m/A measured (report fig 5), 520 rpm
// speed clamp from the 1 kW campaign, 30 A hard target ceiling.
inline constexpr const char *kName = "ninebot";
inline constexpr int kPolePairs = 15;
inline constexpr float kKt = 0.60f;
inline constexpr float kMaxRpm = 520.0f;
inline constexpr float kMaxAmps = 30.0f;
inline constexpr float kHoldAmps = 8.0f; // ~4.8 N.m; a bench number, no slope to hold
inline constexpr float kObsR = 0.323f;
inline constexpr float kObsL = 336.0e-6f;
inline constexpr float kObsLambda = 0.026f;
#endif

// SAFE_STOP ramp when the command's accel_limit is 0. The one place 0 does not
// mean "step": a step to zero rpm under load is a jolt on the stop path.
inline constexpr float kSafeStopRampRpmS = 200.0f;
// Below this the safe stop is over and HOLDING takes the position.
inline constexpr float kStoppedRpm = 2.0f;
// Position hold speed clamp while HOLDING (rpm). Small: it only corrects creep.
inline constexpr float kHoldMaxRpm = 30.0f;

// Command watchdog, ms. The contract: the MIB sends >= 10 Hz, idle or not.
inline constexpr uint32_t kWatchdogMs = 200;

// Firmware version reported in MotorState (major<<24 | minor<<16 | patch).
inline constexpr uint32_t kFwVersion = (0u << 24) | (1u << 16) | 0u;

// Conversions.
inline constexpr float kPi = 3.14159265358979f;
inline constexpr float kRpmToRadS = 2.0f * kPi / 60.0f;
inline constexpr float kRadSToRpm = 60.0f / (2.0f * kPi);
inline constexpr float kRadToDeg = 180.0f / kPi;
inline constexpr float kDegToRad = kPi / 180.0f;

} // namespace motor
