#pragma once

// Forward FOC coordinate transforms — the current-feedback half of FOC that
// espp does not provide. espp::BldcMotor only has the inverse/output path
// (uq,ud,angle -> phase voltages), buried in a private method, because it never
// measures current. These take measured phase currents the other way.
//
// Float, task-context only: never call from an ISR — the Xtensa FPU is
// unavailable there (it raises a coprocessor exception).
//
// Amplitude-invariant convention: the alpha/beta and d/q magnitudes equal the
// phase-current amplitude, so the numbers are directly comparable to a current
// clamp. For the output path in stage 2 we can lift espp's SVPWM sector code
// (also in BldcMotor) rather than rewrite it; the forward transforms below have
// no espp equivalent.
//
// Stage 1 uses clarke() only, to validate that real phase currents survive the
// ISR -> task handoff. park()/inverse_park() are here for stage 2 and unused
// until then.

#include <algorithm>

namespace foc {

struct AlphaBeta {
  float alpha;
  float beta;
};

struct Dq {
  float d;
  float q;
};

inline constexpr float kInvSqrt3 = 0.57735026919f; // 1/sqrt(3)

/// Clarke transform (3-phase -> stationary alpha/beta). Assumes a balanced
/// system (ia + ib + ic == 0); ic is not needed once that holds.
inline AlphaBeta clarke(float ia, float ib) {
  return {ia, (ia + 2.0f * ib) * kInvSqrt3};
}

/// Park transform (stationary alpha/beta -> rotor dq), given the electrical
/// angle theta. Caller supplies sin/cos so a spinning loop can share one
/// sincos per period (e.g. espp::fast_sin / fast_cos).
inline Dq park(AlphaBeta ab, float sin_theta, float cos_theta) {
  return {ab.alpha * cos_theta + ab.beta * sin_theta,
          -ab.alpha * sin_theta + ab.beta * cos_theta};
}

/// Inverse Park (rotor dq -> stationary alpha/beta).
inline AlphaBeta inverse_park(Dq dq, float sin_theta, float cos_theta) {
  return {dq.d * cos_theta - dq.q * sin_theta, dq.d * sin_theta + dq.q * cos_theta};
}

struct Duties {
  float a, b, c;
};

/// Space-vector PWM by min/max zero-sequence injection: inverse-Clarke the
/// alpha/beta voltages to three phase voltages, subtract the midpoint of their
/// min and max (the SVPWM common-mode term — same result as the sector method
/// espp uses, fewer branches), then map to [0,1] duties centered at 0.5. vbus is
/// the DC bus voltage. Duties are clamped so a comparator never sees 0 or 1.
inline Duties svpwm(AlphaBeta v, float vbus, float min_duty, float max_duty) {
  const float ua = v.alpha;
  const float ub = -0.5f * v.alpha + 0.86602540f * v.beta; // sqrt(3)/2
  const float uc = -0.5f * v.alpha - 0.86602540f * v.beta;
  const float vmax = std::max(ua, std::max(ub, uc));
  const float vmin = std::min(ua, std::min(ub, uc));
  const float voff = 0.5f * (vmax + vmin);
  const float inv = 1.0f / vbus;
  auto to_duty = [&](float u) {
    float d = 0.5f + (u - voff) * inv;
    return d < min_duty ? min_duty : (d > max_duty ? max_duty : d);
  };
  return {to_duty(ua), to_duty(ub), to_duty(uc)};
}

} // namespace foc
