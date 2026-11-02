#pragma once

// Speed-loop rest hold on halls (797a07f, revised D43): when the speed target
// is 0 and the wheel has stopped, anchor the hall step count; within one step
// of it the integrator bleeds, past it the setpoint pulls back to the anchor.
//
// D43: the chair hunted (+-0.5 rad/s, +-5 A, 0.65 s period) after a stop from
// a spin. The anchor was taken at the speed loop's overshoot through zero, the
// wheel rolled on 2 steps and that counted as a slip, and the pull-back was an
// on/off kick (10 rpm at 2 steps, 0 at 1) reacting to a speed that is a 10 Hz
// tracking loop on 3 deg steps, ~0.2 s late. So: anchor only once no hall step
// has come for a while, and pull back in proportion to the steps beyond the
// deadband.

#include <algorithm>
#include <cstdlib>

namespace foc {

/// The wheel has stopped for anchoring: slow, and no hall step for `settle_s`.
inline bool rest_settled(float rpm, float since_step_s, float stopped_rpm, float settle_s) {
  return (rpm < 0 ? -rpm : rpm) < stopped_rpm && since_step_s >= settle_s;
}

/// Pull-back setpoint (rpm) for `perr` hall steps from the anchor: 0 inside the
/// +-1 step deadband, then `rpm_per_step` for each step beyond it, clamped.
inline float rest_pullback_rpm(int perr, float rpm_per_step, float max_rpm) {
  const int beyond = std::abs(perr) - 1;
  if (beyond <= 0)
    return 0.0f;
  const float rpm = std::min(rpm_per_step * (float)beyond, max_rpm);
  return perr > 0 ? rpm : -rpm;
}

} // namespace foc
