#pragma once

// Flux-linkage observer + PLL — estimates the rotor electrical angle and speed
// from the electrical model, with no position sensor. Float, FOC-task context.
//
// Method (the ODrive-style nonlinear flux observer, chosen over sliding-mode for
// less chatter and a single magnitude to tune):
//   1. Integrate the stator flux from the applied voltage and measured current:
//        psi += (v - R*i) dt
//   2. Subtract the inductance term to get the rotor PM flux vector:
//        f = psi - L*i
//   3. A pure integrator drifts (DC offset, wrong initial condition), so a
//      nonlinear correction pulls |f| toward the known magnitude lambda_pm:
//        psi += gain * (lambda_pm^2 - |f|^2) * f dt
//      This kills drift without the phase lag a high-pass would add.
//   4. theta_est = atan2(f_beta, f_alpha); a PLL locks a smooth angle/speed onto
//      it (phase error via the cross product, so no atan2 inside the loop).
//
// Limitation by construction: the back-EMF term (v - R*i) vanishes at zero speed,
// so the angle is unobservable at standstill and poor at low speed — worse still
// because we feed COMMANDED voltage, whose deadtime error dominates when the
// back-EMF is small. Stage 3 measures where that wall is; it is not fixed here.

#include <cmath>

#include "fast_math.hpp" // espp::fast_sin / fast_cos

namespace sensorless {

class FluxObserver {
public:
  struct Params {
    float r;          // phase resistance [ohm]
    float l;          // phase inductance [H]
    float lambda_pm;  // PM flux linkage [Wb]
    float flux_gain;  // nonlinear magnitude-correction gain
    float pll_kp;     // PLL proportional gain [1/s]
    float pll_ki;     // PLL integral gain [1/s^2]
  };

  void set_params(const Params &p) { p_ = p; }

  // Clear the integrators. Seed the PLL to the raw estimate so it locks quickly.
  void reset() {
    psi_a_ = 0.0f;
    psi_b_ = 0.0f;
    theta_ = 0.0f;
    omega_ = 0.0f;
    seeded_ = false;
  }

  // One step. ia/ib = measured stationary-frame current; va/vb = the applied
  // stationary-frame voltage (use the PREVIOUS tick's command — it produced this
  // tick's current); dt = seconds since the last step.
  void update(float ia, float ib, float va, float vb, float dt) {
    // 1) stator flux integration
    psi_a_ += (va - p_.r * ia) * dt;
    psi_b_ += (vb - p_.r * ib) * dt;

    // 2) rotor PM flux
    float fa = psi_a_ - p_.l * ia;
    float fb = psi_b_ - p_.l * ib;

    // 3) nonlinear correction toward |f| = lambda_pm
    const float err = p_.lambda_pm * p_.lambda_pm - (fa * fa + fb * fb);
    psi_a_ += p_.flux_gain * err * fa * dt;
    psi_b_ += p_.flux_gain * err * fb * dt;
    // recompute after correcting the integrators
    fa = psi_a_ - p_.l * ia;
    fb = psi_b_ - p_.l * ib;
    flux_mag_ = std::sqrt(fa * fa + fb * fb);
    theta_est_ = atan2f(fb, fa); // raw estimate, for logging

    // 4) PLL. phase error = sin(theta_est - theta_pll), from the cross product of
    // the flux vector with the PLL unit vector, normalized by |lambda_pm|.
    if (!seeded_) { // lock instantly on the first valid step
      theta_ = theta_est_;
      seeded_ = true;
    }
    float tn = theta_;
    tn -= kTwoPi * floorf(tn / kTwoPi);
    const float ct = espp::fast_cos(tn);
    const float st = espp::fast_sin(tn);
    const float inv_lam = p_.lambda_pm > 1e-6f ? 1.0f / p_.lambda_pm : 0.0f;
    const float phase_err = (fb * ct - fa * st) * inv_lam; // ~sin(theta_est - theta_pll)
    omega_ += p_.pll_ki * phase_err * dt;
    theta_ += (omega_ + p_.pll_kp * phase_err) * dt;
    theta_ -= kTwoPi * floorf(theta_ / kTwoPi);
  }

  float theta() const { return theta_; }         // PLL-tracked electrical angle [rad]
  float theta_raw() const { return theta_est_; } // atan2 estimate [rad]
  float omega() const { return omega_; }          // electrical speed [rad/s]
  float flux_mag() const { return flux_mag_; }    // |rotor flux| [Wb] — health check

private:
  static constexpr float kTwoPi = 6.28318530718f;

  Params p_{};
  float psi_a_{0}, psi_b_{0};
  float theta_{0}, omega_{0};
  float theta_est_{0}, flux_mag_{0};
  bool seeded_{false};
};

} // namespace sensorless
