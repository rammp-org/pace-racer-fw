#pragma once

// Control-grade hall commutation for the halls-sensorless app.
//
// POLLED, not interrupt-driven: HallPoller samples the three GPIOs every FOC
// tick (50 us) and validates transitions by consecutive-sample agreement. An
// any-edge interrupt on a floating/oscillating hall line is an interrupt storm
// that wedges CPU0 (seen on hardware 2026-07-30: the app hung between zero-cal
// and the banner the moment the halls were first wired) — polling makes that
// failure mode physically impossible and caps a bad line's damage at "sector
// reads wrong", which the adjacency check then contains. This layer turns the
// polled state into a drivable electrical angle:
//   - a CALIBRATED per-sector center-angle table (measured against the I/f
//     drive angle by `hcal`, so hall wiring order, mounting offset, and
//     direction sense are all absorbed by measurement, not assumed);
//   - transition-interval speed estimation with a "can't have moved a sector
//     without a transition" decay and a stall cutoff;
//   - interpolation between transitions, clamped to the sector so a stale
//     speed estimate can never run the angle away.
//
// Runs in the FOC task (CPU1, float fine). Single writer; `calibrated` is
// atomic so the console/stream tasks can gate and report on it.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include "driver/gpio.h"

namespace sensorless {

/// Interrupt-free replacement for HallSensor in the control path. Samples the
/// pins each FOC tick; a transition is accepted only after the new state has
/// been stable for kDebounceTicks consecutive samples AND is an adjacent
/// sector. Provides the same signals HallDrive consumed from HallSensor
/// (validated steps, transition timing) plus glitch counters.
class HallPoller {
public:
  static constexpr int kDebounceTicks = 4;   // 200 us stable before accepting:
                                             // slow edges on the weak internal
                                             // pull-ups bounced through 100 us
  // A direction reversal within this window of the previous transition is
  // treated as EDGE BOUNCE at a sector boundary (N -> N+1 -> N) and undone: a
  // real reversal can't turn around that fast, but a noisy edge can. Bounce
  // pairs were producing fake +-1500 rpm interval estimates -> speed-loop
  // current pops (seen/heard on the dyno 2026-07-30).
  static constexpr float kBounceSec = 0.005f;
  // Same mapping as HallSensor: (A<<2 | B<<1 | C) -> sector index (60 deg el).
  static constexpr int kHallToSector[8] = {-1, 5, 3, 4, 1, 0, 2, -1};

  void init(gpio_num_t a, gpio_num_t b, gpio_num_t c) {
    a_ = a; b_ = b; c_ = c;
    gpio_config_t cfg{
        .pin_bit_mask = (1ULL << a) | (1ULL << b) | (1ULL << c),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE, // NEVER interrupts — see header comment
    };
    gpio_config(&cfg);
    const uint8_t st = read_state();
    stable_state_ = pend_state_ = st;
    sector_ = kHallToSector[st & 0x7];
    ready_ = true;
  }

  /// One 50 us sample (FOC task). Advances steps on validated transitions.
  void poll(float dt) {
    if (!ready_) return;
    t_since_ += dt;
    const uint8_t st = read_state();
    if (st == stable_state_) {
      pend_state_ = st;
      pend_n_ = 0;
      return;
    }
    if (st != pend_state_) { // new candidate: restart the stability count
      pend_state_ = st;
      pend_n_ = 1;
      return;
    }
    if (++pend_n_ < kDebounceTicks) return;
    // Stable new state. Legal sector? Adjacent?
    pend_n_ = 0;
    stable_state_ = st;
    const int sec = kHallToSector[st & 0x7];
    if (sec < 0) {
      illegal_++;
      return; // 000/111: keep last good sector
    }
    if (sector_ < 0) { // first legal state seen
      sector_ = sec;
      return;
    }
    int delta = sec - sector_;
    if (delta > 3) delta -= 6;
    if (delta < -3) delta += 6;
    if (delta == 1 || delta == -1) {
      if (delta == -last_dir_ && t_since_ < kBounceSec) {
        // Boundary bounce: undo, and poison the last interval so the stale
        // pre-bounce timing can't be reused for a speed estimate.
        bounces_++;
        sector_ = sec;
        steps_ += delta; // net zero with the transition being undone
        last_dir_ = delta;
        interval_ = 0.0f;
        t_since_ = 0.0f;
        return;
      }
      sector_ = sec;
      steps_ += delta;
      last_dir_ = delta;
      interval_ = t_since_;
      t_since_ = 0.0f;
    } else if (delta != 0) {
      // Skipped sector(s): noise burst or a transition missed while stalled.
      // Resync to reality but don't feed the interval estimator garbage.
      glitches_++;
      sector_ = sec;
      steps_ += (delta > 0) ? 1 : -1; // bound the damage to one step
      t_since_ = 0.0f;
      interval_ = 0.0f;
    }
  }

  int sector() const { return sector_; }
  int steps() const { return steps_; }
  uint8_t raw_state() const { return stable_state_; }
  float interval_s() const { return interval_; }  // last validated sector time
  float since_s() const { return t_since_; }      // time since last transition
  uint32_t glitches() const { return glitches_; }
  uint32_t illegals() const { return illegal_; }
  uint32_t bounces() const { return bounces_; }

private:
  uint8_t read_state() const {
    return (uint8_t)((gpio_get_level(a_) << 2) | (gpio_get_level(b_) << 1) | gpio_get_level(c_));
  }
  gpio_num_t a_{GPIO_NUM_NC}, b_{GPIO_NUM_NC}, c_{GPIO_NUM_NC};
  bool ready_{false};
  uint8_t stable_state_{0}, pend_state_{0};
  int pend_n_{0};
  int sector_{-1};
  int steps_{0};
  int last_dir_{0};
  float t_since_{0.0f};
  float interval_{0.0f};
  uint32_t glitches_{0}, illegal_{0}, bounces_{0};
};

class HallDrive {
public:
  static constexpr float kPi = 3.14159265f;
  static constexpr float kTwoPi = 6.28318530718f;
  static constexpr float kSectorRad = kPi / 3.0f; // 60 deg electrical per sector
  // Interpolation may lead the sector center by slightly more than half a
  // sector (covers boundary jitter) but never runs away on a stale speed.
  static constexpr float kLeadLimit = kSectorRad * 0.55f;
  static constexpr float kMaxOmegaE = 2400.0f; // el rad/s ~ 1500 rpm at 15 pp
  static constexpr float kStallSec = 0.3f;     // no transition this long -> speed 0
  static constexpr int kCalMinPerSector = 50;  // ticks per sector for a valid cal

  void init(float rpm_to_omega_e) { rpm_to_omega_e_ = rpm_to_omega_e; }

  /// Per-tick update (FOC task). Consumes the poller's validated transition
  /// state and advances the interpolated angle.
  void update(const HallPoller &h, float dt) {
    const int sec = h.sector();
    const int steps = h.steps();
    since_ += dt;
    if (sec < 0) return; // no legal state seen yet: hold
    if (!seeded_ || sec_ < 0) {
      seeded_ = true;
      last_steps_ = steps;
      sec_ = sec;
      theta_ = center_[sec];
      omega_ = 0.0f;
      since_ = 0.0f;
      return;
    }
    if (steps != last_steps_ && sec != sec_) {
      // Poller-validated transition. Direction comes from the CALIBRATED table
      // (which sector center is electrically ahead), so hall wiring order
      // can't flip the drive.
      const float dth = wrap_pi(center_[sec] - center_[sec_]);
      const float dir = dth >= 0.0f ? 1.0f : -1.0f;
      const float iv = h.interval_s();
      float w = omega_;
      if (dir * omega_ < 0.0f) w = 0.0f; // real reversals pass through zero speed;
                                         // a fast opposite-dir interval is noise
      else if (iv > 0.0002f && iv < 2.0f) w = dir * kSectorRad / iv;
      omega_ = std::clamp(w, -kMaxOmegaE, kMaxOmegaE);
      // Rotor is at the boundary it just crossed: half a sector behind center.
      theta_ = wrap_2pi(center_[sec] - dir * kSectorRad * 0.5f);
      last_steps_ = steps;
      sec_ = sec;
      since_ = 0.0f;
    } else {
      // Between transitions: the true angle can't have left the sector, so a
      // speed that claims it did is stale — shrink it to "one sector per
      // elapsed time", and to zero after the stall cutoff.
      if (since_ > kStallSec) omega_ = 0.0f;
      else if (since_ > 0.02f && std::fabs(omega_) * since_ > kSectorRad)
        omega_ = (omega_ >= 0 ? 1.0f : -1.0f) * kSectorRad / since_;
      theta_ = wrap_2pi(theta_ + omega_ * dt);
      const float lead = wrap_pi(theta_ - center_[sec_]);
      if (lead > kLeadLimit) theta_ = wrap_2pi(center_[sec_] + kLeadLimit);
      if (lead < -kLeadLimit) theta_ = wrap_2pi(center_[sec_] - kLeadLimit);
    }
    // Filtered mechanical rpm for the speed loop (~50 ms time constant).
    rpm_f_ += (dt / (0.05f + dt)) * (omega_ / rpm_to_omega_e_ - rpm_f_);
  }

  float theta() const { return theta_; }
  float omega_e() const { return omega_; }
  float rpm_filtered() const { return rpm_f_; }
  int sector_seen() const { return sec_; }
  bool calibrated() const { return cal_.load(std::memory_order_relaxed); }
  float center_deg(int sec) const { return center_[sec] * 180.0f / kPi; }

  /// Calibration: while I/f drives the rotor, the circular mean of the drive
  /// angle seen during each hall sector IS that sector's center — EXCEPT the
  /// rotor lags the drive by the load angle, which biased the whole table by
  /// tens of degrees on the dyno (drag ~= the cal current: "iq" became mostly
  /// id, 5 A locked the rotor as a detent). The lag flips sign with direction,
  /// so the cal demands EQUAL ticks in each direction and the mean cancels it.
  /// FOC-task context.
  void cal_arm(uint32_t ticks_per_dir) {
    cal_reset();
    cal_fwd_left_ = cal_rev_left_ = ticks_per_dir;
  }

  /// One accumulation tick; `fwd` is the I/f drive direction. Returns 0 while
  /// in progress, +1 finished-ok, -1 finished-failed.
  int cal_step(int sec, float th_drive, bool fwd) {
    uint32_t &left = fwd ? cal_fwd_left_ : cal_rev_left_;
    if (left > 0 && sec >= 0 && sec <= 5) {
      cal_c_[sec] += std::cos(th_drive);
      cal_s_[sec] += std::sin(th_drive);
      cal_n_[sec]++;
      left--;
    }
    if (cal_fwd_left_ == 0 && cal_rev_left_ == 0) return cal_finish() ? 1 : -1;
    return 0;
  }

  uint32_t cal_fwd_left() const { return cal_fwd_left_; }
  uint32_t cal_rev_left() const { return cal_rev_left_; }

  /// Returns false (keeping the PREVIOUS table) unless every sector was seen
  /// enough AND the resulting centers land ~60 deg apart. The spacing check
  /// exists because a smeared accumulation — rotor stalled or pole-slipping
  /// while the drive angle sweeps — can produce a plausible-looking but wrong
  /// table (it did: +3 A drove the rotor BACKWARD on the dyno, 2026-07-30).
  bool cal_finish() {
    for (int s = 0; s < 6; s++) last_n_[s] = cal_n_[s]; // keep for the report
    for (int s = 0; s < 6; s++)
      if (cal_n_[s] < kCalMinPerSector) {
        cal_reset();
        return false;
      }
    // The accumulated mean is the I/f DRIVE angle seen in each sector — but
    // under I/f the rotor d-axis aligns with the CURRENT VECTOR, which pure iq
    // places at drive+90 deg (stage-3 hardware data: aerr ~ +90 in light-load
    // I/f). The table must hold the ROTOR axis, so rotate +90. Without this
    // the table is rotor-90 exactly, and "iq" lands on the d-axis: zero
    // torque, perfect detent — the better the cal, the harder the lock
    // (observed: a validated balanced cal locked the rotor at the clamp).
    float tmp[6];
    for (int s = 0; s < 6; s++)
      tmp[s] = wrap_2pi(std::atan2(cal_s_[s], cal_c_[s]) + kPi * 0.5f);
    float sorted[6];
    std::copy(tmp, tmp + 6, sorted);
    std::sort(sorted, sorted + 6);
    for (int s = 0; s < 6; s++) {
      const float gap =
          (s == 5) ? (sorted[0] + kTwoPi - sorted[5]) : (sorted[s + 1] - sorted[s]);
      if (gap < 35.0f * kPi / 180.0f || gap > 85.0f * kPi / 180.0f) {
        cal_reset();
        return false;
      }
    }
    for (int s = 0; s < 6; s++) center_[s] = tmp[s];
    cal_reset();
    seeded_ = false; // re-seed the interpolator on the new table
    cal_.store(true, std::memory_order_relaxed);
    return true;
  }

  int cal_count(int sec) const { return last_n_[sec]; }
  void cal_reset() {
    for (int s = 0; s < 6; s++) {
      cal_c_[s] = cal_s_[s] = 0.0f;
      cal_n_[s] = 0;
    }
  }

  /// Global trim (console `hofs <deg>`): rotate the whole table.
  void add_offset_deg(float deg) {
    const float r = deg * kPi / 180.0f;
    for (int s = 0; s < 6; s++) center_[s] = wrap_2pi(center_[s] + r);
    seeded_ = false;
  }

private:
  static float wrap_pi(float a) {
    a -= kTwoPi * std::floor(a / kTwoPi + 0.5f);
    return a;
  }
  static float wrap_2pi(float a) {
    a -= kTwoPi * std::floor(a / kTwoPi);
    return a;
  }

  float rpm_to_omega_e_{1.0f};
  // Uncalibrated default: sector index * 60 deg. NOT trustworthy for drive —
  // `hcal` replaces it with measured values; hall modes are gated on cal_.
  float center_[6] = {0.0f,          kSectorRad,        2.0f * kSectorRad,
                      3.0f * kSectorRad, 4.0f * kSectorRad, 5.0f * kSectorRad};
  std::atomic<bool> cal_{false};

  bool seeded_{false};
  int last_steps_{0};
  int sec_{-1};
  float theta_{0.0f};
  float omega_{0.0f};
  float rpm_f_{0.0f};
  float since_{0.0f};

  float cal_c_[6] = {}, cal_s_[6] = {};
  int cal_n_[6] = {};
  int last_n_[6] = {}; // counts at the last cal_finish, for the report
  uint32_t cal_fwd_left_{0}, cal_rev_left_{0}; // per-direction tick budgets
};

} // namespace sensorless
