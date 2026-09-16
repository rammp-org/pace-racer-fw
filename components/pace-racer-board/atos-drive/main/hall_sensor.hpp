#pragma once

#include <atomic>
#include <numbers>
#include <system_error>

#include "driver/gpio.h"
#include "esp_timer.h"

/// Hall sensor satisfying espp::SensorConcept.
/// Tracks cumulative mechanical angle via ISR-driven sector step accumulation.
/// Velocity estimated from time between the last two valid single-sector transitions.
class HallSensor {
public:
  struct Config {
    gpio_num_t pin_a;
    gpio_num_t pin_b;
    gpio_num_t pin_c;
    int pole_pairs;
  };

  struct GlitchInfo {
    uint32_t count;
    int last_delta;
    uint8_t last_state; // raw ABC bits that caused the glitch
  };

  explicit HallSensor(Config cfg)
      : cfg_(cfg)
      , step_mech_rad_(std::numbers::pi_v<float> / (3.0f * cfg.pole_pairs)) {}

  void init() {
    uint8_t state = read_state();
    cur_state_.store(state, std::memory_order_relaxed);
    int boot_sec = kHallToSector[state & 0x7];
    prev_sector_ = boot_sec;
    // Seed steps from the boot sector so pole_pairs*get_mechanical_radians() == get_radians()
    // at all times, making the electrical angle absolute rather than relative to boot position.
    if (boot_sec >= 0)
      steps_.store(boot_sec, std::memory_order_relaxed);

    gpio_config_t cfg{
        .pin_bit_mask = (1ULL << cfg_.pin_a) | (1ULL << cfg_.pin_b) | (1ULL << cfg_.pin_c),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&cfg);
    gpio_install_isr_service(0); // no-op if already installed
    gpio_isr_handler_add(cfg_.pin_a, isr, this);
    gpio_isr_handler_add(cfg_.pin_b, isr, this);
    gpio_isr_handler_add(cfg_.pin_c, isr, this);
  }

  void update(std::error_code &) {
    int steps = steps_.load(std::memory_order_relaxed);
    mech_rad_ = steps * step_mech_rad_;

    // RPM from sector count over the polling window (~100 ms from the stream),
    // NOT from the last transition interval: two noise edges microseconds
    // apart made the interval estimator report absurd speeds (1212, 575,
    // -14815 rpm on the bench). Counting steps bounds a noise edge's damage
    // to ±1 sector over the whole window.
    int64_t now = esp_timer_get_time();
    if (upd_t_ == 0) {
      upd_t_ = now;
      upd_steps_ = steps;
      return;
    }
    const float dt_s = (now - upd_t_) * 1e-6f;
    if (dt_s < 0.02f)
      return; // called faster than expected: keep the window
    rpm_ = (float)(steps - upd_steps_) * step_mech_rad_ / dt_s *
           (60.0f / (2.0f * std::numbers::pi_v<float>));
    upd_t_ = now;
    upd_steps_ = steps;
  }

  bool needs_zero_search() const { return false; }
  float get_mechanical_radians() const { return mech_rad_; }
  float get_rpm() const { return rpm_; }
  float get_radians() const {
    int sec = kHallToSector[cur_state_.load(std::memory_order_relaxed) & 0x7];
    return (sec >= 0) ? sec * (std::numbers::pi_v<float> / 3.0f) : 0.0f;
  }

  GlitchInfo glitch_info() const {
    return {
        .count = glitch_count_.load(std::memory_order_relaxed),
        .last_delta = glitch_delta_.load(std::memory_order_relaxed),
        .last_state = glitch_state_.load(std::memory_order_relaxed),
    };
  }

  // Control-path accessors (HallDrive, FOC task). steps_ only moves on
  // ISR-validated adjacent-sector transitions, so polling it is the clean
  // "a real transition happened" signal; t_last/t_prev give its timing.
  int steps() const { return steps_.load(std::memory_order_relaxed); }
  int sector() const { return kHallToSector[cur_state_.load(std::memory_order_relaxed) & 0x7]; }
  uint8_t raw_state() const { return cur_state_.load(std::memory_order_relaxed); }
  int64_t t_last_us() const { return t_last_.load(std::memory_order_relaxed); }
  int64_t t_prev_us() const { return t_prev_.load(std::memory_order_relaxed); }

private:
  static constexpr int kHallToSector[8] = {-1, 5, 3, 4, 1, 0, 2, -1};
  static constexpr int64_t kVelocityTimeoutUs = 200'000; // 200ms → assume stopped
  static constexpr float kMaxRpm = 1500.0f; // NOTE: was 500 — raise above any expected target

  uint8_t read_state() const {
    return static_cast<uint8_t>((gpio_get_level(cfg_.pin_a) << 2) |
                                (gpio_get_level(cfg_.pin_b) << 1) | gpio_get_level(cfg_.pin_c));
  }

  static void isr(void *arg) {
    auto *self = static_cast<HallSensor *>(arg);
    uint8_t state = static_cast<uint8_t>((gpio_get_level(self->cfg_.pin_a) << 2) |
                                         (gpio_get_level(self->cfg_.pin_b) << 1) |
                                         gpio_get_level(self->cfg_.pin_c));
    int sec = kHallToSector[state & 0x7];

    if (sec < 0)
      return; // 0b000 / 0b111 — ignore entirely

    int prev = self->prev_sector_;
    int64_t t_now = esp_timer_get_time();
    int64_t t_last = self->t_last_.load(std::memory_order_relaxed);

    // Fresh-start: first reading after boot, OR motor has been stopped long enough that
    // prev_sector_ may be stale from a missed intermediate sector during a noise burst.
    // In either case, accept the current sector as the new reference without delta check.
    // This is the recovery path for: noise masks sector N→N+1 transition → sector N+2
    // arrives with delta=2 → rejected → prev_sector_ frozen → commutation stuck → braking.
    if (prev < 0 || t_last == 0 || (t_now - t_last) > kVelocityTimeoutUs) {
      self->cur_state_.store(state, std::memory_order_relaxed);
      self->prev_sector_ = sec;
      self->t_last_.store(t_now,
                          std::memory_order_relaxed); // keep timeout from re-firing constantly
      return;
    }

    int delta = sec - prev;
    if (delta > 3)
      delta -= 6;
    if (delta < -3)
      delta += 6;

    if (delta == 0)
      return; // same sector, spurious edge

    if (delta == 1 || delta == -1) {
      // Valid single-sector step — update commutation state, position, and velocity.
      self->cur_state_.store(state, std::memory_order_relaxed);
      self->steps_.fetch_add(delta, std::memory_order_relaxed);
      self->t_prev_.store(t_last, std::memory_order_relaxed);
      self->t_last_.store(t_now, std::memory_order_relaxed);
      self->prev_sector_ = sec;
    } else {
      // Multi-sector jump = noise. Log it; do NOT touch cur_state_ or prev_sector_.
      self->glitch_count_.fetch_add(1, std::memory_order_relaxed);
      self->glitch_delta_.store(delta, std::memory_order_relaxed);
      self->glitch_state_.store(state, std::memory_order_relaxed);
    }
  }

  Config cfg_;
  float step_mech_rad_;

  std::atomic<uint8_t> cur_state_{0};
  std::atomic<int> steps_{0};
  std::atomic<int64_t> t_last_{0};
  std::atomic<int64_t> t_prev_{0};
  int prev_sector_{-1}; // ISR-only, no atomic needed

  std::atomic<uint32_t> glitch_count_{0};
  std::atomic<int> glitch_delta_{0};
  std::atomic<uint8_t> glitch_state_{0};

  float mech_rad_{0.0f};
  float rpm_{0.0f};
  int64_t upd_t_{0}; // rpm window start (task context only)
  int upd_steps_{0}; // step count at window start
};
