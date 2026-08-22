#pragma once

#include <atomic>
#include <numbers>
#include <system_error>

#include "driver/gpio.h"
#include "esp_timer.h"

/// Hall sensor satisfying espp::SensorConcept.
/// Tracks cumulative mechanical angle via ISR-driven sector step accumulation.
/// Velocity estimated from time between the last two valid transitions.
class HallSensor {
public:
  struct Config {
    gpio_num_t pin_a;
    gpio_num_t pin_b;
    gpio_num_t pin_c;
    int pole_pairs;
  };

  explicit HallSensor(Config cfg)
      : cfg_(cfg)
      , step_mech_rad_(std::numbers::pi_v<float> / (3.0f * cfg.pole_pairs)) {}

  void init() {
    uint8_t state = read_state();
    cur_state_.store(state, std::memory_order_relaxed);
    int boot_sec = kHallToSector[state & 0x7];
    prev_sector_ = boot_sec;
    // Seed steps from the boot sector so pole_pairs*get_mechanical_radians() ==
    // get_radians() at all times, making the electrical angle absolute rather
    // than relative to boot position — temp_sweep.cpp skips align_sensor
    // calibration (zero_electric_offset = 1e-6f) on exactly this assumption.
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

    int64_t t1 = t_last_.load(std::memory_order_relaxed);
    int64_t t0 = t_prev_.load(std::memory_order_relaxed);
    int64_t now = esp_timer_get_time();

    if (t1 == 0 || t1 == t0 || (now - t1) > kVelocityTimeoutUs) {
      rpm_ = 0.0f;
    } else {
      float dt_s = (t1 - t0) * 1e-6f;
      float candidate = (step_mech_rad_ / dt_s) * (60.0f / (2.0f * std::numbers::pi_v<float>));
      if (candidate < kMaxRpm)
        rpm_ = candidate; // ignore sector-boundary noise spikes
    }
  }

  bool needs_zero_search() const { return false; }
  float get_mechanical_radians() const { return mech_rad_; }
  float get_rpm() const { return rpm_; }
  float get_radians() const {
    int sec = kHallToSector[cur_state_.load(std::memory_order_relaxed) & 0x7];
    return (sec >= 0) ? sec * (std::numbers::pi_v<float> / 3.0f) : 0.0f;
  }

private:
  // NOTE: staircase angle (60° elec resolution), interpolate if PID needs smoothing
  static constexpr int kHallToSector[8] = {-1, 5, 3, 4, 1, 0, 2, -1};
  static constexpr int64_t kVelocityTimeoutUs = 200'000; // 200ms → assume stopped
  // Plausibility gate for the velocity estimate. Must sit ABOVE the fastest
  // sweep target (temp_sweep.cpp commands up to 800 RPM) or the feedback
  // freezes at speed and the loop drives against a stale reading — at the old
  // 500 the 500/800 RPM steps rejected every sample.
  static constexpr float kMaxRpm = 1500.0f;

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

    // Fresh-start / resync: first reading after boot, OR stopped long enough
    // that prev_sector_ may be stale from a transition masked by noise. Accept
    // the current sector as the new reference without a delta check — this is
    // the recovery path for a prev_sector_ frozen by a rejected jump.
    if (prev < 0 || t_last == 0 || (t_now - t_last) > kVelocityTimeoutUs) {
      self->cur_state_.store(state, std::memory_order_relaxed);
      self->prev_sector_ = sec;
      self->t_last_.store(t_now, std::memory_order_relaxed); // keep the timeout from re-firing
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
      // Valid single-sector step — update commutation state, position, velocity.
      self->cur_state_.store(state, std::memory_order_relaxed);
      self->steps_.fetch_add(delta, std::memory_order_relaxed);
      self->t_prev_.store(t_last, std::memory_order_relaxed);
      self->t_last_.store(t_now, std::memory_order_relaxed);
      self->prev_sector_ = sec;
    }
    // Multi-sector jump: noise — a real rotor crosses one hall boundary at a
    // time. Touch NOTHING (position, commutation state, prev_sector_); if a
    // real transition was masked, the timeout path above resynchronizes.
  }

  Config cfg_;
  float step_mech_rad_;

  std::atomic<uint8_t> cur_state_{0};
  std::atomic<int> steps_{0};
  std::atomic<int64_t> t_last_{0};
  std::atomic<int64_t> t_prev_{0};
  int prev_sector_{-1}; // ISR-only, no atomic needed

  float mech_rad_{0.0f};
  float rpm_{0.0f};
};
