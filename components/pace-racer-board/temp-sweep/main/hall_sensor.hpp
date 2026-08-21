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
    prev_sector_ = kHallToSector[state & 0x7];

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
    int sector = kHallToSector[state & 0x7];
    self->cur_state_.store(state, std::memory_order_relaxed);

    int prev = self->prev_sector_;
    if (sector >= 0 && prev >= 0) {
      int delta = sector - prev;
      if (delta > 3)
        delta -= 6;
      if (delta < -3)
        delta += 6;
      if (delta != 0) {
        self->steps_.fetch_add(delta, std::memory_order_relaxed);
        if (delta == 1 || delta == -1) {
          // Only update velocity timestamps on single-sector steps.
          // Multi-sector jumps (|delta| > 1) are noise — a real motor can only
          // cross one hall boundary at a time at any sane speed.
          self->t_prev_.store(self->t_last_.load(std::memory_order_relaxed),
                              std::memory_order_relaxed);
          self->t_last_.store(esp_timer_get_time(), std::memory_order_relaxed);
        }
      }
    }
    if (sector >= 0)
      self->prev_sector_ = sector;
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
