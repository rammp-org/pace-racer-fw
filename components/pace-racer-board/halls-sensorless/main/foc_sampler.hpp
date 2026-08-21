#pragma once

// PWM-synchronized phase-current sampler for the PACE RACER board.
//
// Proven in stage 0 (see sensorless.cpp header for the measurements). The short
// version:
//   - Low-side shunts are only valid at the null vector, which on this board is
//     TEP (MCPWM "on_full"), not TEZ. espp::BldcDriver only exposes an on_empty
//     callback and its docstring names the wrong edge, so we register on_full
//     ourselves via a member-pointer "borrow" of its private timer handle.
//   - adc_oneshot_read_isr() re-runs the REGI2C ADC calibration on every call
//     (~14 us/read, trips the interrupt watchdog). We drive adc_oneshot_hal_*
//     directly and hoist the invariant setup + calibration to init, leaving
//     ~7 us per conversion.
//   - No FPU in an ISR on Xtensa (float raises a coprocessor exception), so the
//     ISR is integer-only: trip threshold in raw counts, brake by writing
//     comparator compare-value 0 directly (never espp set_pwm(), which is float).
//
// The ISR does acquisition only — read the two selected phases, run the integer
// safety checks, then vTaskNotifyGiveFromISR() a control task that does all the
// float FOC math. Sampling instant is precise; the math tolerates task jitter
// because the comparators latch on TEZ regardless of when they were written.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <memory>
#include <system_error>
#include <thread>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_clk_tree.h"
#include "esp_cpu.h"
#include "esp_private/adc_share_hw_ctrl.h"
#include "hal/adc_hal_common.h"
#include "hal/adc_oneshot_hal.h"

#include "adc_fast.h"
#include "logger.hpp"
#include "pace-racer-board.hpp"

using namespace std::chrono_literals;

namespace sensorless {

////////////////////////////////////////////////////////////////////////////////
// Borrowed handles from espp::BldcDriver
//
// Two things the driver keeps private: the MCPWM timer (to register our own
// on_full sample callback) and the comparators (so the ISR trip can brake the
// bridge in integers, since espp set_pwm() is float and there is no FPU in an
// ISR). Access checks do not apply to names in an explicit instantiation
// ([temp.explicit]), so these friend-injection borrows are legal. TODO: upstream
// an on_full sample callback + handle accessors to espp and delete this.
////////////////////////////////////////////////////////////////////////////////

using PwmComparators = std::array<mcpwm_cmpr_handle_t, 3>;

mcpwm_timer_handle_t borrow_pwm_timer(const espp::BldcDriver &driver);
const PwmComparators &borrow_pwm_comparators(const espp::BldcDriver &driver);

template <auto Member> struct PwmTimerBorrower {
  friend mcpwm_timer_handle_t borrow_pwm_timer(const espp::BldcDriver &driver) {
    return driver.*Member;
  }
};
template struct PwmTimerBorrower<&espp::BldcDriver::timer_>;

template <auto Member> struct PwmComparatorBorrower {
  friend const PwmComparators &borrow_pwm_comparators(const espp::BldcDriver &driver) {
    return driver.*Member;
  }
};
template struct PwmComparatorBorrower<&espp::BldcDriver::comparators_>;

////////////////////////////////////////////////////////////////////////////////

class FocSampler {
public:
  struct Config {
    espp::PaceRacerBoard *bsp;
    std::shared_ptr<espp::BldcDriver> driver;
    std::array<adc_channel_t, 3> phase_channels;
    float mv_to_a;                     ///< amps per millivolt at the configured CSA gain
    float trip_amps;                   ///< software overcurrent trip (per phase)
    gpio_num_t scope_pin{GPIO_NUM_NC}; ///< toggled around the in-ISR conversions
  };

  bool init(const Config &cfg, espp::Logger &logger) {
    cfg_ = cfg;
    s_self = this;

    if (cfg_.scope_pin != GPIO_NUM_NC) {
      gpio_config_t sc = {};
      sc.pin_bit_mask = 1ULL << cfg_.scope_pin;
      sc.mode = GPIO_MODE_OUTPUT;
      gpio_config(&sc);
      gpio_set_level(cfg_.scope_pin, 0);
    }

    cmp_ = borrow_pwm_comparators(*cfg_.driver);

    // Own ADC calibration handle: adc_oneshot_hal_convert() returns raw counts,
    // so we convert counts->mV->A ourselves. A cali scheme is independent of the
    // BSP's oneshot unit claim.
    adc_cali_curve_fitting_config_t cali_cfg = {};
    cali_cfg.unit_id = ADC_UNIT_1;
    cali_cfg.atten = ADC_ATTEN_DB_6;
    cali_cfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_) != ESP_OK) {
      logger.error("ADC calibration unavailable (eFuse not burnt?) — not running");
      return false;
    }
    // Local slope only: currents are differences from a zero, so the curve's
    // offset cancels and only mV-per-count matters.
    int mv_lo = 0, mv_hi = 0;
    adc_cali_raw_to_voltage(cali_, 1000, &mv_lo);
    adc_cali_raw_to_voltage(cali_, 3000, &mv_hi);
    const float mv_per_count = (float)(mv_hi - mv_lo) / 2000.0f;
    amps_per_count_ = mv_per_count * cfg_.mv_to_a;
    trip_counts_ = (int)(cfg_.trip_amps / amps_per_count_);
    max_jump_counts_ = (int)(kMaxJumpAmps / amps_per_count_);
    max_plausible_counts_ = (int)((cfg_.trip_amps + kPlausibleMarginAmps) / amps_per_count_);
    dbl_dis_counts_ = (int)(kDblDisagreeAmps / amps_per_count_);
    logger.info("ADC slope {:.4f} mV/count -> {:.5f} A/count; {:.0f} A trip = {} counts",
                mv_per_count, amps_per_count_, cfg_.trip_amps, trip_counts_);

    // Our own HAL view of ADC1. The BSP's OneshotAdc keeps the unit claim, pad
    // config and SAR power reference; this only adds a second way to program the
    // same registers, for the ISR.
    uint32_t clk_hz = 0;
    esp_clk_tree_src_get_freq_hz((soc_module_clk_t)ADC_RTC_CLK_SRC_DEFAULT,
                                 ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED, &clk_hz);
    adc_oneshot_hal_cfg_t hal_cfg = {
        .unit = ADC_UNIT_1,
        .work_mode = ADC_HAL_SINGLE_READ_MODE,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .clk_src_freq_hz = clk_hz,
        .disable_dac_output = false,
    };
    adc_oneshot_hal_init(&hal_, &hal_cfg);
    adc_oneshot_hal_chan_cfg_t chan_cfg = {.atten = ADC_ATTEN_DB_6,
                                           .bitwidth = ADC_BITWIDTH_DEFAULT};
    for (auto ch : cfg_.phase_channels) {
      adc_oneshot_hal_channel_config(&hal_, &chan_cfg, ch);
    }
    // The expensive REGI2C calibration, done once. Valid to hoist only because
    // every phase channel shares one attenuation.
    adc_hal_calibration_init(ADC_UNIT_1);
    adc_set_hw_calibration_code(ADC_UNIT_1, ADC_ATTEN_DB_6);
    prime();
    return true;
  }

  /// Re-program the invariant per-channel HAL setup. Must run after any
  /// task-context read through the BSP's OneshotAdc (it reprograms the same
  /// registers for its own channel) and once at init.
  void prime() {
    for (auto ch : cfg_.phase_channels) {
      adc_oneshot_hal_setup(&hal_, ch);
    }
    // The BSP path also moved the mux; restore the pre-selected channel so the
    // next ISR conversion is settled, not post-hop.
    adc_fast_select_channel(ADC_UNIT_1, cfg_.phase_channels[pending_ph_]);
  }

  void set_notify_task(TaskHandle_t t) { notify_task_ = t; }

  /// Register the sampling ISR on the requested edge. TEP (on_full) is the null
  /// vector on this board; TEZ is offered only for the stage-0 edge comparison.
  /// The MCPWM API requires the timer in its init state to attach callbacks, so
  /// the driver is stopped around it (outputs forced off meanwhile).
  bool set_edge(bool at_tep, espp::Logger &logger) {
    auto timer = borrow_pwm_timer(*cfg_.driver);
    cfg_.driver->disable();
    mcpwm_timer_event_callbacks_t cbs = {};
    if (at_tep)
      cbs.on_full = &FocSampler::isr_trampoline;
    else
      cbs.on_empty = &FocSampler::isr_trampoline;
    auto err = mcpwm_timer_register_event_callbacks(timer, &cbs, nullptr);
    cfg_.driver->enable();
    at_tep_ = at_tep;
    if (err != ESP_OK) {
      logger.error("Failed to register PWM sample callback: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  void enable() {
    have_last_entry_ = false; // the gap across a disable is meaningless
    late_run_ = 0;
    clear_filter_counters(); // trip reports read "since arm"
    enabled_.store(true, std::memory_order_release);
  }
  void disable() { enabled_.store(false, std::memory_order_release); }
  bool is_enabled() const { return enabled_.load(std::memory_order_acquire); }
  bool at_tep() const { return at_tep_; }

  /// Select which two phases the ISR samples; the third is reconstructed from
  /// ia+ib+ic=0. Pick the two *lowest*-duty phases: their low sides conduct
  /// longest, so their sample windows are widest, and the reconstructed one is
  /// the narrow-window highest-duty phase we would rather not chase.
  void select_lowest_two(float da, float db, float dc) {
    int hi = 0; // index of the largest duty -> the one to reconstruct
    if (db > da)
      hi = 1;
    if (dc > (hi == 0 ? da : db))
      hi = 2;
    int x = (hi == 0) ? 1 : 0;
    int y = (hi == 2) ? 1 : 2;
    if (x > y)
      std::swap(x, y);
    phase_x_.store(x, std::memory_order_relaxed);
    phase_y_.store(y, std::memory_order_relaxed);
  }
  void set_phases(int x, int y) {
    phase_x_.store(x, std::memory_order_relaxed);
    phase_y_.store(y, std::memory_order_relaxed);
  }
  int phase_x() const { return phase_x_.load(std::memory_order_relaxed); }
  int phase_y() const { return phase_y_.load(std::memory_order_relaxed); }
  int phase_z() const { return 3 - phase_x() - phase_y(); }

  void set_double_sample(bool on) { dbl_.store(on, std::memory_order_relaxed); }
  bool double_sample() const { return dbl_.load(std::memory_order_relaxed); }
  struct DblStats {
    uint32_t pairs, dis;                // pairs taken, pairs disagreeing >3 A
    uint32_t spike_agree;               // both twins >12 A and agreeing: the pin really moved
    uint32_t spike_first, spike_second; // refuted spike by slot (mechanism clue)
    float max_delta_amps;
  };
  DblStats dbl_stats() const {
    return {dbl_n_.load(),
            dbl_dis_.load(),
            dbl_spike_agree_.load(),
            dbl_spike_first_.load(),
            dbl_spike_second_.load(),
            (float)dbl_max_delta_.load() * amps_per_count_};
  }

  void set_decimation(uint32_t n) { decimation_.store(n, std::memory_order_relaxed); }
  uint32_t decimation() const { return decimation_.load(std::memory_order_relaxed); }
  void set_samples_per_isr(int n) { samples_per_isr_.store(n, std::memory_order_relaxed); }
  int samples_per_isr() const { return samples_per_isr_.load(std::memory_order_relaxed); }

  int raw_x() const { return raw_x_.load(std::memory_order_relaxed); }
  int raw_y() const { return raw_y_.load(std::memory_order_relaxed); }
  int raw_zero(int p) const { return raw_zero_[p]; }
  float amps_per_count() const { return amps_per_count_; }

  /// Current of a sampled phase in amps (task context). `which` is 0 for the
  /// x-phase, 1 for the y-phase.
  float amps_x() const { return (float)(raw_x() - raw_zero_[phase_x()]) * amps_per_count_; }
  float amps_y() const { return (float)(raw_y() - raw_zero_[phase_y()]) * amps_per_count_; }

  bool take_tripped() { return tripped_.exchange(false); }     // stream: report it
  bool take_soft_trip() { return soft_trip_.exchange(false); } // FOC task: unload
  bool take_overran() { return overran_.exchange(false); }

  /// Retune the software trip at runtime ('lim' during the high-power
  /// staircase). The plausibility gate follows at trip + kPlausibleMarginAmps
  /// so real current rising toward the new trip stays visible to it. Plain int
  /// stores: single-word writes are atomic on Xtensa and the ISR tolerates one
  /// sample judged against the old threshold.
  void set_trip_amps(float amps) {
    cfg_.trip_amps = amps;
    trip_counts_ = (int)(amps / amps_per_count_);
    max_plausible_counts_ = (int)((amps + kPlausibleMarginAmps) / amps_per_count_);
  }
  float trip_amps() const { return cfg_.trip_amps; }

  // Diagnostic capture: dump the ring of the last kCapN samples (pre-median raw
  // -> amps, and ISR elapsed us), oldest first, then re-arm. Freezes it first so
  // the snapshot is coherent. Auto-frozen on a trip; call this to inspect it.
  void dump_capture() {
    cap_frozen_.store(true, std::memory_order_relaxed);
    const uint32_t head = cap_head_.load(std::memory_order_relaxed);
    const uint32_t n = head < (uint32_t)kCapN ? head : (uint32_t)kCapN;
    fmt::print("#cap n={} (oldest first)\ni,phase,amps,isr_us\n", n);
    for (uint32_t k = 0; k < n; k++) {
      const uint32_t idx = (head - n + k) % kCapN;
      const auto &c = cap_[idx];
      const float amps = (float)((int)c.raw - raw_zero_[c.phase]) * amps_per_count_;
      fmt::print("{},{},{:.2f},{:.2f}\n", k, "ABC"[c.phase], amps, (float)c.cyc / 240.0f);
    }
    cap_head_.store(0, std::memory_order_relaxed);
    cap_frozen_.store(false, std::memory_order_relaxed);
  }

  // Compact analysis of the frozen capture, printed on a trip instead of the
  // 256-line raw dump (which stays available via 'c'/dump_capture). Two views,
  // chosen to separate a real overcurrent from conversion glitches at a glance:
  //  - per-phase MEDIAN over 8 time windows: a real trip ramps into the limit,
  //    a glitch trip has a flat baseline (the median ignores the spikes);
  //  - only the samples deviating > kAnomAmps from their window's baseline,
  //    with ISR timing, capped at kAnomMax lines.
  // Re-arms the ring like dump_capture.
  void dump_summary() {
    cap_frozen_.store(true, std::memory_order_relaxed);
    const uint32_t head = cap_head_.load(std::memory_order_relaxed);
    const uint32_t n = head < (uint32_t)kCapN ? head : (uint32_t)kCapN;
    if (n == 0) {
      fmt::print("#cap empty\n");
    } else {
      constexpr int kWin = 8;
      constexpr float kAnomAmps = 5.0f;
      constexpr int kAnomMax = 16;
      const uint32_t wlen = (n + kWin - 1) / kWin; // ring samples per window
      const float ms_per = 50e-3f * (float)decimation_.load(std::memory_order_relaxed);
      float med[kWin][3];
      auto amps_at = [&](uint32_t k) { // k-th oldest sample -> (phase, amps)
        const auto &c = cap_[(head - n + k) % kCapN];
        return std::pair<int, float>(c.phase,
                                     (float)((int)c.raw - raw_zero_[c.phase]) * amps_per_count_);
      };
      fmt::print("#trend n={} ({} samples/win, {:.2f} ms/sample) med amps per phase:\n"
                 "#  ms_before_trip",
                 n, wlen, ms_per);
      // Column header: only phases actually present (the measured pair).
      bool present[3] = {false, false, false};
      for (uint32_t k = 0; k < n; k++)
        present[amps_at(k).first] = true;
      for (int p = 0; p < 3; p++)
        if (present[p])
          fmt::print("  {}_med", "ABC"[p]);
      fmt::print("\n");
      for (int w = 0; w < kWin; w++) {
        float buf[3][kCapN / (2 * kWin) + 2]; // per-phase slice of one window
        int cnt[3] = {0, 0, 0};
        const uint32_t k0 = (uint32_t)w * wlen, k1 = std::min(n, k0 + wlen);
        for (uint32_t k = k0; k < k1; k++) {
          auto [p, a] = amps_at(k);
          if (cnt[p] < (int)(sizeof(buf[0]) / sizeof(float)))
            buf[p][cnt[p]++] = a;
        }
        for (int p = 0; p < 3; p++) {
          if (cnt[p]) {
            std::sort(buf[p], buf[p] + cnt[p]);
            med[w][p] = buf[p][cnt[p] / 2];
          } else {
            med[w][p] = med[w > 0 ? w - 1 : 0][p];
          }
        }
        fmt::print("#  {:6.1f}", -(float)(n - k0) * ms_per);
        for (int p = 0; p < 3; p++)
          if (present[p])
            fmt::print("  {:+6.2f}", med[w][p]);
        fmt::print("\n");
      }
      int anom = 0;
      for (uint32_t k = 0; k < n; k++) {
        auto [p, a] = amps_at(k);
        const float d = a - med[std::min<uint32_t>(k / wlen, kWin - 1)][p];
        if (d > kAnomAmps || d < -kAnomAmps) {
          if (anom == 0)
            fmt::print("#anom |amps-med|>{:.0f}A: i,ph,amps,isr_us\n", kAnomAmps);
          if (anom < kAnomMax) {
            const auto &c = cap_[(head - n + k) % kCapN];
            fmt::print("#  {},{},{:+.1f},{:.2f}\n", k, "ABC"[p], a, (float)c.cyc / 240.0f);
          }
          anom++;
        }
      }
      if (anom > kAnomMax)
        fmt::print("#  (+{} more)\n", anom - kAnomMax);
      fmt::print("#anom total={} ('c' before next arm for the full ring)\n", anom);
    }
    cap_head_.store(0, std::memory_order_relaxed);
    cap_frozen_.store(false, std::memory_order_relaxed);
  }

  // The state at the last trip, so a false trip (glitch reaching the filter
  // output) can be told apart from a real overcurrent (filtered ~= raw, and a
  // capture baseline that ramps into the limit).
  struct TripInfo {
    float amps[3];  // FILTERED amps indexed A/B/C; the reconstructed phase too
    int trip_phase; // which phase exceeded the limit (may be the reconstructed)
    int recon;      // which phase was reconstructed from ia+ib+ic=0
    float raw_amps; // the trip tick's PRE-filter conversion...
    int raw_phase;  // ...and which phase it sampled
  };
  TripInfo trip_info() const {
    TripInfo ti{};
    const int px = trip_px_.load(), py = trip_py_.load();
    ti.recon = 3 - px - py;
    ti.amps[px] = (float)trip_dx_.load() * amps_per_count_;
    ti.amps[py] = (float)trip_dy_.load() * amps_per_count_;
    ti.amps[ti.recon] = -(ti.amps[px] + ti.amps[py]);
    ti.trip_phase = trip_ph_.load();
    ti.raw_amps = (float)trip_raw_.load() * amps_per_count_;
    ti.raw_phase = trip_raw_ph_.load();
    return ti;
  }

  // Stats (task context).
  struct Stats {
    uint32_t count;
    float min_us, avg_us, max_us;
    uint32_t late;
    uint32_t imp[3], rej[3], esc[3]; // per-phase filter counters, see members
    float max_amps[3];               // largest |raw| conversion seen per phase
  };
  Stats stats() const {
    Stats st{};
    st.count = n_.load();
    const float s = st.count ? 1.0f / (float)st.count / 240.0f : 0.0f;
    st.min_us = (float)cyc_min_.load() / 240.0f;
    st.avg_us = (float)cyc_sum_.load() * s;
    st.max_us = (float)cyc_max_.load() / 240.0f;
    st.late = late_count_.load();
    for (int p = 0; p < 3; p++) {
      st.imp[p] = imp_count_[p].load();
      st.rej[p] = rej_count_[p].load();
      st.esc[p] = esc_count_[p].load();
      st.max_amps[p] = (float)max_dev_[p].load() * amps_per_count_;
    }
    return st;
  }
  void reset_stats() {
    n_.store(0);
    cyc_sum_.store(0);
    cyc_min_.store(0xffffffff);
    cyc_max_.store(0);
    late_count_.store(0);
    clear_filter_counters();
    dbl_n_.store(0);
    dbl_dis_.store(0);
    dbl_spike_agree_.store(0);
    dbl_spike_first_.store(0);
    dbl_spike_second_.store(0);
    dbl_max_delta_.store(0);
  }

  /// Median-of-3 raw read via the BSP's slow OneshotAdc path (task context).
  /// Suspend the ISR first (they don't lock against each other on the SAR unit).
  float read_raw_async(int phase) {
    espp::AdcConfig ch = {
        .unit = ADC_UNIT_1, .channel = cfg_.phase_channels[phase], .attenuation = ADC_ATTEN_DB_6};
    auto &adc = cfg_.bsp->adc1();
    float a = (float)adc.read_raw(ch).value_or(0);
    float b = (float)adc.read_raw(ch).value_or(0);
    float c = (float)adc.read_raw(ch).value_or(0);
    return std::max(std::min(a, b), std::min(std::max(a, b), c));
  }

  /// Zero-current offset calibration. Measures the zero in NORMAL mode with the
  /// bridge braked (espp disable() stops at the peak, all low sides on) — that
  /// reading is the sense path's true zero. DRV8353 CAL mode is measured only as
  /// a diagnostic (its shorted-input offset differs from the connected offset by
  /// ~0.5 A on phase C on this board). Prints a #zero line; returns false if any
  /// channel is noisy or railed. Suspends the ISR for the duration.
  /// On failure NOTHING is committed or resumed: the previous offsets are kept
  /// and both the driver outputs and the sampler are left disabled — a noisy or
  /// railed measurement must never become the zero the trip and the loop run on.
  bool zero_calibrate(espp::Logger &logger) {
    const bool was_enabled = enabled_.exchange(false);
    cfg_.driver->disable();
    std::this_thread::sleep_for(5ms);

    float cal_mean[3] = {};
    std::error_code ec;
    cfg_.bsp->gate_driver()->set_csa_calibration(true, true, true, ec);
    if (ec) {
      fmt::print("! CSA CAL mode failed ({}) — diagnostic skipped\n", ec.message());
    } else {
      float cal_sdev[3] = {};
      std::this_thread::sleep_for(10ms);
      measure_raw(32, cal_mean, cal_sdev);
      cfg_.bsp->gate_driver()->set_csa_calibration(false, false, false, ec);
      cfg_.bsp->gate_driver()->clear_faults(ec);
      std::this_thread::sleep_for(10ms);
    }

    float mean[3] = {}, sdev[3] = {};
    measure_raw(64, mean, sdev);

    // Validate into candidates first; commit only when every channel passes.
    bool ok = true;
    int cand[3];
    float cal_delta[3], noise[3];
    for (int p = 0; p < 3; p++) {
      cand[p] = (int)(mean[p] + 0.5f);
      cal_delta[p] = (mean[p] - cal_mean[p]) * amps_per_count_;
      noise[p] = sdev[p] * amps_per_count_;
      if (noise[p] > kMaxZeroNoiseAmps || mean[p] < 50.0f || mean[p] > 4045.0f)
        ok = false;
    }
    fmt::print("#zero raw=[{},{},{}] cal_delta=[{:+.3f},{:+.3f},{:+.3f}]A "
               "noise=[{:.3f},{:.3f},{:.3f}]A {}\n",
               cand[0], cand[1], cand[2], cal_delta[0], cal_delta[1], cal_delta[2], noise[0],
               noise[1], noise[2], ok ? "OK" : "BAD");
    (void)logger;

    prime(); // the reads above went through the BSP driver
    if (!ok) {
      // Keep the previous offsets; leave the outputs and the sampler down. The
      // caller decides how to report — nothing resumes on a failed zero.
      return false;
    }

    for (int p = 0; p < 3; p++) {
      raw_zero_[p] = cand[p];
      median_hist_[p][0] = median_hist_[p][1] = median_hist_[p][2] = raw_zero_[p];
      reject_run_[p] = 0;
    }
    // Measurable ceiling at this gain: the ADC rails around the zero point bound
    // how much current the soft trip can ever SEE. The worst phase/polarity is
    // the real limit — 'lim' escalations must keep trip + margin below it.
    {
      float pos = 1e9f, neg = 1e9f;
      for (int p = 0; p < 3; p++) {
        pos = std::min(pos, (float)(4095 - raw_zero_[p]) * amps_per_count_);
        neg = std::min(neg, (float)raw_zero_[p] * amps_per_count_);
      }
      fmt::print("#range ADC full scale from this zero: +{:.0f}/-{:.0f} A "
                 "(soft trip must stay below the smaller side)\n",
                 pos, neg);
    }

    cfg_.driver->enable();
    enabled_.store(was_enabled);
    return true;
  }

private:
  // Consecutive late samples before the sampler declares itself faulted and
  // disables (the stream task then forces a defined output state). Isolated
  // lates are normal — the I2C temperature reads and DRV fault polls each hold
  // the ISR off for a handful of consecutive periods — but a persistent run
  // means the control loop is effectively dead while the bridge holds its last
  // duties, which must not stand indefinitely. 40 ticks is well past any
  // bus-traffic burst and still bounds the condition to a few ms at 20 kHz.
  static constexpr int kLateStormStrikes = 40;
  static constexpr int kTripConsecutive = 3;
  static constexpr float kMaxZeroNoiseAmps = 0.5f;
  // A closed current loop moves real current ~0.1 A/period; a jump past this is a
  // glitched conversion. Well above any real per-period change, well below the
  // ~40 A spikes this board throws under rotation.
  static constexpr float kMaxJumpAmps = 5.0f;
  // Consecutive slew-limit rejections before the filter follows the raw signal
  // (bounds hold-last to ~3 per-phase samples; 1-2-long glitch runs still die).
  static constexpr int kMaxRejectRun = 3;
  // The plausibility gate (amplitude beyond which a reading is never real)
  // rides this margin above the software trip: it must sit ABOVE the trip or
  // the trip can never fire — real current rising toward the trip has to stay
  // visible to it. Re-derived on every set_trip_amps(), 18 A at the 15 A boot
  // trip.
  static constexpr float kPlausibleMarginAmps = 3.0f;
  // Double-sample diagnostic: twins differing by more than this disagree.
  static constexpr float kDblDisagreeAmps = 3.0f;
  // Pair only when the entry gap is within ~1.5 us of nominal — a later entry
  // pushes the second aperture toward the edge of the null window and the twin
  // would read switching garbage, polluting the verdict.
  static constexpr uint32_t kDblTightCycles = 360;
  // Extra in-ISR budget for the second conversion before "late" (~8.3 us).
  static constexpr uint32_t kDblExtraCycles = 2000;
  // 20 kHz PWM = 50 us = 12000 cycles at 240 MHz. A sample whose entry gap or
  // in-ISR time exceeds these is late (sampled outside the null window).
  static constexpr uint32_t kPeriodCycles = 12000;
  static constexpr uint32_t kLateGapMarginCycles = 1200; // 5 us late entry
  static constexpr uint32_t kLateCodeCycles = 2400;      // 10 us of in-ISR work (normal ~7.9 us)

  static int med3(int a, int b, int c) {
    return std::max(std::min(a, b), std::min(std::max(a, b), c));
  }

  // Choose the trustworthy value from a same-channel conversion pair (ISR
  // context, integer only). Real current can move ~0.2 A between the two
  // apertures (~8 us, freewheeling during the null vector), so:
  //  - twins agreeing within kDblDisagreeAmps: both honest -> average them
  //    (halves conversion noise for free);
  //  - twins disagreeing: one is a lie -> keep the member closer to the last
  //    accepted filter level. The lie loses regardless of which slot it's in.
  // Pairs only arrive from tight-timed entries (see on_sample), so both
  // apertures are in-window. Mechanism counters: spike_agree = both twins
  // carried a >12 A value (the pin REALLY moved — evidence against the
  // converter-lie theory); spike_first/second = which slot held a refuted
  // spike (first-heavy points at mux-hop settling; mixed points at random SAR
  // corruption).
  int dbl_pick(int phase, int a, int b) {
    dbl_n_.fetch_add(1, std::memory_order_relaxed);
    int d = a - b;
    if (d < 0)
      d = -d;
    if (d > dbl_max_delta_.load(std::memory_order_relaxed))
      dbl_max_delta_.store(d, std::memory_order_relaxed);
    int da = a - raw_zero_[phase];
    if (da < 0)
      da = -da;
    int db = b - raw_zero_[phase];
    if (db < 0)
      db = -db;
    if (d <= dbl_dis_counts_) {
      if (da > max_plausible_counts_ && db > max_plausible_counts_)
        dbl_spike_agree_.fetch_add(1, std::memory_order_relaxed);
      return (a + b) >> 1;
    }
    dbl_dis_.fetch_add(1, std::memory_order_relaxed);
    if (da > max_plausible_counts_)
      dbl_spike_first_.fetch_add(1, std::memory_order_relaxed);
    if (db > max_plausible_counts_)
      dbl_spike_second_.fetch_add(1, std::memory_order_relaxed);
    const int ref = median_hist_[phase][2];
    int ea = a - ref;
    if (ea < 0)
      ea = -ea;
    int eb = b - ref;
    if (eb < 0)
      eb = -eb;
    return ea <= eb ? a : b;
  }

  void clear_filter_counters() {
    for (int p = 0; p < 3; p++) {
      imp_count_[p].store(0);
      rej_count_[p].store(0);
      esc_count_[p].store(0);
      max_dev_[p].store(0);
    }
  }

  // Per-phase glitch rejection. ISR context, integer only; `phase` is the
  // physical channel index. Three layers:
  //
  // 1. PLAUSIBILITY gate: |i| beyond kMaxPlausibleAmps can never be real —
  //    targets clamp at 8 A, the software trip catches a real rise at 10 A on
  //    the way up, and a dead short is the DRV8353 hardware OCP's job (17-22 A).
  //    Always held, never accepted, no escape credit: kills the 30-52 A spike
  //    population regardless of burst length (3-in-a-row bursts defeated the
  //    escape-count alone and a 31 A sample got accepted -> phantom trip).
  //
  // 2. SLEW limit: reject a jump larger than kMaxJumpAmps vs the last ACCEPTED
  //    sample, hold that value — kills in-band isolated glitches that a
  //    2-in-a-row run would sneak past a bare median-of-3. Real di/dt can
  //    exceed the gate (24 V across 94 uH is ~12 A per per-phase interval; I/f
  //    hunting transients hit 5-6 A), so the hold is bounded: kMaxRejectRun
  //    consecutive in-band rejections mean the signal genuinely moved — step
  //    the accepted level toward raw by one slew bound. Without the escape the
  //    filter latches on a stale level forever and the loop winds up on
  //    garbage feedback (seen on HW).
  //
  // 3. Median-of-3 on the accepted values cleans up small noise.
  int median_filter(int phase, int raw) {
    int *h = median_hist_[phase];
    int dev = raw - raw_zero_[phase];
    if (dev < 0)
      dev = -dev;
    if (dev > max_dev_[phase].load(std::memory_order_relaxed))
      max_dev_[phase].store(dev, std::memory_order_relaxed);
    int jump = raw - h[2];
    if (jump < 0)
      jump = -jump;
    if (dev > max_plausible_counts_) {
      imp_count_[phase].fetch_add(1, std::memory_order_relaxed);
      raw = h[2]; // non-physical amplitude -> hold, no escape credit
    } else if (jump > max_jump_counts_) {
      if (++reject_run_[phase] >= kMaxRejectRun) {
        reject_run_[phase] = 0;
        esc_count_[phase].fetch_add(1, std::memory_order_relaxed);
        // Follow the move, but at the physical slew bound instead of jumping to
        // raw: real current can't change more than ~4 A per per-phase interval
        // (8 V loop limit into 188 uH line-to-line), so a real level shift is
        // tracked within a few samples, while an in-band glitch burst can pull
        // the level at most one step before it decays. (An escape that jumped
        // straight to raw accepted a decaying 18.9/14.7/10.7 A burst's tail and
        // phantom-tripped at 10.7 A.)
        raw = h[2] + (raw > h[2] ? max_jump_counts_ : -max_jump_counts_);
        h[0] = h[1] = h[2] = raw; // reseed so the median doesn't drag the old level
        return raw;
      }
      rej_count_[phase].fetch_add(1, std::memory_order_relaxed);
      raw = h[2]; // isolated non-physical jump -> hold the last accepted sample
    } else {
      reject_run_[phase] = 0;
    }
    h[0] = h[1];
    h[1] = h[2];
    h[2] = raw;
    return med3(h[0], h[1], h[2]);
  }

  void measure_raw(int n, float mean[3], float sdev[3]) {
    double acc[3] = {0, 0, 0}, acc2[3] = {0, 0, 0};
    for (int i = 0; i < n; i++) {
      for (int p = 0; p < 3; p++) {
        const double v = read_raw_async(p);
        acc[p] += v;
        acc2[p] += v * v;
      }
      std::this_thread::sleep_for(1ms);
    }
    for (int p = 0; p < 3; p++) {
      mean[p] = (float)(acc[p] / n);
      const double var = acc2[p] / n - (acc[p] / n) * (acc[p] / n);
      sdev[p] = (float)std::sqrt(var > 0 ? var : 0);
    }
  }

  // Not IRAM_ATTR: MCPWM_ISR_IRAM_SAFE=n, so this ISR runs with the flash cache
  // enabled and may live in flash like any other code (marking it IRAM here also
  // mis-orders the literal pool at -O2 -> "dangerous relocation").
  static bool isr_trampoline(mcpwm_timer_handle_t t, const mcpwm_timer_event_data_t *ed,
                             void *ctx) {
    return s_self ? s_self->on_sample() : false;
  }

  bool on_sample() {
    if (!enabled_.load(std::memory_order_relaxed))
      return false;
    if (++skip_ < decimation_.load(std::memory_order_relaxed))
      return false;
    skip_ = 0;

    // ISR entry time, before anything, for the late-sample check below. TEP
    // events are exactly decimation*period apart; a longer gap means this ISR
    // fired late (preempted before it even started) and its sample lands outside
    // the null window.
    const uint32_t t_entry = esp_cpu_get_cycle_count();
    const uint32_t gap = t_entry - last_entry_;
    const bool had_last = have_last_entry_;
    last_entry_ = t_entry;
    have_last_entry_ = true;

    if (cfg_.scope_pin != GPIO_NUM_NC)
      gpio_set_level(cfg_.scope_pin, 1);
    const uint32_t t0 = t_entry;

    const int px = phase_x_.load(std::memory_order_relaxed);
    const int py = phase_y_.load(std::memory_order_relaxed);

    // Read the raw conversion(s) into locals; do NOT commit to raw_x_/raw_y_ yet
    // — the timing check below decides whether this sample is trustworthy.
    uint32_t t_done;
    int raw = 0, cap_raw = 0, cap_ph;
    int new_x = 0, new_y = 0;
    bool upd_x = false, upd_y = false;
    int dbl_raw2 = 0;
    bool took_pair = false;
    if (samples_per_isr_.load(std::memory_order_relaxed) >= 2) {
      adc_fast_select_channel(ADC_UNIT_1, cfg_.phase_channels[px]);
      adc_oneshot_hal_convert(&hal_, &raw);
      new_x = raw;
      upd_x = true;
      adc_fast_select_channel(ADC_UNIT_1, cfg_.phase_channels[py]);
      adc_oneshot_hal_convert(&hal_, &raw);
      new_y = raw;
      upd_y = true;
      t_done = esp_cpu_get_cycle_count();
      cap_raw = raw;
      cap_ph = py;
    } else {
      // Alternate phases, one per period (~50 us skew on the other). The
      // channel for THIS conversion was selected at the END of the previous
      // ISR, so the mux has settled for a full period: the double-sample
      // experiment proved the conversion taken ~1 us after a channel hop is
      // the corrupted one (1322/1322 refuted spikes in the post-hop slot, 0 in
      // the settled slot; 37% of post-hop conversions off by >3 A at idle).
      int ph = pending_ph_;
      if (ph != px && ph != py) {
        // The pair changed since the pre-select ('p x y' or auto-select): the
        // mux is still physically parked on the OLD channel, so converting now
        // and labeling the result as a member of the new pair would inject
        // another phase's current into the control loop. Re-target the mux to
        // the new pair and DISCARD this tick — the next ISR converts the newly
        // selected channel after a full period of settling, exactly like every
        // other pre-selected conversion.
        adc_fast_select_channel(ADC_UNIT_1, cfg_.phase_channels[px]);
        pending_ph_ = px;
        if (cfg_.scope_pin != GPIO_NUM_NC)
          gpio_set_level(cfg_.scope_pin, 0);
        return false;
      }
      adc_oneshot_hal_convert(&hal_, &raw);
      if (ph == py) {
        new_y = raw;
        upd_y = true;
      } else {
        new_x = raw;
        upd_x = true;
      }
      t_done = esp_cpu_get_cycle_count();
      cap_raw = raw;
      cap_ph = ph;
      // Verification pair (dbl 1): re-convert the same settled channel. With
      // pre-selection both slots should now be honest -> dis ~0 proves the
      // hop-settling mechanism; dbl_pick still feeds the sane choice. Tight
      // entries only: a late-ish entry pushes the twin's aperture out of the
      // null window (systematic ~0 A read) and the closer-to-level pick would
      // prefer that lie during current rise (caused real overcurrent on HW).
      if (dbl_.load(std::memory_order_relaxed) && had_last &&
          gap <= decimation_.load(std::memory_order_relaxed) * kPeriodCycles + kDblTightCycles) {
        adc_oneshot_hal_convert(&hal_, &dbl_raw2);
        t_done = esp_cpu_get_cycle_count();
        const int chosen = dbl_pick(ph, raw, dbl_raw2);
        if (ph == py)
          new_y = chosen;
        else
          new_x = chosen;
        took_pair = true;
      }
      // Pre-select the next tick's channel so it settles for a full period.
      const int nxt = (ph == px) ? py : px;
      adc_fast_select_channel(ADC_UNIT_1, cfg_.phase_channels[nxt]);
      pending_ph_ = nxt;
    }

    const uint32_t elapsed = t_done - t0;
    if (cfg_.scope_pin != GPIO_NUM_NC)
      gpio_set_level(cfg_.scope_pin, 0);

    // Diagnostic ring of the pre-median conversion + how long the ISR took, so a
    // glitched sample can be correlated with a late (preempted) ISR. Frozen on
    // trip so the dump shows what led up to it.
    if (!cap_frozen_.load(std::memory_order_relaxed)) {
      uint32_t h = cap_head_.load(std::memory_order_relaxed);
      const uint16_t us = (uint16_t)(elapsed > 65535 ? 65535 : elapsed);
      cap_[h++ % kCapN] = {(uint16_t)cap_raw, us, (uint8_t)cap_ph};
      // In double-sample mode the twin lands as the next ring entry (same
      // phase back-to-back), so a dump shows the pairs adjacently.
      if (took_pair)
        cap_[h++ % kCapN] = {(uint16_t)dbl_raw2, us, (uint8_t)cap_ph};
      cap_head_.store(h, std::memory_order_relaxed);
    }

    n_.fetch_add(1, std::memory_order_relaxed);
    cyc_sum_.fetch_add(elapsed, std::memory_order_relaxed);
    if (elapsed < cyc_min_.load(std::memory_order_relaxed))
      cyc_min_.store(elapsed);
    if (elapsed > cyc_max_.load(std::memory_order_relaxed))
      cyc_max_.store(elapsed);

    // Late-sample rejection. Two ways a sample lands outside the null window:
    //  - late ENTRY: the ISR fired late (gap since the previous entry exceeds the
    //    expected decimation*period). Its conversion samples during high-side
    //    conduction -> garbage, at normal in-ISR elapsed.
    //  - mid-code PREEMPTION: elapsed itself is long (a higher-prio ISR ran
    //    between channel-select and conversion), so the sample was taken late.
    // Either way, discard: hold the last good raw, skip the median and the trip.
    // This is the real fix for the phase glitches — they were late samples, not
    // bad channels (see the capture dump: glitches coincide with long gap/elapsed).
    const uint32_t expected_gap = decimation_.load(std::memory_order_relaxed) * kPeriodCycles;
    const uint32_t code_budget = kLateCodeCycles + (took_pair ? kDblExtraCycles : 0);
    const bool late =
        (had_last && gap > expected_gap + kLateGapMarginCycles) || (elapsed > code_budget);
    if (late) {
      late_count_.fetch_add(1, std::memory_order_relaxed);
      // A SUSTAINED run of late samples means the control loop is effectively
      // dead while the bridge holds its last duties — that must not stand.
      // Declare a fault: disable and signal the stream task, which forces the
      // outputs to a defined state (centered duties + Hi-Z coast).
      if (++late_run_ >= kLateStormStrikes) {
        late_run_ = 0;
        enabled_.store(false, std::memory_order_relaxed);
        overran_.store(true, std::memory_order_relaxed);
        return false;
      }
      // An ISOLATED late sample is discarded: do NOT notify the control task —
      // running the current loop on a held, stale current sample is wrong once
      // the frame is rotating (the sample belongs to an earlier electrical
      // angle). Skip this tick — the task keeps the last duties until a fresh
      // sample arrives, and its dt accounts for the gap. In stage 2a (parked)
      // this was harmless; while spinning it matters.
      return false;
    }
    late_run_ = 0;

    if (upd_x)
      raw_x_.store(median_filter(px, new_x), std::memory_order_relaxed);
    if (upd_y)
      raw_y_.store(median_filter(py, new_y), std::memory_order_relaxed);

    // Integer overcurrent trip. Consecutive samples only — a lone over-limit
    // sample is far more likely a conversion outside the window than a real fault.
    const int dx = raw_x_.load(std::memory_order_relaxed) - raw_zero_[px];
    const int dy = raw_y_.load(std::memory_order_relaxed) - raw_zero_[py];
    // Reconstructed third phase: iz = -(ix + iy), so its count delta is -(dx+dy).
    // Check it too — under rotating vectors the highest-current phase is often
    // the reconstructed one, and the measured pair alone would miss it.
    const int dz = -(dx + dy);
    const int lim = trip_counts_;
    const bool over = dx > lim || dx < -lim || dy > lim || dy < -lim || dz > lim || dz < -lim;
    over_count_ = over ? over_count_ + 1 : 0;
    if (over_count_ >= kTripConsecutive) {
      over_count_ = 0;
      // SOFT trip: record and signal, but do NOT brake or disable. The old
      // comparator=0 brake shorted the back-EMF at speed (~30 A bang) — the
      // "violent stop" that then fired the hardware VDS OCP, making phantom
      // trips indistinguishable from real ones. Instead the FOC task unloads
      // to 0 A with the loop closed (STOPPING) and the stream applies Hi-Z
      // coast; the DRV8353 VDS OCP (17-22 A, latched, responds by going Hi-Z
      // itself) remains the hard layer for genuine shorts.
      trip_dx_.store(dx, std::memory_order_relaxed);
      trip_dy_.store(dy, std::memory_order_relaxed);
      trip_px_.store(px, std::memory_order_relaxed);
      trip_py_.store(py, std::memory_order_relaxed);
      // Which limit fired (the reconstructed phase is a common culprit), and
      // this tick's pre-filter conversion for a filtered-vs-raw comparison.
      int tp = 3 - px - py;
      if (dx > lim || dx < -lim)
        tp = px;
      else if (dy > lim || dy < -lim)
        tp = py;
      trip_ph_.store(tp, std::memory_order_relaxed);
      trip_raw_.store(cap_raw - raw_zero_[cap_ph], std::memory_order_relaxed);
      trip_raw_ph_.store(cap_ph, std::memory_order_relaxed);
      cap_frozen_.store(true, std::memory_order_relaxed); // keep the run-up to the trip
      tripped_.store(true, std::memory_order_relaxed);
      soft_trip_.store(true, std::memory_order_relaxed);
    }

    // Hand the fresh sample to the FOC task for the float math.
    BaseType_t hpw = pdFALSE;
    if (notify_task_)
      vTaskNotifyGiveFromISR(notify_task_, &hpw);
    return hpw == pdTRUE;
  }

  static FocSampler *s_self;

  Config cfg_{};
  adc_oneshot_hal_ctx_t hal_{};
  adc_cali_handle_t cali_{nullptr};
  PwmComparators cmp_{};
  TaskHandle_t notify_task_{nullptr};

  float amps_per_count_{0.0f};
  int trip_counts_{0};
  // Glitch rejection state — see median_filter for the full story (slew gate
  // with a bounded hold, then median-of-3). median_hist_[phase][2] is the last
  // accepted sample.
  int median_hist_[3][3] = {};
  int reject_run_[3] = {0, 0, 0}; // consecutive slew-limit rejections per phase
  int max_jump_counts_{0};        // slew-limit threshold in raw counts
  int max_plausible_counts_{0};   // plausibility threshold in raw counts (vs zero)
  int dbl_dis_counts_{0};         // pair-disagreement threshold in raw counts

  std::atomic<bool> dbl_{false}; // verification pair mode ('dbl 1'); normal op
                                 // is a single settled conversion (~8 us ISR)
  std::atomic<uint32_t> dbl_n_{0}, dbl_dis_{0};
  std::atomic<uint32_t> dbl_spike_agree_{0}, dbl_spike_first_{0}, dbl_spike_second_{0};
  std::atomic<int> dbl_max_delta_{0};
  int raw_zero_[3] = {0, 0, 0};
  bool at_tep_{true};

  std::atomic<int> phase_x_{0};
  std::atomic<int> phase_y_{1};
  std::atomic<int> raw_x_{0};
  std::atomic<int> raw_y_{0};

  std::atomic<bool> enabled_{false};
  std::atomic<bool> tripped_{false};
  std::atomic<bool> soft_trip_{false};
  std::atomic<bool> overran_{false};
  std::atomic<uint32_t> decimation_{1};
  std::atomic<int> samples_per_isr_{1};
  std::atomic<int> trip_dx_{0}, trip_dy_{0};
  std::atomic<int> trip_px_{0}, trip_py_{0};
  std::atomic<int> trip_ph_{0};     // phase whose limit fired (incl. reconstructed)
  std::atomic<int> trip_raw_{0};    // trip tick's pre-filter conversion, counts
  std::atomic<int> trip_raw_ph_{0}; // phase of that conversion

  // ISR-only scratch (single writer, no atomics needed; pending_ph_ is also
  // written by init/prime, but only while the sampler is disabled; late_run_
  // is also reset by enable(), only while disabled).
  uint32_t skip_{0};
  int pending_ph_{0}; // phase whose channel is currently selected (settled)
  int late_run_{0};   // consecutive late samples -> kLateStormStrikes fault
  int over_count_{0};
  uint32_t last_entry_{0};
  bool have_last_entry_{false};

  std::atomic<uint32_t> n_{0};
  std::atomic<uint32_t> cyc_min_{0xffffffff};
  std::atomic<uint32_t> cyc_max_{0};
  std::atomic<uint64_t> cyc_sum_{0};
  std::atomic<uint32_t> late_count_{0};
  // Per-phase filter counters (cleared on arm and on stats reset). These are
  // the H1-vs-H0 discriminator for the trip investigation: glitch-caused trips
  // show imp >> 0 with a flat capture baseline; a real-current trip shows
  // imp ~ 0 and a baseline ramping into the limit.
  std::atomic<uint32_t> imp_count_[3] = {}; // implausible amplitude, held
  std::atomic<uint32_t> rej_count_[3] = {}; // in-band slew rejection, held
  std::atomic<uint32_t> esc_count_[3] = {}; // escapes: filter followed a real move
  std::atomic<int> max_dev_[3] = {};        // largest |raw - zero| seen, counts

  static constexpr int kCapN = 256;
  struct CapSample {
    uint16_t raw;
    uint16_t cyc;
    uint8_t phase;
  };
  CapSample cap_[kCapN] = {};
  std::atomic<uint32_t> cap_head_{0};
  std::atomic<bool> cap_frozen_{false};
};

inline FocSampler *FocSampler::s_self = nullptr;

} // namespace sensorless
