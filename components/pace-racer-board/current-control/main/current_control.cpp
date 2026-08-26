// Phase-current PI tuning harness.
//
// Drives DC current through phases A->B (C held at midpoint, so ic ~= 0) with
// the rotor parked at a detent. Two step types, commanded over the USB console:
//
//   v <volts> <ms>              open-loop voltage step (plant identification)
//   c <kp> <ki> <amps> <ms>     closed-loop PI current step (verification)
//   s <+1|-1>                   set current-feedback sign (from ID polarity)
//
// Each run captures (t_us, i_amps, u_volts) to RAM at the loop rate, then dumps
// CSV framed by "#dump ..." / "#end ok|trip" for tune.py to parse. The plant
// seen here is the two windings in series: R = 2*R_phase, L = 2*L_phase.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "esp_timer.h"

#include "high_resolution_timer.hpp"
#include "logger.hpp"
#include "pace-racer-board.hpp"

using namespace std::chrono_literals;

static constexpr float kPowerSupplyVoltage = 48.0f; // set to your bench supply
static constexpr float kVoltageLimit       = 8.0f;  // max differential drive volts
static constexpr float kTripAmps           = 12.0f; // hard overcurrent abort
static constexpr uint64_t kLoopPeriodUs    = 200;   // 5 kHz target; real dt is logged
static constexpr uint32_t kMaxDurationMs   = 1500;
static constexpr size_t kMaxSamples        = 8000;
// Async ADC sampling occasionally catches a PWM switching edge: lone samples
// tens of amps off. Median-of-3 rejects a single glitch outright (a mean only
// dilutes it), and the software trip needs consecutive over-limit samples.
static constexpr int kTripConsecutive      = 3; // ~600 us sustained at 5 kHz

struct Sample {
  uint32_t t_us;
  float i;
  float u;
};
static Sample g_samples[kMaxSamples];
static size_t g_n = 0;

enum class Mode : int { Idle, OpenLoop, ClosedLoop };
static std::atomic<Mode> g_mode{Mode::Idle};
static std::atomic<bool> g_done{false};
static std::atomic<bool> g_trip{false};
// Written by the stdin task only while Idle, read by the timer while running.
static float g_kp = 0, g_ki = 0, g_target = 0; // target: volts (OpenLoop) or amps
static uint64_t g_duration_us = 0;
static float g_sign = 1.0f;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "current-control", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup — phase current PI tuning harness");

  using Bsp = espp::PaceRacerBoard;
  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::WARN);

  // We only use the gate driver + BldcDriver, but init_motor also constructs a
  // BldcMotor and initialize() would run sensor alignment — which DRIVES THE
  // MOTOR open-loop at boot. Non-zero offset + known direction skips it; this
  // harness never touches the motor object again.
  auto motor_cfg = bsp.default_motor_config;
  motor_cfg.zero_electric_offset = 1e-6f;
  motor_cfg.sensor_direction = espp::detail::SensorDirection::CLOCKWISE;
  if (!bsp.init_motor(motor_cfg, {.power_supply_voltage = kPowerSupplyVoltage,
                                  .limit_voltage        = kVoltageLimit})) {
    logger.error("BSP motor init failed");
    return;
  }

  {
    std::error_code ec;
    // ESP32 reboot does NOT power-cycle the DRV8353 — clear any latched faults.
    bsp.gate_driver()->clear_faults(ec);
    if (ec) logger.error("Failed to clear DRV8353 faults: {}", ec.message());
  }

  // Hardware overcurrent protection, independent of the software loop — the
  // board must survive a bad gain calculation or parameter typo.
  //  - VDS_OCP 0.06 V: ISC0802NLS Rds(on) is 2.7 mΩ typ / 3.6 mΩ max @ 10 V,
  //    so this trips at ~17-22 A (lower when hot) — above the 12 A software
  //    trip, far below FET/board limits. 12 A * 3.6 mΩ = 43 mV: margin, no
  //    nuisance trips. 4 µs deglitch rides out switching transients.
  //  - SEN_OCP 0.25 V across the 1 mΩ shunt = 250 A: dead-short catch only.
  //  - LATCHED_SHUTDOWN: a tripped test harness stays off until investigated.
  {
    using GD = Bsp::GateDriver;
    std::error_code ec;
    auto gd = bsp.gate_driver();
    // Write DRIVER_CONTROL to a known-good state outright (6x PWM, coast/brake
    // off) rather than read-modify-write: earlier binaries with broken SPI
    // reads poisoned this register (COAST latched -> bridge Hi-Z), and RMW
    // faithfully preserves the poison.
    GD::DriverControl drv_ctl{.raw = 0x0000};
    bool ok = gd->write_driver_control(drv_ctl, ec) &&
              gd->set_ocp_mode(GD::OcpMode::LATCHED_SHUTDOWN, ec) &&
              gd->set_ocp_deglitch(GD::OcpDeglitch::US_4, ec) &&
              gd->set_vds_level(GD::VdsLevel::V_0_06, ec) &&
              gd->set_sense_overcurrent_enabled(true, ec) &&
              gd->set_sense_level(GD::SenseLevel::V_0_25, ec) &&
              // Chip default is GAIN_20; the BSP's mv->A constant assumes GAIN_5.
              gd->set_csa_gain(GD::CsaGain::GAIN_5, ec);
    gd->clear_faults(ec); // protected-register writes can latch faults
    // Verify by read-back — refuse to run unprotected.
    auto ocp = gd->read_ocp_control(ec);
    auto csa = gd->read_csa_control(ec);
    auto dcv = gd->read_driver_control(ec);
    if (!ok || ec || dcv.coast() || dcv.brake() ||
        ocp.ocp_mode() != GD::OcpMode::LATCHED_SHUTDOWN ||
        ocp.deglitch() != GD::OcpDeglitch::US_4 || // non-zero encoding: catches dead SPI reads
        ocp.vds_level() != GD::VdsLevel::V_0_06 || csa.sense_overcurrent_disabled() ||
        csa.sense_level() != GD::SenseLevel::V_0_25 || csa.gain() != GD::CsaGain::GAIN_5) {
      logger.error("Failed to configure DRV8353 OCP (ocp=0x{:04x} csa=0x{:04x}) — not running",
                   ocp.raw, csa.raw);
      return;
    }
    logger.info("DRV8353 OCP armed: VDS 0.06 V (~17-22 A), SEN 0.25 V, latched shutdown");
  }

  // Fast phase-A read path: the BSP's motor_current_a_amps() oversamples 8x and
  // re-reads vref every call (~16 conversions) — far too slow for a 5 kHz loop.
  // Cache vref once and read the phase-A channel directly. Config mirrors the
  // BSP's private current_sense_m_a_ (GPIO4).
  espp::AdcConfig ch_a = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_3,
      .attenuation = ADC_ATTEN_DB_6,
  };
  auto &adc = bsp.adc1();

  float vref_mv = 0;
  for (int i = 0; i < 64; i++) vref_mv += bsp.motor_current_sense_vref();
  vref_mv /= 64;

  auto mean_mv = [&](const espp::AdcConfig &ch) {
    float sum = 0;
    for (int i = 0; i < 32; i++) sum += (float)adc.read_mv(ch).value_or(0);
    return sum / 32;
  };
  logger.info("raw mv: vref={:.1f} ch_a={:.1f} (idle, gates off)", vref_mv, mean_mv(ch_a));

  auto read_amps_raw = [&]() {
    float a = (float)adc.read_mv(ch_a).value_or((int)vref_mv);
    float b = (float)adc.read_mv(ch_a).value_or((int)vref_mv);
    float c = (float)adc.read_mv(ch_a).value_or((int)vref_mv);
    float med = std::max(std::min(a, b), std::min(std::max(a, b), c));
    return (med - vref_mv) * Bsp::motor_current_sense_mv_to_a();
  };

  // CSA zero offset via DRV8353 CAL mode (shorts amp inputs), same as hall-foc,
  // but measured through our fast read path so the offset matches it.
  float csa_zero = 0.0f;
  {
    std::error_code cal_ec;
    bsp.gate_driver()->set_csa_calibration(true, true, true, cal_ec);
    if (cal_ec) {
      logger.warn("CSA CAL mode failed ({}), zero offset will be 0", cal_ec.message());
    } else {
      std::this_thread::sleep_for(10ms);
      constexpr int kCalN = 32;
      for (int i = 0; i < kCalN; i++) {
        csa_zero += read_amps_raw();
        std::this_thread::sleep_for(1ms);
      }
      csa_zero /= kCalN;
      logger.info("raw mv in CAL: ch_a={:.1f} vref_now={:.1f} (cached vref={:.1f})",
                  mean_mv(ch_a), bsp.motor_current_sense_vref(), vref_mv);
      bsp.gate_driver()->set_csa_calibration(false, false, false, cal_ec);
      bsp.gate_driver()->clear_faults(cal_ec); // write_protected_register can latch faults
      logger.info("CSA zero offset A: {:.3f} A", csa_zero);
    }
  }
  // The raw zero is large (~-64 A): bidirectional CSA rests at VREF/2 while the
  // vref ADC channel reads a different node — a constant that subtracts out.
  // The real sanity check: with gates off and the zero applied, current ~ 0.
  {
    float resid = 0;
    for (int i = 0; i < 32; i++) resid += read_amps_raw() - csa_zero;
    resid /= 32;
    if (std::fabs(resid) > 0.5f) {
      logger.error("Zero-current residual {:.2f} A after cal — sensing broken, not running",
                   resid);
      return;
    }
    logger.info("Zero-current residual: {:.3f} A — sensing OK", resid);
  }

  auto driver = bsp.motor_driver();
  driver->enable();
  driver->set_voltage(0, 0, 0); // idle: brake (all low-side on), zero current

  // Differential drive around the midpoint: ua-ub = u, uc at mid so ic ~= 0.
  constexpr float kMid = kVoltageLimit / 2.0f;
  auto apply = [&](float u) {
    u = std::clamp(u, -kVoltageLimit, kVoltageLimit);
    driver->set_voltage(kMid + u / 2.0f, kMid - u / 2.0f, kMid);
    return u;
  };

  auto timer_fn = [&]() -> bool {
    static int64_t t0 = 0, t_prev = 0;
    static float integ = 0;
    static int over_count = 0;
    Mode m = g_mode.load(std::memory_order_acquire);
    if (m == Mode::Idle) {
      t0 = 0;
      return false;
    }
    int64_t now = esp_timer_get_time();
    if (t0 == 0) { // first tick of a run
      t0 = now;
      t_prev = now;
      integ = 0;
      over_count = 0;
    }
    float i = g_sign * (read_amps_raw() - csa_zero);

    over_count = std::fabs(i) > kTripAmps ? over_count + 1 : 0;
    bool trip = over_count >= kTripConsecutive;
    if (trip || (uint64_t)(now - t0) >= g_duration_us) {
      driver->set_voltage(0, 0, 0); // brake — winding current freewheels to zero
      g_trip.store(trip);
      g_mode.store(Mode::Idle, std::memory_order_release);
      g_done.store(true, std::memory_order_release);
      return false;
    }

    float u;
    if (m == Mode::OpenLoop) {
      u = g_target;
    } else {
      float dt = (float)(now - t_prev) * 1e-6f;
      float e = g_target - i;
      integ = std::clamp(integ + g_ki * e * dt, -kVoltageLimit, kVoltageLimit);
      u = g_kp * e + integ;
    }
    t_prev = now;
    u = apply(u);

    if (g_n < kMaxSamples) g_samples[g_n++] = {(uint32_t)(now - t0), i, u};
    return false;
  };

  auto timer = espp::HighResolutionTimer({.name      = "ctrl",
                                          .callback  = timer_fn,
                                          .log_level = espp::Logger::Verbosity::WARN});
  timer.periodic(kLoopPeriodUs);

  auto run = [&](Mode mode, float kp, float ki, float target, uint32_t ms) {
    ms = std::min(ms, kMaxDurationMs);
    g_kp = kp;
    g_ki = ki;
    g_target = mode == Mode::OpenLoop ? std::clamp(target, -kVoltageLimit, kVoltageLimit)
                                      : std::clamp(target, -kTripAmps, kTripAmps);
    g_duration_us = (uint64_t)ms * 1000;
    g_n = 0;
    g_done.store(false);
    g_trip.store(false);
    g_mode.store(mode, std::memory_order_release);
    while (!g_done.load(std::memory_order_acquire)) std::this_thread::sleep_for(10ms);

    fmt::print("#dump {} kp={:.6g} ki={:.6g} target={:.4g} sign={:+.0f} n={} limit={:.2f}\n",
               mode == Mode::OpenLoop ? "v" : "c", g_kp, g_ki, g_target, g_sign, g_n,
               kVoltageLimit);
    fmt::print("t_us,i_a,u_v\n");
    for (size_t j = 0; j < g_n; j++)
      fmt::print("{},{:.4f},{:.4f}\n", g_samples[j].t_us, g_samples[j].i, g_samples[j].u);

    // A hardware OCP trip latches the DRV8353 off mid-run (current collapses to
    // zero in the data). Surface it — the harness stays latched deliberately.
    std::error_code fec;
    auto fs = bsp.gate_driver()->fault_status(fec);
    if (!fec && fs.raw != 0)
      fmt::print("#end fault 0x{:04x}\n", fs.raw);
    else
      fmt::print("#end {}\n", g_trip.load() ? "trip" : "ok");
  };

  setvbuf(stdin, nullptr, _IONBF, 0);
  fmt::print("#ready\n");

  // USB-Serial-JTAG stdin is non-blocking: getchar() returns EOF when idle
  // (same pattern as temp-sweep) — accumulate a line by hand.
  char line[128];
  size_t len = 0;
  while (true) {
    int ch = getchar();
    if (ch == EOF) {
      std::this_thread::sleep_for(50ms);
      continue;
    }
    if (ch != '\n' && ch != '\r') {
      if (len < sizeof(line) - 1) line[len++] = (char)ch;
      continue;
    }
    if (len == 0) continue;
    line[len] = '\0';
    len = 0;
    float a = 0, b = 0, c = 0;
    unsigned ms = 0;
    if (sscanf(line, "v %f %u", &a, &ms) == 2) {
      run(Mode::OpenLoop, 0, 0, a, ms);
    } else if (sscanf(line, "c %f %f %f %u", &a, &b, &c, &ms) == 4) {
      run(Mode::ClosedLoop, a, b, c, ms);
    } else if (sscanf(line, "s %f", &a) == 1 && (a == 1.0f || a == -1.0f)) {
      g_sign = a;
      fmt::print("#sign {:+.0f}\n", g_sign);
    } else if (line[0] == 'r') {
      // Read-only DRV8353 register dump for diagnosing a dead power stage.
      std::error_code rec;
      auto gd = bsp.gate_driver();
      auto regs = gd->read_all_registers(rec);
      if (rec) {
        fmt::print("#err register read failed: {}\n", rec.message());
      } else {
        fmt::print("#regs fault1=0x{:04x} vgs2=0x{:04x} drv_ctl=0x{:04x} "
                   "gate_hs=0x{:04x} gate_ls=0x{:04x} ocp=0x{:04x} csa=0x{:04x} "
                   "nfault_active={:d}\n",
                   regs.fault_status_1, regs.vgs_status_2, regs.driver_control,
                   regs.gate_drive_hs, regs.gate_drive_ls, regs.ocp_control,
                   regs.csa_control, gd->fault_pin_active() ? 1 : 0);
      }
    } else {
      fmt::print("#err unknown command\n");
    }
  }
}
