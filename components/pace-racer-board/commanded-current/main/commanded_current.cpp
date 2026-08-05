// Commanded phase-current app: runs the tuned current PI continuously on the
// A->B path (C at midpoint) and takes setpoints over the USB console.
//
// Console protocol (line-based; monitor.py is the intended client):
//   i <amps>       set target current (clamped to +/-kMaxTargetAmps)
//   g <kp> <ki>    update PI gains
//   o / f / space  enable / disable / toggle (disable zeroes the target)
//   r              dump DRV8353 registers
//
// Streams CSV at 25 Hz:  %time(s), target_a, i_a, u_v, enabled
// Alert lines start with '!' (trip, DRV fault) and are ignored by the parser.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "esp_timer.h"

#include "high_resolution_timer.hpp"
#include "logger.hpp"
#include "pace-racer-board.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

static constexpr float kPowerSupplyVoltage = 48.0f; // set to your bench supply
static constexpr float kVoltageLimit       = 24.0f; // hot winding needs ~0.45 Ω × 40 A
static constexpr float kMaxTargetAmps      = 40.0f;
static constexpr float kTripAmps           = 45.0f; // shunt-based, temperature-independent
static constexpr int kTripConsecutive      = 3; // lone ADC glitches are not trips
// 5 kHz. The loop's esp_timer task is pinned to CPU1 (sdkconfig.defaults:
// ESP_TIMER_TASK_AFFINITY_CPU1) — on CPU0 it starved the console task and the
// motor went uncommandable while enabled. 2.5 kHz "fixed" that but doubled the
// discrete correction steps and the current ripple tripped OCP at 22 A.
static constexpr uint64_t kLoopPeriodUs    = 200;

// Tuned by tune.py on 2026-07-14 (48 V VM): R=0.15 ohm/phase, L=94 uH/phase.
static constexpr float kDefaultKp = 0.0754f; // V/A
static constexpr float kDefaultKi = 119.6f;  // V/(A*s)

static std::atomic<float> g_target{0.0f};
static std::atomic<float> g_kp{kDefaultKp};
static std::atomic<float> g_ki{kDefaultKi};
static std::atomic<bool> g_enabled{false};
static std::atomic<bool> g_tripped{false};
static std::atomic<float> g_i_disp{0.0f}; // lowpassed current for the 25 Hz stream
static std::atomic<float> g_u_disp{0.0f};

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "commanded-current", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup — commanded phase current (A->B), tuned PI");

  using Bsp = espp::PaceRacerBoard;
  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::WARN);

  // Skip BldcMotor sensor alignment — it DRIVES THE MOTOR at boot.
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
    bsp.gate_driver()->clear_faults(ec);
    if (ec) logger.error("Failed to clear DRV8353 faults: {}", ec.message());
  }

  // Hardware protection + known-good driver state (see current-control for the
  // full story: DRIVER_CONTROL force-write immunizes against poisoned RMW).
  {
    using GD = Bsp::GateDriver;
    std::error_code ec;
    auto gd = bsp.gate_driver();
    GD::DriverControl drv_ctl{.raw = 0x0000};
    bool ok = gd->write_driver_control(drv_ctl, ec) &&
              gd->set_ocp_mode(GD::OcpMode::LATCHED_SHUTDOWN, ec) &&
              gd->set_ocp_deglitch(GD::OcpDeglitch::US_4, ec) &&
              // 0.30 V: coarse hot-FET backstop (~55-75 A at 1.5x Rds(on) @ 100C).
              // The 45 A soft trip on the shunt is the precise, temp-independent guard.
              gd->set_vds_level(GD::VdsLevel::V_0_30, ec) &&
              gd->set_sense_overcurrent_enabled(true, ec) &&
              gd->set_sense_level(GD::SenseLevel::V_0_25, ec) &&
              gd->set_csa_gain(GD::CsaGain::GAIN_5, ec);
    gd->clear_faults(ec);
    auto ocp = gd->read_ocp_control(ec);
    auto csa = gd->read_csa_control(ec);
    auto dcv = gd->read_driver_control(ec);
    if (!ok || ec || dcv.coast() || dcv.brake() ||
        ocp.ocp_mode() != GD::OcpMode::LATCHED_SHUTDOWN ||
        ocp.deglitch() != GD::OcpDeglitch::US_4 ||
        ocp.vds_level() != GD::VdsLevel::V_0_30 || csa.sense_overcurrent_disabled() ||
        csa.sense_level() != GD::SenseLevel::V_0_25 || csa.gain() != GD::CsaGain::GAIN_5) {
      logger.error("Failed to configure DRV8353 (ocp=0x{:04x} csa=0x{:04x} drv=0x{:04x}) — "
                   "not running. Is VM on?",
                   ocp.raw, csa.raw, dcv.raw);
      return;
    }
    logger.info("DRV8353 OCP armed: VDS 0.30 V (hot backstop ~55-75 A), SEN 0.25 V, "
                "latched shutdown; 45 A shunt soft trip is the primary guard");
  }

  // Fast read paths for phases A and B (mirror BSP's private configs, GPIO4/5).
  // Feedback comes from the PARKED phase (~2% duty): its low-side FET conducts
  // ~98% of the time, so async samples are valid regardless of drive voltage.
  // Sampling the driven phase corrupted feedback at high duty — low-side shunts
  // read 0 A whenever the high side is on, and the PI chased the phantom error.
  espp::AdcConfig ch_a = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_3,
      .attenuation = ADC_ATTEN_DB_6,
  };
  espp::AdcConfig ch_b = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_4,
      .attenuation = ADC_ATTEN_DB_6,
  };
  auto &adc = bsp.adc1();

  float vref_mv = 0;
  for (int i = 0; i < 64; i++) vref_mv += bsp.motor_current_sense_vref();
  vref_mv /= 64;

  auto read_amps_raw = [&](const espp::AdcConfig &ch) {
    // median-of-3: rejects lone samples that catch a PWM switching edge
    float a = (float)adc.read_mv(ch).value_or((int)vref_mv);
    float b = (float)adc.read_mv(ch).value_or((int)vref_mv);
    float c = (float)adc.read_mv(ch).value_or((int)vref_mv);
    float med = std::max(std::min(a, b), std::min(std::max(a, b), c));
    return (med - vref_mv) * Bsp::motor_current_sense_mv_to_a();
  };

  float csa_zero_a = 0.0f, csa_zero_b = 0.0f;
  {
    std::error_code cal_ec;
    bsp.gate_driver()->set_csa_calibration(true, true, true, cal_ec);
    if (cal_ec) {
      logger.warn("CSA CAL mode failed ({}), zero offsets will be 0", cal_ec.message());
    } else {
      std::this_thread::sleep_for(10ms);
      constexpr int kCalN = 32;
      for (int i = 0; i < kCalN; i++) {
        csa_zero_a += read_amps_raw(ch_a);
        csa_zero_b += read_amps_raw(ch_b);
        std::this_thread::sleep_for(1ms);
      }
      csa_zero_a /= kCalN;
      csa_zero_b /= kCalN;
      bsp.gate_driver()->set_csa_calibration(false, false, false, cal_ec);
      bsp.gate_driver()->clear_faults(cal_ec);
      logger.info("CSA zero offsets: A={:.3f} A B={:.3f} A", csa_zero_a, csa_zero_b);
    }
  }
  {
    float resid_a = 0, resid_b = 0;
    for (int i = 0; i < 32; i++) {
      resid_a += read_amps_raw(ch_a) - csa_zero_a;
      resid_b += read_amps_raw(ch_b) - csa_zero_b;
    }
    resid_a /= 32;
    resid_b /= 32;
    if (std::fabs(resid_a) > 0.5f || std::fabs(resid_b) > 0.5f) {
      logger.error("Zero-current residual A={:.2f} B={:.2f} A after cal — sensing broken, "
                   "not running",
                   resid_a, resid_b);
      return;
    }
    logger.info("Zero-current residuals: A={:.3f} B={:.3f} A — sensing OK", resid_a, resid_b);
  }

  auto driver = bsp.motor_driver();
  driver->enable();
  driver->set_voltage(0, 0, 0);

  // Asymmetric drive: park the return phase at a small floor (~2% duty; never
  // 0 — a zero comparator wedges the MCPWM generator) and put the full
  // differential on the driven phase. Keeps common-mode duty minimal so the
  // parked phase's low side conducts almost always (valid feedback samples).
  constexpr float kFloorV = 1.0f;
  auto apply = [&](float u) {
    u = std::clamp(u, -(kVoltageLimit - kFloorV), kVoltageLimit - kFloorV);
    float mag = std::fabs(u);
    if (u >= 0)
      driver->set_voltage(kFloorV + mag, kFloorV, kFloorV + mag / 2.0f);
    else
      driver->set_voltage(kFloorV, kFloorV + mag, kFloorV + mag / 2.0f);
    return u;
  };

  auto timer_fn = [&]() -> bool {
    static int64_t t_prev = 0;
    static float integ = 0;
    static int over_count = 0;
    static bool was_enabled = false;

    bool en = g_enabled.load(std::memory_order_acquire);
    int64_t now = esp_timer_get_time();
    if (!en) {
      if (was_enabled) {
        driver->set_voltage(0, 0, 0);
        integ = 0;
        over_count = 0;
        g_u_disp.store(0);
        was_enabled = false;
      }
      t_prev = now;
      return false;
    }
    was_enabled = true;

    // Feedback from the parked phase (selected by target sign so it doesn't
    // chatter with the PI output). ib = -ia since C carries ~no current.
    float target = g_target.load();
    float i = target >= 0 ? -(read_amps_raw(ch_b) - csa_zero_b)
                          : (read_amps_raw(ch_a) - csa_zero_a);
    g_i_disp.store(g_i_disp.load() + 0.05f * (i - g_i_disp.load()));

    over_count = std::fabs(i) > kTripAmps ? over_count + 1 : 0;
    if (over_count >= kTripConsecutive) {
      driver->set_voltage(0, 0, 0);
      g_target.store(0);
      g_enabled.store(false);
      g_tripped.store(true);
      return false;
    }

    float dt = (float)(now - t_prev) * 1e-6f;
    t_prev = now;
    float e = target - i;
    integ = std::clamp(integ + g_ki.load() * e * dt, -kVoltageLimit, kVoltageLimit);
    float u = apply(g_kp.load() * e + integ);
    g_u_disp.store(u);
    return false;
  };

  auto timer = espp::HighResolutionTimer({.name      = "ctrl",
                                          .callback  = timer_fn,
                                          .log_level = espp::Logger::Verbosity::WARN});
  timer.periodic(kLoopPeriodUs);

  // 25 Hz CSV stream + 1 Hz DRV fault poll.
  fmt::print("%time(s), target_a, i_a, u_v, enabled\n");
  auto stream_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::steady_clock::now();
    static int tick = 0;

    if (g_tripped.exchange(false))
      fmt::print("! software overcurrent trip (>{:.0f} A) — disabled, target zeroed\n",
                 kTripAmps);
    if (++tick % 25 == 0) {
      std::error_code fec;
      auto fs = bsp.gate_driver()->fault_status(fec);
      if (!fec && fs.raw != 0) {
        g_enabled.store(false);
        g_target.store(0);
        fmt::print("! DRV8353 fault 0x{:04x} — disabled (power-cycle VM to clear)\n", fs.raw);
      }
    }
    float seconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
    fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:d}\n", seconds, g_target.load(),
               g_i_disp.load(), g_u_disp.load(), g_enabled.load() ? 1 : 0);
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 40ms);
    return false;
  };
  auto stream_task = espp::Task({.callback    = stream_fn,
                                 .task_config = {.name = "stream", .stack_size_bytes = 4 * 1024},
                                 .log_level   = espp::Logger::Verbosity::WARN});
  stream_task.start();

  setvbuf(stdin, nullptr, _IONBF, 0);
  fmt::print("#ready kp={:.6g} ki={:.6g}\n", g_kp.load(), g_ki.load());

  char line[128];
  size_t len = 0;
  while (true) {
    int ch = getchar();
    if (ch == EOF) {
      std::this_thread::sleep_for(50ms);
      continue;
    }
    if (ch == ' ' && len == 0) { // bare space toggles, like temp-sweep
      bool now_en = !g_enabled.load();
      if (!now_en) g_target.store(0);
      g_enabled.store(now_en);
      continue;
    }
    if (ch != '\n' && ch != '\r') {
      if (len < sizeof(line) - 1) line[len++] = (char)ch;
      continue;
    }
    if (len == 0) continue;
    line[len] = '\0';
    len = 0;
    float a = 0, b = 0;
    if (sscanf(line, "i %f", &a) == 1) {
      g_target.store(std::clamp(a, -kMaxTargetAmps, kMaxTargetAmps));
    } else if (sscanf(line, "g %f %f", &a, &b) == 2) {
      g_kp.store(a);
      g_ki.store(b);
      fmt::print("#gains kp={:.6g} ki={:.6g}\n", a, b);
    } else if (line[0] == 'o') {
      g_enabled.store(true);
    } else if (line[0] == 'f') {
      g_target.store(0);
      g_enabled.store(false);
    } else if (line[0] == 'r') {
      std::error_code rec;
      auto regs = bsp.gate_driver()->read_all_registers(rec);
      if (rec) {
        fmt::print("#err register read failed: {}\n", rec.message());
      } else {
        fmt::print("#regs fault1=0x{:04x} vgs2=0x{:04x} drv_ctl=0x{:04x} gate_hs=0x{:04x} "
                   "gate_ls=0x{:04x} ocp=0x{:04x} csa=0x{:04x} nfault_active={:d}\n",
                   regs.fault_status_1, regs.vgs_status_2, regs.driver_control,
                   regs.gate_drive_hs, regs.gate_drive_ls, regs.ocp_control, regs.csa_control,
                   bsp.gate_driver()->fault_pin_active() ? 1 : 0);
      }
    } else {
      fmt::print("#err unknown command\n");
    }
  }
}
