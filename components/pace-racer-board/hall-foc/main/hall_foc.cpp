#include <chrono>
#include <numbers>
#include <thread>

#include "bldc_motor.hpp"
#include "high_resolution_timer.hpp"
#include "logger.hpp"
#include "pace-racer-board.hpp"
#include "task.hpp"

#include "hall_sensor.hpp"
#include "simple_lowpass_filter.hpp"

using namespace std::chrono_literals;
using HallMotor = espp::BldcMotor<espp::BldcDriver, HallSensor>;

static constexpr float kPowerSupplyVoltage = 48.0f;
static constexpr float kVoltageLimit       =  5.0f; // start low — increase once closed loop is stable
static constexpr float kTargetVelocityRpm  = 600.0f;
static constexpr int   kPolePairs          = 15;    // measured: 90 hall transitions per rev

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "hall-foc", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup — hall FOC closed loop, {:.1f} V limit, {:.0f} RPM target",
              kVoltageLimit, kTargetVelocityRpm);

  using Bsp = espp::PaceRacerBoard;
  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::WARN);

  auto bsp_cfg = bsp.default_motor_config;
  bsp_cfg.num_pole_pairs = kPolePairs;
  if (!bsp.init_motor(bsp_cfg, {.power_supply_voltage = kPowerSupplyVoltage,
                                 .limit_voltage        = kVoltageLimit})) {
    logger.error("BSP motor init failed");
    return;
  }

  {
    std::error_code ec;
    // ESP32 reboot does NOT power-cycle the DRV8353 — clear any latched faults from previous boots.
    bsp.gate_driver()->clear_faults(ec);
    if (ec) logger.error("Failed to clear DRV8353 faults: {}", ec.message());
  }

  // Measure CSA amplifier DC offset using DRV8353 CAL mode (shorts differential inputs to zero).
  // At GAIN_5 the amplifier offset is small but non-zero; at any gain the ADC common-mode
  // bias pushes all three channels to a consistent non-zero reading — subtract it out.
  float csa_zero_a = 0.0f, csa_zero_b = 0.0f, csa_zero_c = 0.0f;
  {
    std::error_code cal_ec;
    bsp.gate_driver()->set_csa_calibration(true, true, true, cal_ec);
    if (cal_ec) {
      logger.warn("CSA CAL mode failed ({}), zero offsets will be 0", cal_ec.message());
    } else {
      std::this_thread::sleep_for(10ms);
      constexpr int kCalN = 32;
      for (int i = 0; i < kCalN; i++) {
        csa_zero_a += bsp.motor_current_a_amps();
        csa_zero_b += bsp.motor_current_b_amps();
        csa_zero_c += bsp.motor_current_c_amps();
        std::this_thread::sleep_for(1ms);
      }
      bsp.gate_driver()->set_csa_calibration(false, false, false, cal_ec);
      bsp.gate_driver()->clear_faults(cal_ec); // write_protected_register can latch faults
      csa_zero_a /= kCalN;
      csa_zero_b /= kCalN;
      csa_zero_c /= kCalN;
      logger.info("CSA zero offsets: A={:.3f}A B={:.3f}A C={:.3f}A", csa_zero_a, csa_zero_b, csa_zero_c);
    }
  }

  auto hall = std::make_shared<HallSensor>(HallSensor::Config{
      .pin_a       = GPIO_NUM_3,
      .pin_b       = GPIO_NUM_46,
      .pin_c       = GPIO_NUM_9,
      .pole_pairs  = kPolePairs,
  });
  hall->init();

  std::error_code ec;
  hall->update(ec);
  if (hall->get_radians() == 0.0f && hall->get_mechanical_radians() == 0.0f) {
    logger.warn("Initial hall state may be invalid (000/111) — check wiring before proceeding");
  }

  auto motor = std::make_shared<HallMotor>(HallMotor::Config{
      .num_pole_pairs    = kPolePairs,
      .phase_resistance  = 1.0f,
      .kv_rating         = 500,
      .current_limit     = 10.0f,
      .zero_electric_offset = 1e-6f, // skip align_sensor cal; steps_ seeded from boot sector handles absolute position
      .sensor_direction  = espp::detail::SensorDirection::CLOCKWISE,
      .foc_type          = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver            = bsp.motor_driver(),
      .sensor            = hall,
      .run_sensor_update = true,
      .velocity_pid_config = {
          .kp             = 0.05f,
          .ki             = 0.05f,
          .kd             = 0.000f,
          .integrator_min = -10.0f,
          .integrator_max =  10.0f,
          .output_min     = -5.0f,
          .output_max     =  5.0f,
      },
      .angle_pid_config = {}, // unused in velocity mode
      .velocity_filter  = [lpf = espp::SimpleLowpassFilter({.time_constant = 0.05f})](float v) mutable {
                            return lpf(v);
                          },
      .angle_filter     = [](float a) { return a; },
  });

  motor->set_motion_control_type(espp::detail::MotionControlType::VELOCITY);
  motor->initialize();
  motor->enable();

  const float target_rads = kTargetVelocityRpm * espp::RPM_TO_RADS;

  auto motor_timer = espp::HighResolutionTimer(
      {.name     = "Motor Timer",
       .callback = [&]() -> bool {
         motor->loop_foc();
         motor->move(target_rads);
         return false;
       },
       .log_level = espp::Logger::Verbosity::WARN});
  motor_timer.periodic(1'000); // 1 kHz

  fmt::print("%time(s), hall_rpm, shaft_angle(rad), shaft_velocity(rpm), vref_mv, ia_a, ib_a, ic_a\n");

  auto diag_fn = [&, csa_zero_a, csa_zero_b, csa_zero_c](std::mutex &m, std::condition_variable &cv) {
    static auto     start        = std::chrono::steady_clock::now();
    static uint32_t last_glitches = 0;

    float seconds = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
    auto &b = espp::PaceRacerBoard::get();

    // Emit a ! alert line whenever a new hall glitch is detected.
    // These lines are ignored by the CSV parser but visible in the serial monitor.
    auto gi = hall->glitch_info();
    if (gi.count != last_glitches) {
      float shaft_rpm = motor->get_shaft_velocity() * espp::RADS_TO_RPM;
      fmt::print("! t={:.3f} hall_glitch: delta={} state=0b{:03b} total={} shaft_rpm={:.1f}\n",
                 seconds, gi.last_delta, gi.last_state, gi.count, shaft_rpm);
      last_glitches = gi.count;
    }

    fmt::print("{:.3f}, {:.2f}, {:.4f}, {:.2f}, {:.1f}, {:.3f}, {:.3f}, {:.3f}\n",
               seconds,
               hall->get_rpm(),
               motor->get_shaft_angle(),
               motor->get_shaft_velocity() * espp::RADS_TO_RPM,
               b.motor_current_sense_vref(),
               b.motor_current_a_amps() - csa_zero_a,
               b.motor_current_b_amps() - csa_zero_b,
               b.motor_current_c_amps() - csa_zero_c);
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 50ms);
    return false;
  };

  auto diag_task = espp::Task({.callback    = diag_fn,
                               .task_config = {.name             = "Diagnostics",
                                               .stack_size_bytes = 4 * 1024},
                               .log_level   = espp::Logger::Verbosity::WARN});
  diag_task.start();

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
