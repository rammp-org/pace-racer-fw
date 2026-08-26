#include <chrono>
#include <numbers>
#include <thread>

#include "bldc_motor.hpp"
#include "high_resolution_timer.hpp"
#include "logger.hpp"
#include "pace-racer-board.hpp"
#include "task.hpp"

#include "hall_sensor.hpp"

using namespace std::chrono_literals;
using HallMotor = espp::BldcMotor<espp::BldcDriver, HallSensor>;

static constexpr float kPowerSupplyVoltage = 48.0f;
static constexpr float kVoltageLimit       =  5.0f;
static constexpr float kTargetVelocityRpm  = 30.0f; // open loop: needs enough to overcome stiction
static constexpr int   kPolePairs          = 15;    // measured: 90 hall transitions per rev

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "hall-foc", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup — hall FOC, {:.1f} V limit, {:.0f} RPM target", kVoltageLimit, kTargetVelocityRpm);

  using Bsp = espp::PaceRacerBoard;
  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::WARN);

  // Init DRV8353 + BldcDriver via BSP. We ignore bsp.motor() — it uses the encoder.
  auto bsp_cfg       = bsp.default_motor_config;
  bsp_cfg.num_pole_pairs = kPolePairs;
  if (!bsp.init_motor(bsp_cfg, {.power_supply_voltage = kPowerSupplyVoltage,
                                 .limit_voltage        = kVoltageLimit})) {
    logger.error("BSP motor init failed");
    return;
  }

  auto hall = std::make_shared<HallSensor>(HallSensor::Config{
      .pin_a       = GPIO_NUM_3,
      .pin_b       = GPIO_NUM_46,
      .pin_c       = GPIO_NUM_9,
      .pole_pairs  = kPolePairs,
  });
  hall->init();

  // Validate initial hall state before enabling anything
  std::error_code ec;
  hall->update(ec);
  if (hall->get_radians() == 0.0f && hall->get_mechanical_radians() == 0.0f) {
    logger.warn("Initial hall state may be invalid (000/111) — check wiring before proceeding");
  }

  auto motor = std::make_shared<HallMotor>(HallMotor::Config{
      .num_pole_pairs    = kPolePairs,
      .phase_resistance  = 1.0f,
      .kv_rating         = 320,
      .current_limit     = 10.0f,
      .sensor_direction  = espp::detail::SensorDirection::CLOCKWISE,
      .foc_type          = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver            = bsp.motor_driver(),
      .sensor            = hall,
      .run_sensor_update = true,
      .velocity_pid_config = {
          .kp             = 0.015f,
          .ki             = 0.075f,
          .kd             = 0.000f,
          .integrator_min = -1.0f,
          .integrator_max =  1.0f,
          .output_min     = -1.0f,
          .output_max     =  1.0f,
      },
      .angle_pid_config = {}, // unused in velocity mode
      .velocity_filter  = [](float v) { return v; },
      .angle_filter     = [](float a) { return a; },
  });

  {
    std::error_code drv_ec;
    // ESP32 reboot does NOT power-cycle the DRV8353 — clear any latched faults from previous boots.
    bsp.gate_driver()->clear_faults(drv_ec);
    if (drv_ec) logger.error("Failed to clear DRV8353 faults: {}", drv_ec.message());
  }

  motor->set_motion_control_type(espp::detail::MotionControlType::VELOCITY_OPENLOOP);
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

  auto diag_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::steady_clock::now();
    float seconds     = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
    auto &b = espp::PaceRacerBoard::get();
    float vref_mv = b.motor_current_sense_vref();
    fmt::print("{:.3f}, {:.2f}, {:.4f}, {:.2f}, {:.1f}, {:.3f}, {:.3f}, {:.3f}\n",
               seconds,
               hall->get_rpm(),
               motor->get_shaft_angle(),
               motor->get_shaft_velocity() * espp::RADS_TO_RPM,
               vref_mv,
               b.motor_current_a_amps(),
               b.motor_current_b_amps(),
               b.motor_current_c_amps());
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
