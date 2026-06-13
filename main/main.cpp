#include <array>
#include <atomic>
#include <chrono>
#include <limits>
#include <thread>

#include "butterworth_filter.hpp"
#include "high_resolution_timer.hpp"
#include "logger.hpp"
#include "pace-racer-board.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "PACE RACER", .level = espp::Logger::Verbosity::INFO});

  logger.info("Bootup");

  using Bsp = espp::PaceRacerBoard;
  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::INFO);

  std::error_code ec;
  if (!bsp.init_temperature_sensors(ec)) {
    logger.error("Failed to initialize board temperature sensors: {}", ec.message());
    return;
  }

  // set the configuration for the motor. For simplicity, copy the defaults and
  // override the values we want to exercise in the board-level test app.
  auto motor_config = bsp.default_motor_config;
  motor_config.phase_resistance = 4.0f; // ohms
  motor_config.current_limit = 1.0f;    // amps
  motor_config.velocity_pid_config.kp = 0.020f;
  motor_config.velocity_pid_config.ki = 0.700f;
  motor_config.velocity_pid_config.kd = 0.000f;
  motor_config.angle_pid_config.kp = 5.000f;
  motor_config.angle_pid_config.ki = 1.000f;
  motor_config.angle_pid_config.kd = 0.000f;

  if (!bsp.init_motor(motor_config)) {
    logger.error("Failed to initialize motor");
    return;
  }

  auto motor = bsp.motor();
  if (!motor) {
    logger.error("Motor not available after initialization");
    return;
  }

  auto gate_driver = bsp.gate_driver();
  if (!gate_driver) {
    logger.error("Gate driver not available after motor initialization");
    return;
  }

  auto fault_status = gate_driver->fault_status(ec);
  if (ec) {
    logger.error("Failed to read DRV8353 fault status: {}", ec.message());
    return;
  }
  logger.info("Initial DRV8353 fault status: 0x{:03X}", fault_status.raw);

  static constexpr uint64_t core_update_period_us = 1'000;
  static constexpr float kHomeOffsetRadians = 0.0f;
  static constexpr float kQuarterTurnRadians = 1.57079633f;
  static constexpr float kVelocityTargetRpm = 150.0f;

  motor->enable();

  std::atomic<float> target = motor->get_shaft_angle();
  std::atomic<bool> target_is_angle = true;
  const float home_angle = motor->get_shaft_angle();

  auto motor_fn = [&]() -> bool {
    motor->loop_foc();
    motor->move(target.load());
    return false;
  };

  auto motor_timer = espp::HighResolutionTimer(
      {.name = "Motor Timer", .callback = motor_fn, .log_level = espp::Logger::Verbosity::WARN});
  motor_timer.periodic(core_update_period_us);

  struct ControlPhase {
    espp::detail::MotionControlType control_type;
    float command;
    std::chrono::milliseconds duration;
    const char *name;
    bool relative_to_home;
  };

  const std::array<ControlPhase, 6> phases{{
      {.control_type = espp::detail::MotionControlType::VELOCITY,
       .command = kVelocityTargetRpm * espp::RPM_TO_RADS,
       .duration = 4s,
       .name = "velocity-forward",
       .relative_to_home = false},
      {.control_type = espp::detail::MotionControlType::VELOCITY,
       .command = -kVelocityTargetRpm * espp::RPM_TO_RADS,
       .duration = 4s,
       .name = "velocity-reverse",
       .relative_to_home = false},
      {.control_type = espp::detail::MotionControlType::VELOCITY,
       .command = 0.0f,
       .duration = 2s,
       .name = "velocity-hold",
       .relative_to_home = false},
      {.control_type = espp::detail::MotionControlType::ANGLE,
       .command = kQuarterTurnRadians,
       .duration = 3s,
       .name = "position-positive",
       .relative_to_home = true},
      {.control_type = espp::detail::MotionControlType::ANGLE,
       .command = -kQuarterTurnRadians,
       .duration = 3s,
       .name = "position-negative",
       .relative_to_home = true},
      {.control_type = espp::detail::MotionControlType::ANGLE,
       .command = kHomeOffsetRadians,
       .duration = 3s,
       .name = "position-home",
       .relative_to_home = true},
  }};

  std::atomic<size_t> active_phase_index = 0;

  auto apply_phase = [&](size_t index) {
    const auto &phase = phases[index];
    const bool angle_mode = phase.control_type == espp::detail::MotionControlType::ANGLE ||
                            phase.control_type == espp::detail::MotionControlType::ANGLE_OPENLOOP;
    target_is_angle = angle_mode;
    motor->set_motion_control_type(phase.control_type);
    target = phase.relative_to_home ? (home_angle + phase.command) : phase.command;

    if (angle_mode) {
      logger.info("Starting {} test: target angle {:.3f} rad", phase.name, target.load());
    } else {
      logger.info("Starting {} test: target velocity {:.1f} rpm", phase.name,
                  phase.command * espp::RADS_TO_RPM);
    }
  };

  apply_phase(active_phase_index.load());

  static constexpr float sample_freq_hz = 20.0f;
  static constexpr float filter_cutoff_freq_hz = 2.0f;
  static constexpr float normalized_cutoff_frequency =
      2.0f * filter_cutoff_freq_hz / sample_freq_hz;
  using VelocityFilter = espp::ButterworthFilter<2, espp::BiquadFilterDf2>;
  VelocityFilter velocity_filter({.normalized_cutoff_frequency = normalized_cutoff_frequency});

  fmt::print("%time(s), control_mode, target_angle(rad), target_velocity(rpm), "
             "motor_angle(rad), motor_speed(rpm)\n");

  auto diagnostics_task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::steady_clock::now();
    static auto next_diagnostics = start;

    auto now = std::chrono::steady_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    const bool angle_mode = target_is_angle.load();
    const float target_value = target.load();
    const float target_angle = angle_mode ? target_value : std::numeric_limits<float>::quiet_NaN();
    const float target_velocity_rpm =
        angle_mode ? std::numeric_limits<float>::quiet_NaN() : (target_value * espp::RADS_TO_RPM);
    const float measured_angle = motor->get_shaft_angle();
    const float measured_velocity_rpm =
        velocity_filter(motor->get_shaft_velocity() * espp::RADS_TO_RPM);

    fmt::print("{:.3f}, {}, {:.3f}, {:.3f}, {:.3f}, {:.3f}\n", seconds,
               angle_mode ? "position" : "velocity", target_angle, target_velocity_rpm,
               measured_angle, measured_velocity_rpm);

    if (now >= next_diagnostics) {
      Bsp::TemperatureErrors temperature_errors;
      auto temperatures_c = bsp.board_temperatures_c(temperature_errors);
      bool temperatures_ok = true;
      for (size_t i = 0; i < temperatures_c.size(); i++) {
        if (temperature_errors[i]) {
          logger.error("Failed to read board temperature sensor {}: {}", i,
                       temperature_errors[i].message());
          temperatures_ok = false;
          break;
        }
      }

      fault_status = gate_driver->fault_status(ec);
      if (ec) {
        logger.error("Failed to read DRV8353 fault status: {}", ec.message());
      } else if (temperatures_ok) {
        logger.info("Diagnostics: fault=0x{:03X}, board_temps_c=[{:.3f}, {:.3f}, {:.3f}, {:.3f}]",
                    fault_status.raw, temperatures_c[0], temperatures_c[1], temperatures_c[2],
                    temperatures_c[3]);
      }
      next_diagnostics = now + 1s;
    }

    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 50ms);
    }
    return false;
  };

  auto diagnostics_task = espp::Task({.callback = diagnostics_task_fn,
                                      .task_config =
                                          {
                                              .name = "Diagnostics Task",
                                              .stack_size_bytes = 5 * 1024,
                                          },
                                      .log_level = espp::Logger::Verbosity::WARN});
  diagnostics_task.start();

  auto phase_task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    const auto phase_index = active_phase_index.load();
    const auto delay = phases[phase_index].duration;
    const auto deadline = std::chrono::steady_clock::now() + delay;
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, deadline);
    }

    const size_t next_phase = (phase_index + 1) % phases.size();
    active_phase_index = next_phase;
    apply_phase(next_phase);
    return false;
  };

  auto phase_task = espp::Task({
      .callback = phase_task_fn,
      .task_config =
          {
              .name = "Control Phase Task",
              .stack_size_bytes = 4 * 1024,
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });
  phase_task.start();

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
