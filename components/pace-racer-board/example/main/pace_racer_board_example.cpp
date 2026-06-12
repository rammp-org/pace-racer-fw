#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "butterworth_filter.hpp"
#include "high_resolution_timer.hpp"
#include "pace-racer-board.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "PACE RACER BSP Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");
  //! [pace-racer-board example]
  using Bsp = espp::PaceRacerBoard;
  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::INFO);

  // set the configuration for the motor For simplicity, we'll copy the defaults
  // and modify them, but you can also just make a type of
  // Bsp::BldcMotor::Config
  auto motor_config = bsp.default_motor_config;
  motor_config.phase_resistance = 4.0f; // ohms
  motor_config.current_limit = 1.0f;    // amps
  // velocity PID config:
  motor_config.velocity_pid_config.kp = 0.020f;
  motor_config.velocity_pid_config.ki = 0.700f;
  motor_config.velocity_pid_config.kd = 0.000f;
  // angle PID config:
  motor_config.angle_pid_config.kp = 5.000f;
  motor_config.angle_pid_config.ki = 1.000f;
  motor_config.angle_pid_config.kd = 0.000f;

  // now initialize the motor
  if (!bsp.init_motor(motor_config)) {
    logger.error("Failed to initialize motor");
    return;
  }

  // get the motor objects (shared pointers) for use in the script
  auto motor = bsp.motor();

  static constexpr uint64_t core_update_period_us = 1'000;                  // microseconds
  static constexpr float core_update_period = core_update_period_us / 1e6f; // seconds

  // static auto motion_control_type = espp::detail::MotionControlType::VELOCITY;
  static auto motion_control_type = espp::detail::MotionControlType::ANGLE;

  logger.info("Setting motion control type to {}", motion_control_type);
  motor->set_motion_control_type(motion_control_type);

  motor->enable();

  std::atomic<float> target = 60.0f;
  static bool target_is_angle =
      motion_control_type == espp::detail::MotionControlType::ANGLE ||
      motion_control_type == espp::detail::MotionControlType::ANGLE_OPENLOOP;
  // Function for initializing the target based on the motion control type
  auto initialize_target = [&]() {
    if (target_is_angle) {
      target = motor->get_shaft_angle();
    } else {
      target = 50.0f * espp::RPM_TO_RADS;
    }
  };
  // run it once
  initialize_target();

  auto motor_fn = [&]() -> bool {
    motor->loop_foc();
    motor->move(target);
    return false; // don't want to stop the task
  };

  auto motor_timer = espp::HighResolutionTimer(
      {.name = "Motor Timer", .callback = motor_fn, .log_level = espp::Logger::Verbosity::WARN});
  motor_timer.periodic(core_update_period_us);

  static constexpr float sample_freq_hz = 100.0f;
  static constexpr float filter_cutoff_freq_hz = 5.0f;
  static constexpr float normalized_cutoff_frequency =
      2.0f * filter_cutoff_freq_hz / sample_freq_hz;
  static constexpr size_t ORDER = 2;
  // NOTE: using the Df2 since it's hardware accelerated :)
  using Filter = espp::ButterworthFilter<ORDER, espp::BiquadFilterDf2>;
  Filter filter({.normalized_cutoff_frequency = normalized_cutoff_frequency});

  // if it's a velocity setpoint then target is RPM
  fmt::print("%time(s), "
             "motor target, " // target is either RPM or radians
             "motor angle (radians), "
             "motor speed (rpm)\n");

  // make the task to periodically poll the encoders and print the state. NOTE:
  // the encoders run their own tasks to maintain state, so we're just polling
  // the current state.
  auto logging_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    auto _target = target.load();
    if (!target_is_angle)
      _target *= espp::RADS_TO_RPM;
    auto rpm = filter(motor->get_shaft_velocity() * espp::RADS_TO_RPM);
    auto rads = motor->get_shaft_angle();
    fmt::print("{:.3f}, {:.3f}, {:.3f}, {:.3f}\n", seconds, _target, rads, rpm);
    // NOTE: sleeping in this way allows the sleep to exit early when the
    // task is being stopped / destroyed
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 10ms);
    }
    // don't want to stop the task
    return false;
  };
  auto logging_task = espp::Task({.callback = logging_fn,
                                  .task_config =
                                      {
                                          .name = "Logging Task",
                                          .stack_size_bytes = 5 * 1024,
                                      },
                                  .log_level = espp::Logger::Verbosity::WARN});
  logging_task.start();

  std::this_thread::sleep_for(1s);
  logger.info("Starting target task");

  enum class IncrementDirection { DOWN = -1, HOLD = 0, UP = 1 };
  static IncrementDirection increment_direction = IncrementDirection::UP;

  auto update_target = [&](auto &target, auto &increment_direction) {
    float max_target = target_is_angle ? (2.0f * M_PI) : (200.0f * espp::RPM_TO_RADS);
    float target_delta =
        target_is_angle ? (M_PI / 4.0f) : (50.0f * espp::RPM_TO_RADS * core_update_period);
    // update target
    if (increment_direction == IncrementDirection::UP) {
      target += target_delta;
      if (target >= max_target) {
        increment_direction = IncrementDirection::DOWN;
      }
    } else if (increment_direction == IncrementDirection::DOWN) {
      target -= target_delta;
      if (target <= -max_target) {
        increment_direction = IncrementDirection::UP;
      }
    }
  };

  // make a task which will update the target (velocity or angle)
  auto target_task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    auto delay = std::chrono::duration<float>(target_is_angle ? 1.0f : core_update_period);
    auto start = std::chrono::high_resolution_clock::now();
    update_target(target, increment_direction);
    // NOTE: sleeping in this way allows the sleep to exit early when the
    // task is being stopped / destroyed
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, start + delay);
    }
    // don't want to stop the task
    return false;
  };
  auto target_task = espp::Task({
      .callback = target_task_fn,
      .task_config = {.name = "Target Task"},
  });
  target_task.start();

  // now loop forever (do nothing)
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [pace-racer-board example]
}
