#include <atomic>
#include <chrono>
#include <thread>

#include "logger.hpp"
#include "task.hpp"

#include "pace-racer-board.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "PACE RACER", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("Bootup");

  // initialize the board support package, and initialize the motor subsystem
  using Bsp = espp::PaceRacerBoard;
  auto &bsp = Bsp::get();

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
  bsp.init_motor(motor_config);

  // get the motor objects (shared pointers) for use in the script
  auto motor = bsp.motor();

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

  static constexpr uint64_t core_update_period_us = 1'000;
  auto motor_timer = espp::HighResolutionTimer(
      {.name = "Motor Timer", .callback = motor_fn, .log_level = espp::Logger::Verbosity::WARN});
  motor_timer.periodic(core_update_period_us);

  // counter to show the number of prints, shared between main and task
  std::atomic<int> counter = 0;

  // make a simple task that prints "Hello World!" every second
  espp::Task task({.callback = [&](auto &m, auto &cv) -> bool {
                     logger.debug("[{}] Hello from the task!", counter++);
                     std::unique_lock<std::mutex> lock(m);
                     cv.wait_for(lock, 1s);
                     // we don't want to stop the task, so return false
                     return false;
                   },
                   .task_config = {
                       .name = "Hello World",
                       .stack_size_bytes = 4096,
                   }});
  task.start();

  // also print in the main thread
  while (true) {
    logger.debug("[{}] Hello World!", counter++);
    std::this_thread::sleep_for(1s);
  }
}
