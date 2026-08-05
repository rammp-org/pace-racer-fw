#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
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
static constexpr float kVoltageLimit       =  1.0f; // start low — raise once closed loop is stable
static constexpr int   kPolePairs          = 15;    // measured: 90 hall transitions per rev

// Closed-loop (VELOCITY) speed sweep: hold each target this long, then advance.
// ponytail: gains below are the hall-foc values tuned near 600 RPM; these low
// targets may track sloppily. Bump the sweep RPMs or retune if it won't hold.
static constexpr float kSweepRpm[]   = {200.0f, 200.0f, 500.0f, 800.0f, 500.0f};
static constexpr auto  kHoldPerStep  = 30s;

// Shared with the 1 kHz motor timer and the stdin command task.
static std::atomic<float> g_target_rads{0.0f};
static std::atomic<bool>  g_enabled{true};

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "temp-sweep", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup — closed-loop speed sweep + temperature CSV");

  using Bsp = espp::PaceRacerBoard;
  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::WARN);

  std::error_code ec;
  if (!bsp.init_temperature_sensors(ec)) {
    logger.error("Temperature sensor init failed: {}", ec.message());
    // keep going — CSV emits NaN for unavailable sensors
  }

  auto bsp_cfg           = bsp.default_motor_config;
  bsp_cfg.num_pole_pairs = kPolePairs;
  if (!bsp.init_motor(bsp_cfg, {.power_supply_voltage = kPowerSupplyVoltage,
                                 .limit_voltage        = kVoltageLimit})) {
    logger.error("BSP motor init failed");
    return;
  }

  {
    std::error_code drv_ec;
    // ESP32 reboot does NOT power-cycle the DRV8353 — clear any latched faults.
    bsp.gate_driver()->clear_faults(drv_ec);
    if (drv_ec) logger.error("Failed to clear DRV8353 faults: {}", drv_ec.message());
  }

  // Measure CSA amplifier DC offset using DRV8353 CAL mode (shorts differential
  // inputs to zero). Subtracted from every current reading below so closed-loop
  // current sensing starts from true zero.
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
      .pin_a      = GPIO_NUM_3,
      .pin_b      = GPIO_NUM_46,
      .pin_c      = GPIO_NUM_9,
      .pole_pairs = kPolePairs,
  });
  hall->init();

  hall->update(ec);
  if (hall->get_radians() == 0.0f && hall->get_mechanical_radians() == 0.0f) {
    logger.warn("Initial hall state may be invalid (000/111) — check wiring before proceeding");
  }

  auto motor = std::make_shared<HallMotor>(HallMotor::Config{
      .num_pole_pairs    = kPolePairs,
      .phase_resistance  = 1.0f,
      .kv_rating         = 500,
      .current_limit     = 5.0f,
      .zero_electric_offset = 1e-6f, // skip align_sensor cal; steps_ seeded from boot sector handles absolute position
      .sensor_direction  = espp::detail::SensorDirection::CLOCKWISE,
      .foc_type          = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver            = bsp.motor_driver(),
      .sensor            = hall,
      .run_sensor_update = true,
      .velocity_pid_config = {
          .kp             = 0.01f,
          .ki             = 0.005f,
          .kd             = 0.000f,
          .integrator_min = -10.0f,
          .integrator_max =  10.0f,
          .output_min     = -10.0f,
          .output_max     =  10.0f,
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

  auto motor_timer = espp::HighResolutionTimer(
      {.name     = "Motor Timer",
       .callback = [&]() -> bool {
         // ponytail: unsynchronized enable/disable vs this callback; worst case one
         // stale move() across a toggle at 1 kHz. Add a mutex only if that matters.
         if (g_enabled.load()) {
           motor->loop_foc();
           motor->move(g_target_rads.load());
         }
         return false;
       },
       .log_level = espp::Logger::Verbosity::WARN});
  motor_timer.periodic(1'000); // 1 kHz

  // stdin command task: 'o'/'1' enable, 'f'/'0' disable, ' ' toggle.
  // Blocking getchar in its own task — no polling. Needs the USB-Serial-JTAG
  // console (see sdkconfig.defaults) for the host to reach stdin.
  setvbuf(stdin, nullptr, _IONBF, 0);
  std::thread([&] {
    while (true) {
      int c = getchar();
      if (c == EOF) { std::this_thread::sleep_for(50ms); continue; }
      bool now;
      switch (c) {
        case 'o': case '1': now = true;  break;
        case 'f': case '0': now = false; break;
        case ' ':           now = !g_enabled.load(); break;
        default: continue;
      }
      if (now) motor->enable(); else motor->disable();
      g_enabled.store(now);
    }
  }).detach();

  fmt::print("%time(s), target_rpm, hall_rpm, enabled, ia_a, ib_a, ic_a, "
             "temp0_c, temp1_c, temp2_c, temp3_c\n");

  auto start      = std::chrono::steady_clock::now();
  auto step_start = start;
  size_t step     = 0;
  g_target_rads.store(kSweepRpm[0] * espp::RPM_TO_RADS);

  auto diag_fn = [&, csa_zero_a, csa_zero_b, csa_zero_c](std::mutex &m, std::condition_variable &cv) {
    auto now = std::chrono::steady_clock::now();
    if (now - step_start >= kHoldPerStep) {
      step       = (step + 1) % (sizeof(kSweepRpm) / sizeof(kSweepRpm[0]));
      step_start = now;
      g_target_rads.store(kSweepRpm[step] * espp::RPM_TO_RADS);
    }
    float seconds = std::chrono::duration<float>(now - start).count();

    Bsp::TemperatureErrors terrs;
    auto temps = bsp.board_temperatures_c(terrs);

    fmt::print("{:.3f}, {:.1f}, {:.2f}, {:d}, {:.3f}, {:.3f}, {:.3f}, "
               "{:.2f}, {:.2f}, {:.2f}, {:.2f}\n",
               seconds, kSweepRpm[step], hall->get_rpm(), g_enabled.load() ? 1 : 0,
               bsp.motor_current_a_amps() - csa_zero_a,
               bsp.motor_current_b_amps() - csa_zero_b,
               bsp.motor_current_c_amps() - csa_zero_c,
               terrs[0] ? std::nanf("") : temps[0],
               terrs[1] ? std::nanf("") : temps[1],
               terrs[2] ? std::nanf("") : temps[2],
               terrs[3] ? std::nanf("") : temps[3]);
    std::unique_lock<std::mutex> lk(m);
    cv.wait_for(lk, 250ms); // temps drift slowly; 4 Hz is plenty
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
