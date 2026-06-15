#include <chrono>
#include <cmath>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "format.hpp"
#include "pace-racer-board.hpp"

using namespace std::chrono_literals;

namespace {
enum class CheckStatus {
  PASS,
  WARN,
  FAIL,
  SKIP,
};

struct CheckResult {
  std::string subsystem;
  std::string check;
  CheckStatus status;
  std::string details;
};

const char *to_string(CheckStatus status) {
  switch (status) {
  case CheckStatus::PASS:
    return "PASS";
  case CheckStatus::WARN:
    return "WARN";
  case CheckStatus::FAIL:
    return "FAIL";
  case CheckStatus::SKIP:
    return "SKIP";
  }
  return "UNKNOWN";
}

const char *
to_string(espp::Mt6701<espp::Mt6701Interface::SSI>::MagneticFieldStrength field_strength) {
  using Encoder = espp::Mt6701<espp::Mt6701Interface::SSI>;
  switch (field_strength) {
  case Encoder::MagneticFieldStrength::NORMAL:
    return "NORMAL";
  case Encoder::MagneticFieldStrength::TOO_STRONG:
    return "TOO_STRONG";
  case Encoder::MagneticFieldStrength::TOO_WEAK:
    return "TOO_WEAK";
  }
  return "UNKNOWN";
}

const char *to_string(espp::Mt6701<espp::Mt6701Interface::SSI>::TrackingStatus tracking_status) {
  using Encoder = espp::Mt6701<espp::Mt6701Interface::SSI>;
  switch (tracking_status) {
  case Encoder::TrackingStatus::NORMAL:
    return "NORMAL";
  case Encoder::TrackingStatus::LOST:
    return "LOST";
  }
  return "UNKNOWN";
}

std::string join_strings(const std::vector<std::string> &items, std::string_view separator) {
  std::string joined;
  for (size_t i = 0; i < items.size(); i++) {
    if (i > 0) {
      joined += separator;
    }
    joined += items[i];
  }
  return joined;
}

std::string csv_quote(std::string_view value) {
  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (char c : value) {
    if (c == '"') {
      escaped.push_back('"');
    }
    escaped.push_back(c);
  }
  escaped.push_back('"');
  return escaped;
}
} // namespace

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "PACE RACER Bringup", .level = espp::Logger::Verbosity::INFO});
  using Bsp = espp::PaceRacerBoard;

  std::vector<CheckResult> results;
  auto record = [&](std::string_view subsystem, std::string_view check, CheckStatus status,
                    std::string details) {
    results.push_back({.subsystem = std::string(subsystem),
                       .check = std::string(check),
                       .status = status,
                       .details = std::move(details)});
    const auto &result = results.back();
    switch (result.status) {
    case CheckStatus::PASS:
      logger.info("{} / {}: {}", result.subsystem, result.check, result.details);
      break;
    case CheckStatus::WARN:
      logger.warn("{} / {}: {}", result.subsystem, result.check, result.details);
      break;
    case CheckStatus::FAIL:
      logger.error("{} / {}: {}", result.subsystem, result.check, result.details);
      break;
    case CheckStatus::SKIP:
      logger.warn("{} / {}: {}", result.subsystem, result.check, result.details);
      break;
    }
  };

  logger.info("Starting PACE RACER board bringup");

  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::INFO);
  record("board", "singleton", CheckStatus::PASS, "Acquired board singleton");

  bsp.stop_breathing();
  bsp.set_blue_led_duty(15.0f);
  bsp.set_green_led_duty(0.0f);
  std::this_thread::sleep_for(250ms);
  bsp.set_blue_led_duty(0.0f);
  bsp.set_green_led_duty(15.0f);
  std::this_thread::sleep_for(250ms);
  bsp.set_green_led_duty(0.0f);
  record("leds", "manual-control", CheckStatus::PASS,
         "Issued blue, green, and off commands; visual confirmation recommended");

  const float current_sense_vref_mv = bsp.motor_current_sense_vref();
  const float phase_a_raw = bsp.motor_current_a_amps();
  const float phase_b_raw = bsp.motor_current_b_amps();
  const float phase_c_raw = bsp.motor_current_c_amps();
  const bool adc_values_finite = std::isfinite(current_sense_vref_mv) &&
                                 std::isfinite(phase_a_raw) && std::isfinite(phase_b_raw) &&
                                 std::isfinite(phase_c_raw);
  const bool adc_values_in_range =
      current_sense_vref_mv >= 0.0f && current_sense_vref_mv <= 3300.0f && phase_a_raw >= 0.0f &&
      phase_a_raw <= 3300.0f && phase_b_raw >= 0.0f && phase_b_raw <= 3300.0f &&
      phase_c_raw >= 0.0f && phase_c_raw <= 3300.0f;
  if (!adc_values_finite || !adc_values_in_range) {
    record("adc", "current-sense-baseline", CheckStatus::FAIL,
           fmt::format("Unexpected current-sense readings: vref_mv={:.3f}, phase_a_raw={:.3f}, "
                       "phase_b_raw={:.3f}, phase_c_raw={:.3f}",
                       current_sense_vref_mv, phase_a_raw, phase_b_raw, phase_c_raw));
  } else {
    record(
        "adc", "current-sense-baseline", CheckStatus::PASS,
        fmt::format("vref_mv={:.3f}, phase_a_raw={:.3f}, phase_b_raw={:.3f}, phase_c_raw={:.3f}; "
                    "phase values use placeholder CURRENT_SENSE_MV_TO_A scaling",
                    current_sense_vref_mv, phase_a_raw, phase_b_raw, phase_c_raw));
  }

  std::error_code ec;
  if (!bsp.init_temperature_sensors(ec)) {
    record("i2c", "temperature-sensors", CheckStatus::FAIL,
           fmt::format("Failed to initialize LM75 sensors: {}", ec.message()));
  } else if (!bsp.temperature_sensors_initialized()) {
    record("i2c", "temperature-sensors", CheckStatus::FAIL,
           "LM75 initialization returned success but sensors were not fully registered");
  } else {
    Bsp::TemperatureErrors temperature_errors;
    auto temperatures_c = bsp.board_temperatures_c(temperature_errors);
    std::vector<std::string> temperature_notes;
    bool temperature_read_failed = false;
    for (size_t i = 0; i < temperatures_c.size(); i++) {
      if (temperature_errors[i]) {
        temperature_read_failed = true;
        temperature_notes.push_back(fmt::format("0x{:02X}=ERROR({})",
                                                Bsp::TEMPERATURE_SENSOR_ADDRESSES[i],
                                                temperature_errors[i].message()));
      } else {
        temperature_notes.push_back(fmt::format(
            "0x{:02X}={:.3f}C", Bsp::TEMPERATURE_SENSOR_ADDRESSES[i], temperatures_c[i]));
      }
    }

    record("i2c", "temperature-sensors",
           temperature_read_failed ? CheckStatus::FAIL : CheckStatus::PASS,
           join_strings(temperature_notes, ", "));
  }

  bool motor_subsystem_ready = false;
  auto motor_config = bsp.default_motor_config;
  if (!bsp.init_motor(motor_config)) {
    record("motor", "subsystem-init", CheckStatus::FAIL,
           "init_motor(...) failed; encoder, gate driver, or motor driver did not initialize");
  } else {
    auto encoder = bsp.encoder();
    auto gate_driver = bsp.gate_driver();
    auto motor_driver = bsp.motor_driver();
    auto motor = bsp.motor();
    motor_subsystem_ready = static_cast<bool>(encoder) && static_cast<bool>(gate_driver) &&
                            static_cast<bool>(motor_driver) && static_cast<bool>(motor);
    record("motor", "subsystem-init", motor_subsystem_ready ? CheckStatus::PASS : CheckStatus::FAIL,
           motor_subsystem_ready ? "Encoder, DRV8353, motor driver, and motor objects initialized; "
                                   "DRV8353 enabled and PWM configured at 0% duty"
                                 : "init_motor(...) returned success but one or more motor "
                                   "subsystem objects are missing");
  }

  if (!motor_subsystem_ready) {
    record("drv8353", "status-registers", CheckStatus::SKIP,
           "Skipped because motor subsystem initialization failed");
    record("encoder", "status", CheckStatus::SKIP,
           "Skipped because motor subsystem initialization failed");
  } else {
    auto gate_driver = bsp.gate_driver();
    auto encoder = bsp.encoder();

    auto fault_status = gate_driver->fault_status(ec);
    if (ec) {
      record("drv8353", "status-registers", CheckStatus::FAIL,
             fmt::format("Failed to read FAULT_STATUS_1: {}", ec.message()));
    } else {
      auto vgs_status = gate_driver->vgs_status(ec);
      if (ec) {
        record("drv8353", "status-registers", CheckStatus::FAIL,
               fmt::format("Failed to read VGS_STATUS_2: {}", ec.message()));
      } else {
        const bool has_critical_fault =
            fault_status.any_vds_fault() || fault_status.overtemperature_shutdown() ||
            fault_status.gate_driver_fault() || fault_status.vds_ocp_fault() ||
            vgs_status.any_vgs_fault() || vgs_status.any_sense_overcurrent_fault();
        const bool has_warning_condition =
            fault_status.undervoltage_lockout() || fault_status.latched_fault() ||
            vgs_status.charge_pump_undervoltage() || vgs_status.overtemperature_warning();
        std::vector<std::string> flags;
        if (fault_status.undervoltage_lockout()) {
          flags.push_back("UVLO");
        }
        if (fault_status.gate_driver_fault()) {
          flags.push_back("GDF");
        }
        if (fault_status.any_vds_fault()) {
          flags.push_back("VDS");
        }
        if (fault_status.vds_ocp_fault()) {
          flags.push_back("VDS_OCP");
        }
        if (fault_status.overtemperature_shutdown()) {
          flags.push_back("OTSD");
        }
        if (fault_status.latched_fault()) {
          flags.push_back("FAULT");
        }
        if (vgs_status.charge_pump_undervoltage()) {
          flags.push_back("GDUV");
        }
        if (vgs_status.overtemperature_warning()) {
          flags.push_back("OTW");
        }
        if (vgs_status.any_vgs_fault()) {
          flags.push_back("VGS");
        }
        if (vgs_status.any_sense_overcurrent_fault()) {
          flags.push_back("SENSE_OCP");
        }
        if (flags.empty()) {
          flags.push_back("none");
        }

        auto details =
            fmt::format("fault_status=0x{:03X}, vgs_status=0x{:03X}, fault_pin_active={}, flags={}",
                        fault_status.raw, vgs_status.raw, gate_driver->fault_pin_active(),
                        join_strings(flags, "|"));
        if (has_critical_fault) {
          record("drv8353", "status-registers", CheckStatus::FAIL, details);
        } else if (has_warning_condition) {
          record("drv8353", "status-registers", CheckStatus::WARN,
                 details + "; UVLO/GDUV commonly indicates motor VM is not applied");
        } else {
          record("drv8353", "status-registers", CheckStatus::PASS, details);
        }
      }
    }

    std::this_thread::sleep_for(5ms);
    const auto field_strength = encoder->get_magnetic_field_strength();
    const auto tracking_status = encoder->get_tracking_status();
    const int count = encoder->get_count();
    const float angle_deg = encoder->get_mechanical_degrees();
    const float rpm = encoder->get_rpm();
    const bool encoder_ok = tracking_status == Bsp::Encoder::TrackingStatus::NORMAL &&
                            field_strength == Bsp::Encoder::MagneticFieldStrength::NORMAL &&
                            std::isfinite(angle_deg) && std::isfinite(rpm);
    record("encoder", "status", encoder_ok ? CheckStatus::PASS : CheckStatus::FAIL,
           fmt::format("count={}, angle_deg={:.3f}, rpm={:.3f}, field_strength={}, tracking={}",
                       count, angle_deg, rpm, to_string(field_strength),
                       to_string(tracking_status)));
  }

  size_t pass_count = 0;
  size_t warn_count = 0;
  size_t fail_count = 0;
  size_t skip_count = 0;
  for (const auto &result : results) {
    switch (result.status) {
    case CheckStatus::PASS:
      pass_count++;
      break;
    case CheckStatus::WARN:
      warn_count++;
      break;
    case CheckStatus::FAIL:
      fail_count++;
      break;
    case CheckStatus::SKIP:
      skip_count++;
      break;
    }
  }

  if (fail_count > 0) {
    bsp.set_blue_led_duty(15.0f);
    bsp.set_green_led_duty(0.0f);
  } else if (warn_count > 0) {
    bsp.set_blue_led_duty(15.0f);
    bsp.set_green_led_duty(15.0f);
  } else {
    bsp.set_blue_led_duty(0.0f);
    bsp.set_green_led_duty(15.0f);
  }

  logger.info("Bringup complete: {} pass, {} warn, {} fail, {} skip", pass_count, warn_count,
              fail_count, skip_count);

  fmt::print("\nPACE RACER bringup summary\n");
  fmt::print("{:<12} {:<24} {:<6} {}\n", "Subsystem", "Check", "Status", "Details");
  fmt::print("{:-<12} {:-<24} {:-<6} {:-<1}\n", "", "", "", "");
  for (const auto &result : results) {
    fmt::print("{:<12} {:<24} {:<6} {}\n", result.subsystem, result.check, to_string(result.status),
               result.details);
  }

  fmt::print("\nbringup_csv_begin\n");
  fmt::print("subsystem,check,status,details\n");
  for (const auto &result : results) {
    fmt::print("{},{},{},{}\n", csv_quote(result.subsystem), csv_quote(result.check),
               csv_quote(to_string(result.status)), csv_quote(result.details));
  }
  fmt::print("bringup_csv_end\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
