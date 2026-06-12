#include "lm75adp.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>

using namespace espp;

namespace {
// LM75A configuration register fields per datasheet:
// B0 = shutdown, B1 = OS comparator/interrupt mode,
// B2 = OS polarity, B[4:3] = fault queue, B[7:5] = reserved.
constexpr uint8_t kShutdownMask = 1 << 0;
constexpr uint8_t kOsModeMask = 1 << 1;
constexpr uint8_t kOsPolarityMask = 1 << 2;
constexpr uint8_t kFaultQueueShift = 3;
constexpr uint8_t kFaultQueueMask = 0x3 << kFaultQueueShift;
constexpr float kTemperatureLsbC = 0.125f;
// LM75A register table range: -55 C to +127 C in 0.125 C steps.
constexpr int kMinTemperatureSteps = -440;
constexpr int kMaxTemperatureSteps = 1016;
} // namespace

uint8_t Lm75adp::Configuration::raw() const {
  uint8_t value = 0;
  value |= os_polarity == OsPolarity::ACTIVE_HIGH ? kOsPolarityMask : 0;
  value |= shutdown ? kShutdownMask : 0;
  value |= os_mode == OsMode::INTERRUPT_MODE ? kOsModeMask : 0;
  value |= (static_cast<uint8_t>(fault_queue) & 0x03) << kFaultQueueShift;
  return value;
}

Lm75adp::Configuration Lm75adp::Configuration::from_raw(uint8_t raw) {
  return {
      .os_polarity = (raw & kOsPolarityMask) ? OsPolarity::ACTIVE_HIGH : OsPolarity::ACTIVE_LOW,
      .shutdown = (raw & kShutdownMask) != 0,
      .os_mode = (raw & kOsModeMask) ? OsMode::INTERRUPT_MODE : OsMode::COMPARATOR_MODE,
      .fault_queue = static_cast<FaultQueue>((raw & kFaultQueueMask) >> kFaultQueueShift),
  };
}

Lm75adp::Lm75adp(const Config &config)
    : BasePeripheral({.address = config.device_address,
                      .probe = config.probe,
                      .write = config.write,
                      .read_register = config.read_register,
                      .write_then_read = config.write_then_read},
                     "Lm75adp", config.log_level)
    , config_(config) {
  if (config.auto_init) {
    std::error_code ec;
    if (!initialize(ec)) {
      logger_.error("Failed to initialize LM75ADP: {}", ec.message());
    }
  }
}

bool Lm75adp::initialize(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (config_.probe && !probe(ec)) {
    if (!ec) {
      ec = std::make_error_code(std::errc::no_such_device);
    }
    return false;
  }

  [[maybe_unused]] auto config_raw =
      read_u8_from_register(static_cast<uint8_t>(Register::CONFIGURATION), ec);
  return !ec;
}

float Lm75adp::temperature_c(std::error_code &ec) const {
  auto raw = read_u16_from_register(static_cast<uint8_t>(Register::TEMPERATURE), ec);
  if (ec) {
    return 0.0f;
  }
  return register_to_celsius(raw);
}

float Lm75adp::hysteresis_temperature_c(std::error_code &ec) const {
  auto raw = read_u16_from_register(static_cast<uint8_t>(Register::THYST), ec);
  if (ec) {
    return 0.0f;
  }
  return register_to_celsius(raw);
}

float Lm75adp::overtemperature_shutdown_c(std::error_code &ec) const {
  auto raw = read_u16_from_register(static_cast<uint8_t>(Register::TOS), ec);
  if (ec) {
    return 0.0f;
  }
  return register_to_celsius(raw);
}

void Lm75adp::set_hysteresis_temperature_c(float temperature_c, std::error_code &ec) {
  write_u16_to_register(static_cast<uint8_t>(Register::THYST), celsius_to_register(temperature_c),
                        ec);
}

void Lm75adp::set_overtemperature_shutdown_c(float temperature_c, std::error_code &ec) {
  write_u16_to_register(static_cast<uint8_t>(Register::TOS), celsius_to_register(temperature_c),
                        ec);
}

Lm75adp::Configuration Lm75adp::configuration(std::error_code &ec) const {
  auto raw = read_u8_from_register(static_cast<uint8_t>(Register::CONFIGURATION), ec);
  if (ec) {
    return {};
  }
  return Configuration::from_raw(raw);
}

void Lm75adp::set_configuration(const Configuration &configuration, std::error_code &ec) {
  write_u8_to_register(static_cast<uint8_t>(Register::CONFIGURATION), configuration.raw(), ec);
}

void Lm75adp::set_shutdown_enabled(bool enabled, std::error_code &ec) {
  auto config = configuration(ec);
  if (ec) {
    return;
  }
  config.shutdown = enabled;
  set_configuration(config, ec);
}

void Lm75adp::set_os_polarity(OsPolarity polarity, std::error_code &ec) {
  auto config = configuration(ec);
  if (ec) {
    return;
  }
  config.os_polarity = polarity;
  set_configuration(config, ec);
}

void Lm75adp::set_os_mode(OsMode mode, std::error_code &ec) {
  auto config = configuration(ec);
  if (ec) {
    return;
  }
  config.os_mode = mode;
  set_configuration(config, ec);
}

void Lm75adp::set_fault_queue(FaultQueue queue, std::error_code &ec) {
  auto config = configuration(ec);
  if (ec) {
    return;
  }
  config.fault_queue = queue;
  set_configuration(config, ec);
}

float Lm75adp::register_to_celsius(uint16_t raw_register) {
  auto steps = static_cast<int16_t>(raw_register) / 32;
  return static_cast<float>(steps) * kTemperatureLsbC;
}

uint16_t Lm75adp::celsius_to_register(float temperature_c) {
  int steps = static_cast<int>(std::lround(temperature_c / kTemperatureLsbC));
  steps = std::clamp(steps, kMinTemperatureSteps, kMaxTemperatureSteps);
  auto encoded = static_cast<int16_t>(steps * 32);
  return static_cast<uint16_t>(encoded);
}
