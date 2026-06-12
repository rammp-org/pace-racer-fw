#pragma once

#include <cstdint>
#include <system_error>

#include "base_peripheral.hpp"

namespace espp {
/**
 * LM75ADP Digital Temperature Sensor (I2C)
 *
 * Provides helpers to read the current temperature, access the configuration
 * register, and program the hysteresis / overtemperature thresholds.
 *
 * \section lm75adp_ex1 LM75ADP Example
 * \snippet lm75adp_example.cpp lm75adp example
 */
class Lm75adp : public BasePeripheral<uint8_t, true> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x48; ///< Default I2C address.

  /// Output polarity for the OS pin.
  enum class OsPolarity : uint8_t {
    ACTIVE_LOW = 0,
    ACTIVE_HIGH = 1,
  };

  /// Operating mode for the OS pin.
  enum class OsMode : uint8_t {
    COMPARATOR_MODE = 0,
    INTERRUPT_MODE = 1,
  };

  /// Number of consecutive faults before OS asserts.
  enum class FaultQueue : uint8_t {
    ONE = 0,
    TWO = 1,
    FOUR = 2,
    SIX = 3,
  };

  /// Decoded configuration register fields.
  struct Configuration {
    OsPolarity os_polarity{OsPolarity::ACTIVE_LOW};
    bool shutdown{false};
    OsMode os_mode{OsMode::COMPARATOR_MODE};
    FaultQueue fault_queue{FaultQueue::ONE};

    /// Encode the configuration for register writes.
    uint8_t raw() const;

    /// Decode a raw configuration register value.
    static Configuration from_raw(uint8_t raw);
  };

  /// Configuration for the LM75ADP component.
  struct Config {
    uint8_t device_address{DEFAULT_ADDRESS}; ///< I2C address of the device.
    BasePeripheral::probe_fn probe{nullptr}; ///< Probe function to check device presence.
    BasePeripheral::write_fn write{nullptr}; ///< Function to write to the device.
    BasePeripheral::read_register_fn read_register{
        nullptr}; ///< Function to read a register from the device.
    BasePeripheral::write_then_read_fn write_then_read{
        nullptr};         ///< Optional function to write then read from the device.
    bool auto_init{true}; ///< Automatically initialize on construction.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// Construct the LM75ADP peripheral.
  explicit Lm75adp(const Config &config);

  /// Initialize the LM75ADP and optionally probe the configured address.
  bool initialize(std::error_code &ec);

  /// Read the current temperature in degrees Celsius.
  float temperature_c(std::error_code &ec) const;

  /// Read the current hysteresis threshold in degrees Celsius.
  float hysteresis_temperature_c(std::error_code &ec) const;

  /// Read the overtemperature shutdown threshold in degrees Celsius.
  float overtemperature_shutdown_c(std::error_code &ec) const;

  /// Program the hysteresis threshold in degrees Celsius.
  void set_hysteresis_temperature_c(float temperature_c, std::error_code &ec);

  /// Program the overtemperature shutdown threshold in degrees Celsius.
  void set_overtemperature_shutdown_c(float temperature_c, std::error_code &ec);

  /// Read the decoded configuration register.
  Configuration configuration(std::error_code &ec) const;

  /// Write the decoded configuration register.
  void set_configuration(const Configuration &configuration, std::error_code &ec);

  /// Set or clear shutdown mode.
  void set_shutdown_enabled(bool enabled, std::error_code &ec);

  /// Set the OS output polarity.
  void set_os_polarity(OsPolarity polarity, std::error_code &ec);

  /// Set the OS comparator / interrupt mode.
  void set_os_mode(OsMode mode, std::error_code &ec);

  /// Set the fault queue depth.
  void set_fault_queue(FaultQueue queue, std::error_code &ec);

protected:
  enum class Register : uint8_t {
    TEMPERATURE = 0x00,
    CONFIGURATION = 0x01,
    THYST = 0x02,
    TOS = 0x03,
  };

  static float register_to_celsius(uint16_t raw_register);
  static uint16_t celsius_to_register(float temperature_c);

  Config config_{};
};
} // namespace espp
