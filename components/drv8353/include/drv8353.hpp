#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <system_error>

#include <driver/gpio.h>

#include "base_peripheral.hpp"

namespace espp {
/**
 * DRV8353 Three-Phase Gate Driver (SPI)
 *
 * The DRV8353 uses 16-bit SPI frames where bit 15 selects read/write,
 * bits 14:11 select the register, and bits 10:0 contain the payload.
 * This component uses `BasePeripheral` for the raw byte transport and layers
 * the device-specific framing on top.
 *
 * \section drv8353_ex1 DRV8353 Example
 * \snippet drv8353_example.cpp drv8353 example
 */
class Drv8353 : public BasePeripheral<uint8_t, false> {
public:
  static constexpr std::chrono::microseconds DEFAULT_STARTUP_DELAY{40};
  static constexpr std::chrono::microseconds DEFAULT_INTER_FRAME_DELAY{1};

  /// DRV8353 register addresses.
  enum class Register : uint8_t {
    FAULT_STATUS_1 = 0x00,
    VGS_STATUS_2 = 0x01,
    DRIVER_CONTROL = 0x02,
    GATE_DRIVE_HS = 0x03,
    GATE_DRIVE_LS = 0x04,
    OCP_CONTROL = 0x05,
    CSA_CONTROL = 0x06,
    DRIVER_CONFIGURATION = 0x07,
  };

  /// Raw fault-status register wrapper.
  struct FaultStatus {
    uint16_t raw{0};

    bool any_fault() const { return raw != 0; }
  };

  /// Raw VGS-status register wrapper.
  struct VgsStatus {
    uint16_t raw{0};

    bool any_fault() const { return raw != 0; }
  };

  /// Snapshot of the DRV8353 register map.
  struct RegisterValues {
    uint16_t fault_status_1{0};
    uint16_t vgs_status_2{0};
    uint16_t driver_control{0};
    uint16_t gate_drive_hs{0};
    uint16_t gate_drive_ls{0};
    uint16_t ocp_control{0};
    uint16_t csa_control{0};
    uint16_t driver_configuration{0};
  };

  /// Datasheet gate-drive source-current settings in milliamps for register codes 0x0-0xF.
  inline static constexpr std::array<uint16_t, 16> SOURCE_CURRENT_MILLIAMPS = {
      50, 50, 100, 150, 300, 350, 400, 450, 550, 600, 650, 700, 850, 900, 950, 1000,
  };

  /// Datasheet gate-drive sink-current settings in milliamps for register codes 0x0-0xF.
  inline static constexpr std::array<uint16_t, 16> SINK_CURRENT_MILLIAMPS = {
      100, 100, 200, 300, 600, 700, 800, 900, 1100, 1200, 1300, 1400, 1700, 1800, 1900, 2000,
  };

  /// Datasheet gate-drive current selection for one half-bridge side.
  struct GateDriveCurrent {
    uint16_t source_milliamps{SOURCE_CURRENT_MILLIAMPS[0]};
    uint16_t sink_milliamps{SINK_CURRENT_MILLIAMPS[0]};
  };

  /// Configuration for the DRV8353 peripheral.
  struct Config {
    BasePeripheral::write_fn write{nullptr}; ///< Function to write bytes to the SPI device.
    BasePeripheral::read_fn read{nullptr};   ///< Function to read bytes from the SPI device.
    gpio_num_t enable_gpio{GPIO_NUM_NC};     ///< Optional nSLEEP / enable GPIO.
    gpio_num_t fault_gpio{GPIO_NUM_NC};      ///< Optional nFAULT GPIO.
    bool reset_before_init{true};            ///< Pulse nSLEEP low before SPI access.
    std::chrono::microseconds startup_delay{
        DEFAULT_STARTUP_DELAY}; ///< Minimum nSLEEP timing between toggles.
    std::chrono::microseconds inter_frame_delay{
        DEFAULT_INTER_FRAME_DELAY}; ///< Delay between the read command and response frame.
    std::optional<GateDriveCurrent> high_side_gate_drive_current{
        std::nullopt}; ///< Optional HS gate-drive current applied during initialize().
    std::optional<GateDriveCurrent> low_side_gate_drive_current{
        std::nullopt};    ///< Optional LS gate-drive current applied during initialize().
    bool auto_init{true}; ///< Automatically initialize on construction.
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// Construct the DRV8353 peripheral.
  explicit Drv8353(const Config &config);

  /// Initialize the DRV8353, optionally toggling the enable pin first.
  bool initialize(std::error_code &ec);

  /// Drive the optional enable / nSLEEP pin.
  void set_enabled(bool enabled, std::error_code &ec);

  /// Pulse the optional enable / nSLEEP pin low then high.
  bool reset(std::error_code &ec);

  /// Read the optional nFAULT pin. Returns false if no fault pin is configured.
  bool fault_pin_active() const;

  /// Clear any latched faults by pulsing the CLR_FLT bit in DRIVER_CONTROL.
  bool clear_faults(std::error_code &ec);

  /// Read a raw 11-bit register value.
  uint16_t read_register(Register reg, std::error_code &ec);

  /// Write a raw 11-bit register value.
  bool write_register(Register reg, uint16_t data, std::error_code &ec);

  /// Read the two status registers.
  FaultStatus fault_status(std::error_code &ec);
  VgsStatus vgs_status(std::error_code &ec);

  /// Read the configured datasheet gate-drive current for the high-side MOSFETs.
  GateDriveCurrent high_side_gate_drive_current(std::error_code &ec);

  /// Read the configured datasheet gate-drive current for the low-side MOSFETs.
  GateDriveCurrent low_side_gate_drive_current(std::error_code &ec);

  /// Set the high-side MOSFET gate-drive current using supported datasheet current values.
  bool set_high_side_gate_drive_current(const GateDriveCurrent &current, std::error_code &ec);

  /// Set the low-side MOSFET gate-drive current using supported datasheet current values.
  bool set_low_side_gate_drive_current(const GateDriveCurrent &current, std::error_code &ec);

  /// Read all visible registers into a snapshot.
  RegisterValues read_all_registers(std::error_code &ec);

  /// Write the configurable registers (0x02 through 0x07).
  bool write_registers(const RegisterValues &values, std::error_code &ec);

protected:
  static constexpr uint16_t READ_BIT = 1u << 15;
  static constexpr uint16_t REGISTER_SHIFT = 11;
  static constexpr uint16_t REGISTER_MASK = 0x0Fu;
  static constexpr uint16_t DATA_MASK = 0x07FFu;
  static constexpr uint16_t CLEAR_FAULT_MASK = 1u << 0;
  static constexpr uint16_t GATE_DRIVE_SOURCE_SHIFT = 4;
  static constexpr uint16_t GATE_DRIVE_SOURCE_MASK = 0x0Fu << GATE_DRIVE_SOURCE_SHIFT;
  static constexpr uint16_t GATE_DRIVE_SINK_MASK = 0x0Fu;

  static std::array<uint8_t, 2> to_bytes(uint16_t word);
  static uint16_t from_bytes(const uint8_t *data);
  static uint16_t make_read_frame(Register reg);
  static uint16_t make_write_frame(Register reg, uint16_t data);
  static std::optional<uint8_t> gate_drive_source_code(uint16_t milliamps);
  static std::optional<uint8_t> gate_drive_sink_code(uint16_t milliamps);
  static GateDriveCurrent decode_gate_drive_current(uint16_t value);
  static std::optional<uint16_t> encode_gate_drive_current(const GateDriveCurrent &current);

  bool configure_gpios(std::error_code &ec);
  bool send_frame(uint16_t frame, std::error_code &ec);
  bool receive_frame(uint16_t &frame, std::error_code &ec);

  Config config_{};
};
} // namespace espp
