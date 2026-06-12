#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
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
    enum class Bit : uint16_t {
      VDS_LC = 1u << 0,  ///< VDS overcurrent fault on phase C low-side MOSFET.
      VDS_HC = 1u << 1,  ///< VDS overcurrent fault on phase C high-side MOSFET.
      VDS_LB = 1u << 2,  ///< VDS overcurrent fault on phase B low-side MOSFET.
      VDS_HB = 1u << 3,  ///< VDS overcurrent fault on phase B high-side MOSFET.
      VDS_LA = 1u << 4,  ///< VDS overcurrent fault on phase A low-side MOSFET.
      VDS_HA = 1u << 5,  ///< VDS overcurrent fault on phase A high-side MOSFET.
      OTSD = 1u << 6,    ///< Overtemperature shutdown.
      UVLO = 1u << 7,    ///< Undervoltage lockout.
      GDF = 1u << 8,     ///< Gate driver fault.
      VDS_OCP = 1u << 9, ///< VDS monitor overcurrent summary fault.
      FAULT = 1u << 10,  ///< Fault type: 0 = warning, 1 = latched fault.
    };

    static constexpr uint16_t VDS_MASK =
        static_cast<uint16_t>(Bit::VDS_LC) | static_cast<uint16_t>(Bit::VDS_HC) |
        static_cast<uint16_t>(Bit::VDS_LB) | static_cast<uint16_t>(Bit::VDS_HB) |
        static_cast<uint16_t>(Bit::VDS_LA) | static_cast<uint16_t>(Bit::VDS_HA);

    uint16_t raw{0};

    bool any_fault() const { return raw != 0; }
    bool has(Bit bit) const { return (raw & static_cast<uint16_t>(bit)) != 0; }
    bool any_vds_fault() const { return (raw & VDS_MASK) != 0; }
    bool vds_low_side_c() const { return has(Bit::VDS_LC); }
    bool vds_high_side_c() const { return has(Bit::VDS_HC); }
    bool vds_low_side_b() const { return has(Bit::VDS_LB); }
    bool vds_high_side_b() const { return has(Bit::VDS_HB); }
    bool vds_low_side_a() const { return has(Bit::VDS_LA); }
    bool vds_high_side_a() const { return has(Bit::VDS_HA); }
    bool overtemperature_shutdown() const { return has(Bit::OTSD); }
    bool undervoltage_lockout() const { return has(Bit::UVLO); }
    bool gate_driver_fault() const { return has(Bit::GDF); }
    bool vds_ocp_fault() const { return has(Bit::VDS_OCP); }
    bool latched_fault() const { return has(Bit::FAULT); }
  };

  /// Raw VGS-status register wrapper.
  struct VgsStatus {
    enum class Bit : uint16_t {
      VGS_LC = 1u << 0, ///< VGS gate-drive fault on phase C low-side MOSFET.
      VGS_HC = 1u << 1, ///< VGS gate-drive fault on phase C high-side MOSFET.
      VGS_LB = 1u << 2, ///< VGS gate-drive fault on phase B low-side MOSFET.
      VGS_HB = 1u << 3, ///< VGS gate-drive fault on phase B high-side MOSFET.
      VGS_LA = 1u << 4, ///< VGS gate-drive fault on phase A low-side MOSFET.
      VGS_HA = 1u << 5, ///< VGS gate-drive fault on phase A high-side MOSFET.
      GDUV = 1u << 6,   ///< Charge pump undervoltage and/or VGLS undervoltage fault.
      OTW = 1u << 7,    ///< Overtemperature warning.
      SC_OC = 1u << 8,  ///< Sense/phase C overcurrent fault.
      SB_OC = 1u << 9,  ///< Sense/phase B overcurrent fault.
      SA_OC = 1u << 10, ///< Sense/phase A overcurrent fault.
    };

    static constexpr uint16_t VGS_MASK =
        static_cast<uint16_t>(Bit::VGS_LC) | static_cast<uint16_t>(Bit::VGS_HC) |
        static_cast<uint16_t>(Bit::VGS_LB) | static_cast<uint16_t>(Bit::VGS_HB) |
        static_cast<uint16_t>(Bit::VGS_LA) | static_cast<uint16_t>(Bit::VGS_HA);

    static constexpr uint16_t SENSE_OCP_MASK = static_cast<uint16_t>(Bit::SC_OC) |
                                               static_cast<uint16_t>(Bit::SB_OC) |
                                               static_cast<uint16_t>(Bit::SA_OC);

    uint16_t raw{0};

    bool any_fault() const { return raw != 0; }
    bool has(Bit bit) const { return (raw & static_cast<uint16_t>(bit)) != 0; }
    bool any_vgs_fault() const { return (raw & VGS_MASK) != 0; }
    bool any_sense_overcurrent_fault() const { return (raw & SENSE_OCP_MASK) != 0; }
    bool vgs_low_side_c() const { return has(Bit::VGS_LC); }
    bool vgs_high_side_c() const { return has(Bit::VGS_HC); }
    bool vgs_low_side_b() const { return has(Bit::VGS_LB); }
    bool vgs_high_side_b() const { return has(Bit::VGS_HB); }
    bool vgs_low_side_a() const { return has(Bit::VGS_LA); }
    bool vgs_high_side_a() const { return has(Bit::VGS_HA); }
    bool charge_pump_undervoltage() const {
      return has(Bit::GDUV);
    } // NOTE: same bit as VGLS undervoltage
    bool vgls_undervoltage() const {
      return has(Bit::GDUV);
    } // NOTE: same bit as charge pump undervoltage
    bool overtemperature_warning() const { return has(Bit::OTW); }
    bool phase_c_overcurrent() const { return has(Bit::SC_OC); }
    bool phase_b_overcurrent() const { return has(Bit::SB_OC); }
    bool phase_a_overcurrent() const { return has(Bit::SA_OC); }
  };

  enum class PwmMode : uint8_t {
    SIX_PWM = 0,         ///< 6x PWM mode.
    THREE_PWM = 1,       ///< 3x PWM mode.
    ONE_PWM = 2,         ///< 1x PWM mode.
    INDEPENDENT_PWM = 3, ///< Independent PWM mode.
  };

  enum class PeakDriveTime : uint8_t {
    NS_500 = 0,  ///< 500 ns peak gate-drive time.
    NS_1000 = 1, ///< 1000 ns peak gate-drive time.
    NS_2000 = 2, ///< 2000 ns peak gate-drive time.
    NS_4000 = 3, ///< 4000 ns peak gate-drive time.
  };

  enum class DeadTime : uint8_t {
    NS_50 = 0,  ///< 50 ns gate dead time.
    NS_100 = 1, ///< 100 ns gate dead time.
    NS_200 = 2, ///< 200 ns gate dead time.
    NS_400 = 3, ///< 400 ns gate dead time.
  };

  enum class RetryTime : uint8_t {
    MS_8 = 0,  ///< 8 ms retry time.
    US_50 = 1, ///< 50 us retry time.
  };

  enum class OcpMode : uint8_t {
    LATCHED_SHUTDOWN = 0, ///< Overcurrent latches a fault and shuts down.
    AUTOMATIC_RETRY = 1,  ///< Overcurrent automatically retries after tRETRY.
    REPORT_ONLY = 2,      ///< Overcurrent is reported but no shutdown occurs.
    DISABLED = 3,         ///< Overcurrent protection/reporting is disabled.
  };

  enum class OcpDeglitch : uint8_t {
    US_1 = 0, ///< 1 us OCP deglitch.
    US_2 = 1, ///< 2 us OCP deglitch.
    US_4 = 2, ///< 4 us OCP deglitch.
    US_8 = 3, ///< 8 us OCP deglitch.
  };

  enum class VdsLevel : uint8_t {
    V_0_06 = 0,
    V_0_07 = 1,
    V_0_08 = 2,
    V_0_09 = 3,
    V_0_10 = 4,
    V_0_20 = 5,
    V_0_30 = 6,
    V_0_40 = 7,
    V_0_50 = 8,
    V_0_60 = 9,
    V_0_70 = 10,
    V_0_80 = 11,
    V_0_90 = 12,
    V_1_00 = 13,
    V_1_50 = 14,
    V_2_00 = 15,
  };

  enum class SenseLevel : uint8_t {
    V_0_25 = 0, ///< 0.25 V sense OCP threshold.
    V_0_50 = 1, ///< 0.5 V sense OCP threshold.
    V_0_75 = 2, ///< 0.75 V sense OCP threshold.
    V_1_00 = 3, ///< 1.0 V sense OCP threshold.
  };

  enum class CsaGain : uint8_t {
    GAIN_5 = 0,  ///< 5 V/V shunt amplifier gain.
    GAIN_10 = 1, ///< 10 V/V shunt amplifier gain.
    GAIN_20 = 2, ///< 20 V/V shunt amplifier gain.
    GAIN_40 = 3, ///< 40 V/V shunt amplifier gain.
  };

  /// Raw driver-control register wrapper.
  struct DriverControl {
    uint16_t raw{0};

    bool overcurrent_shutdown_all_bridges() const { return (raw & (1u << 10)) != 0; }
    bool gate_uvlo_fault_disabled() const { return (raw & (1u << 9)) != 0; }
    bool gate_driver_fault_disabled() const { return (raw & (1u << 8)) != 0; }
    bool overtemperature_warning_reporting_enabled() const { return (raw & (1u << 7)) != 0; }
    PwmMode pwm_mode() const { return static_cast<PwmMode>((raw >> 5) & 0x03u); }
    bool asynchronous_rectification() const { return (raw & (1u << 4)) != 0; }
    bool pwm1_direction() const { return (raw & (1u << 3)) != 0; }
    bool coast() const { return (raw & (1u << 2)) != 0; }
    bool brake() const { return (raw & (1u << 1)) != 0; }
    bool clear_fault_requested() const { return (raw & (1u << 0)) != 0; }

    void set_overcurrent_shutdown_all_bridges(bool enabled) {
      raw = (raw & ~(1u << 10)) | (enabled ? (1u << 10) : 0u);
    }
    void set_gate_uvlo_fault_disabled(bool disabled) {
      raw = (raw & ~(1u << 9)) | (disabled ? (1u << 9) : 0u);
    }
    void set_gate_driver_fault_disabled(bool disabled) {
      raw = (raw & ~(1u << 8)) | (disabled ? (1u << 8) : 0u);
    }
    void set_overtemperature_warning_reporting_enabled(bool enabled) {
      raw = (raw & ~(1u << 7)) | (enabled ? (1u << 7) : 0u);
    }
    void set_pwm_mode(PwmMode mode) {
      raw = (raw & ~(0x03u << 5)) | (static_cast<uint16_t>(mode) << 5);
    }
    void set_asynchronous_rectification(bool enabled) {
      raw = (raw & ~(1u << 4)) | (enabled ? (1u << 4) : 0u);
    }
    void set_pwm1_direction(bool high) { raw = (raw & ~(1u << 3)) | (high ? (1u << 3) : 0u); }
    void set_coast(bool enabled) { raw = (raw & ~(1u << 2)) | (enabled ? (1u << 2) : 0u); }
    void set_brake(bool enabled) { raw = (raw & ~(1u << 1)) | (enabled ? (1u << 1) : 0u); }
  };

  /// Raw OCP-control register wrapper.
  struct OcpControl {
    uint16_t raw{0};

    RetryTime retry_time() const { return static_cast<RetryTime>((raw >> 10) & 0x01u); }
    DeadTime dead_time() const { return static_cast<DeadTime>((raw >> 8) & 0x03u); }
    OcpMode ocp_mode() const { return static_cast<OcpMode>((raw >> 6) & 0x03u); }
    OcpDeglitch deglitch() const { return static_cast<OcpDeglitch>((raw >> 4) & 0x03u); }
    VdsLevel vds_level() const { return static_cast<VdsLevel>(raw & 0x0Fu); }

    void set_retry_time(RetryTime retry_time) {
      raw = (raw & ~(1u << 10)) | (static_cast<uint16_t>(retry_time) << 10);
    }
    void set_dead_time(DeadTime dead_time) {
      raw = (raw & ~(0x03u << 8)) | (static_cast<uint16_t>(dead_time) << 8);
    }
    void set_ocp_mode(OcpMode mode) {
      raw = (raw & ~(0x03u << 6)) | (static_cast<uint16_t>(mode) << 6);
    }
    void set_deglitch(OcpDeglitch deglitch) {
      raw = (raw & ~(0x03u << 4)) | (static_cast<uint16_t>(deglitch) << 4);
    }
    void set_vds_level(VdsLevel level) { raw = (raw & ~0x0Fu) | static_cast<uint16_t>(level); }
  };

  /// Raw CSA-control register wrapper.
  struct CsaControl {
    uint16_t raw{0};

    bool fet_sense_enabled() const { return (raw & (1u << 10)) != 0; }
    bool vref_divided_by_two() const { return (raw & (1u << 9)) != 0; }
    bool low_side_reference_uses_snx() const { return (raw & (1u << 8)) != 0; }
    CsaGain gain() const { return static_cast<CsaGain>((raw >> 6) & 0x03u); }
    bool sense_overcurrent_disabled() const { return (raw & (1u << 5)) != 0; }
    bool phase_a_calibration_enabled() const { return (raw & (1u << 4)) != 0; }
    bool phase_b_calibration_enabled() const { return (raw & (1u << 3)) != 0; }
    bool phase_c_calibration_enabled() const { return (raw & (1u << 2)) != 0; }
    SenseLevel sense_level() const { return static_cast<SenseLevel>(raw & 0x03u); }

    void set_fet_sense_enabled(bool enabled) {
      raw = (raw & ~(1u << 10)) | (enabled ? (1u << 10) : 0u);
    }
    void set_vref_divided_by_two(bool enabled) {
      raw = (raw & ~(1u << 9)) | (enabled ? (1u << 9) : 0u);
    }
    void set_low_side_reference_to_snx(bool enabled) {
      raw = (raw & ~(1u << 8)) | (enabled ? (1u << 8) : 0u);
    }
    void set_gain(CsaGain gain) {
      raw = (raw & ~(0x03u << 6)) | (static_cast<uint16_t>(gain) << 6);
    }
    void set_sense_overcurrent_disabled(bool disabled) {
      raw = (raw & ~(1u << 5)) | (disabled ? (1u << 5) : 0u);
    }
    void set_phase_a_calibration_enabled(bool enabled) {
      raw = (raw & ~(1u << 4)) | (enabled ? (1u << 4) : 0u);
    }
    void set_phase_b_calibration_enabled(bool enabled) {
      raw = (raw & ~(1u << 3)) | (enabled ? (1u << 3) : 0u);
    }
    void set_phase_c_calibration_enabled(bool enabled) {
      raw = (raw & ~(1u << 2)) | (enabled ? (1u << 2) : 0u);
    }
    void set_sense_level(SenseLevel level) { raw = (raw & ~0x03u) | static_cast<uint16_t>(level); }
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

  /// Full-duplex SPI transfer for one DRV8353 frame.
  using transfer_fn =
      std::function<bool(std::span<const uint8_t> tx_data, std::span<uint8_t> rx_data)>;

  /// Configuration for the DRV8353 peripheral.
  struct Config {
    BasePeripheral::write_fn write{nullptr}; ///< Function to write bytes to the SPI device.
    BasePeripheral::read_fn read{nullptr};   ///< Function to read bytes from the SPI device.
    transfer_fn transfer{nullptr};           ///< Preferred full-duplex SPI transfer function.
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

  /// Alias for clear_faults().
  bool clear_fault(std::error_code &ec);

  /// Read a raw 11-bit register value.
  uint16_t read_register(Register reg, std::error_code &ec);

  /// Write a raw 11-bit register value.
  bool write_register(Register reg, uint16_t data, std::error_code &ec);

  /// Read the two status registers.
  FaultStatus fault_status(std::error_code &ec);
  VgsStatus vgs_status(std::error_code &ec);

  /// Read the driver-control register.
  DriverControl read_driver_control(std::error_code &ec);

  /// Write the driver-control register. CLR_FLT is always masked off; use clear_faults().
  bool write_driver_control(const DriverControl &control, std::error_code &ec);

  /// Read the OCP-control register.
  OcpControl read_ocp_control(std::error_code &ec);

  /// Write the OCP-control register.
  bool write_ocp_control(const OcpControl &control, std::error_code &ec);

  /// Read the CSA-control register.
  CsaControl read_csa_control(std::error_code &ec);

  /// Write the CSA-control register.
  bool write_csa_control(const CsaControl &control, std::error_code &ec);

  /// Read the configured datasheet gate-drive current for the high-side MOSFETs.
  GateDriveCurrent high_side_gate_drive_current(std::error_code &ec);

  /// Read the configured datasheet gate-drive current for the low-side MOSFETs.
  GateDriveCurrent low_side_gate_drive_current(std::error_code &ec);

  /// Set the high-side MOSFET gate-drive current using supported datasheet current values.
  bool set_high_side_gate_drive_current(const GateDriveCurrent &current, std::error_code &ec);

  /// Set the low-side MOSFET gate-drive current using supported datasheet current values.
  bool set_low_side_gate_drive_current(const GateDriveCurrent &current, std::error_code &ec);

  /// Set the COAST bit in DRIVER_CONTROL.
  bool set_coast(bool enabled, std::error_code &ec);

  /// Set the BRAKE bit in DRIVER_CONTROL.
  bool set_brake(bool enabled, std::error_code &ec);

  /// Set the PWM mode in DRIVER_CONTROL.
  bool set_pwm_mode(PwmMode mode, std::error_code &ec);

  /// Read the configured low-side peak gate-drive time.
  PeakDriveTime peak_drive_time(std::error_code &ec);

  /// Set the configured low-side peak gate-drive time.
  bool set_peak_drive_time(PeakDriveTime peak_drive_time, std::error_code &ec);

  /// Read whether cycle-by-cycle retry is enabled in GATE_DRIVE_LS.
  bool cycle_by_cycle_enabled(std::error_code &ec);

  /// Set the cycle-by-cycle retry behavior in GATE_DRIVE_LS.
  bool set_cycle_by_cycle(bool enabled, std::error_code &ec);

  /// Read the OCP retry time from OCP_CONTROL.
  RetryTime retry_time(std::error_code &ec);

  /// Set the OCP retry time in OCP_CONTROL.
  bool set_retry_time(RetryTime retry_time, std::error_code &ec);

  /// Read the configured gate dead time from OCP_CONTROL.
  DeadTime dead_time(std::error_code &ec);

  /// Set the configured gate dead time in OCP_CONTROL.
  bool set_dead_time(DeadTime dead_time, std::error_code &ec);

  /// Read the OCP operating mode from OCP_CONTROL.
  OcpMode ocp_mode(std::error_code &ec);

  /// Set the OCP operating mode in OCP_CONTROL.
  bool set_ocp_mode(OcpMode mode, std::error_code &ec);

  /// Read the OCP deglitch configuration from OCP_CONTROL.
  OcpDeglitch ocp_deglitch(std::error_code &ec);

  /// Set the OCP deglitch configuration in OCP_CONTROL.
  bool set_ocp_deglitch(OcpDeglitch deglitch, std::error_code &ec);

  /// Read the configured VDS threshold from OCP_CONTROL.
  VdsLevel vds_level(std::error_code &ec);

  /// Set the configured VDS threshold in OCP_CONTROL.
  bool set_vds_level(VdsLevel level, std::error_code &ec);

  /// Read the configured sense OCP threshold from CSA_CONTROL.
  SenseLevel sense_level(std::error_code &ec);

  /// Set the configured sense OCP threshold in CSA_CONTROL.
  bool set_sense_level(SenseLevel level, std::error_code &ec);

  /// Read the configured CSA gain from CSA_CONTROL.
  CsaGain csa_gain(std::error_code &ec);

  /// Set the configured CSA gain in CSA_CONTROL.
  bool set_csa_gain(CsaGain gain, std::error_code &ec);

  /// Enable or disable the CSA FET sensing path in CSA_CONTROL.
  bool set_csa_fet(bool enabled, std::error_code &ec);

  /// Enable or disable VREF/2 in CSA_CONTROL.
  bool set_vref_divider_enabled(bool enabled, std::error_code &ec);

  /// Select SHx-to-SNx low-side VDS measurement in CSA_CONTROL.
  bool set_low_side_reference_to_snx(bool enabled, std::error_code &ec);

  /// Enable or disable sense OCP in CSA_CONTROL.
  bool set_sense_overcurrent_enabled(bool enabled, std::error_code &ec);

  /// Control the three CSA calibration bits together.
  bool set_csa_calibration(bool phase_a_enabled, bool phase_b_enabled, bool phase_c_enabled,
                           std::error_code &ec);

  /// Read all visible registers into a snapshot.
  RegisterValues read_all_registers(std::error_code &ec);

  /// Write the configurable registers (0x02 through 0x07).
  bool write_registers(const RegisterValues &values, std::error_code &ec);

protected:
  static constexpr uint16_t READ_BIT = 1u << 15;
  static constexpr uint16_t REGISTER_SHIFT = 11;
  static constexpr uint16_t REGISTER_MASK = 0x0Fu;
  static constexpr uint16_t DATA_MASK = 0x07FFu;
  static constexpr uint16_t DRIVER_CONTROL_CLEAR_FAULT_MASK = 1u << 0;
  static constexpr uint16_t DRIVER_CONTROL_BRAKE_MASK = 1u << 1;
  static constexpr uint16_t DRIVER_CONTROL_COAST_MASK = 1u << 2;
  static constexpr uint16_t DRIVER_CONTROL_PWM1_DIR_MASK = 1u << 3;
  static constexpr uint16_t DRIVER_CONTROL_PWM1_COM_MASK = 1u << 4;
  static constexpr uint16_t DRIVER_CONTROL_PWM_MODE_SHIFT = 5;
  static constexpr uint16_t DRIVER_CONTROL_PWM_MODE_MASK = 0x03u << DRIVER_CONTROL_PWM_MODE_SHIFT;
  static constexpr uint16_t DRIVER_CONTROL_OTW_REP_MASK = 1u << 7;
  static constexpr uint16_t DRIVER_CONTROL_DIS_GDF_MASK = 1u << 8;
  static constexpr uint16_t DRIVER_CONTROL_DIS_GDUV_MASK = 1u << 9;
  static constexpr uint16_t DRIVER_CONTROL_OCP_ACT_MASK = 1u << 10;
  static constexpr uint16_t GATE_DRIVE_LOCK_SHIFT = 8;
  static constexpr uint16_t GATE_DRIVE_LOCK_MASK = 0x07u << GATE_DRIVE_LOCK_SHIFT;
  static constexpr uint16_t GATE_DRIVE_LOCK = 0x06u << GATE_DRIVE_LOCK_SHIFT;
  static constexpr uint16_t GATE_DRIVE_UNLOCK = 0x03u << GATE_DRIVE_LOCK_SHIFT;
  static constexpr uint16_t GATE_DRIVE_SOURCE_SHIFT = 4;
  static constexpr uint16_t GATE_DRIVE_SOURCE_MASK = 0x0Fu << GATE_DRIVE_SOURCE_SHIFT;
  static constexpr uint16_t GATE_DRIVE_SINK_MASK = 0x0Fu;
  static constexpr uint16_t GATE_DRIVE_TDRIVE_SHIFT = 8;
  static constexpr uint16_t GATE_DRIVE_TDRIVE_MASK = 0x03u << GATE_DRIVE_TDRIVE_SHIFT;
  static constexpr uint16_t GATE_DRIVE_CBC_MASK = 1u << 10;
  static constexpr uint16_t OCP_CONTROL_VDS_LEVEL_MASK = 0x0Fu;
  static constexpr uint16_t OCP_CONTROL_DEGLITCH_SHIFT = 4;
  static constexpr uint16_t OCP_CONTROL_DEGLITCH_MASK = 0x03u << OCP_CONTROL_DEGLITCH_SHIFT;
  static constexpr uint16_t OCP_CONTROL_MODE_SHIFT = 6;
  static constexpr uint16_t OCP_CONTROL_MODE_MASK = 0x03u << OCP_CONTROL_MODE_SHIFT;
  static constexpr uint16_t OCP_CONTROL_DEAD_TIME_SHIFT = 8;
  static constexpr uint16_t OCP_CONTROL_DEAD_TIME_MASK = 0x03u << OCP_CONTROL_DEAD_TIME_SHIFT;
  static constexpr uint16_t OCP_CONTROL_RETRY_MASK = 1u << 10;
  static constexpr uint16_t CSA_CONTROL_SENSE_LEVEL_MASK = 0x03u;
  static constexpr uint16_t CSA_CONTROL_CAL_C_MASK = 1u << 2;
  static constexpr uint16_t CSA_CONTROL_CAL_B_MASK = 1u << 3;
  static constexpr uint16_t CSA_CONTROL_CAL_A_MASK = 1u << 4;
  static constexpr uint16_t CSA_CONTROL_DIS_SEN_MASK = 1u << 5;
  static constexpr uint16_t CSA_CONTROL_GAIN_SHIFT = 6;
  static constexpr uint16_t CSA_CONTROL_GAIN_MASK = 0x03u << CSA_CONTROL_GAIN_SHIFT;
  static constexpr uint16_t CSA_CONTROL_LS_REF_MASK = 1u << 8;
  static constexpr uint16_t CSA_CONTROL_VREF_DIV_MASK = 1u << 9;
  static constexpr uint16_t CSA_CONTROL_FET_MASK = 1u << 10;

  static BasePeripheral::write_fn make_write_fn(const Config &config);
  static BasePeripheral::read_fn make_read_fn(const Config &config);
  static std::array<uint8_t, 2> to_bytes(uint16_t word);
  static uint16_t from_bytes(const uint8_t *data);
  static uint16_t make_read_frame(Register reg);
  static uint16_t make_write_frame(Register reg, uint16_t data);
  static std::optional<uint8_t> gate_drive_source_code(uint16_t milliamps);
  static std::optional<uint8_t> gate_drive_sink_code(uint16_t milliamps);
  static GateDriveCurrent decode_gate_drive_current(uint16_t value);
  static std::optional<uint16_t> encode_gate_drive_current(const GateDriveCurrent &current);
  static uint16_t with_gate_drive_lock(uint16_t value, uint16_t lock_bits);
  static bool bit_is_set(uint16_t value, uint16_t mask);
  static uint16_t get_field(uint16_t value, uint16_t mask, uint16_t shift);
  static uint16_t set_bit(uint16_t value, uint16_t mask, bool enabled);
  static uint16_t set_field(uint16_t value, uint16_t mask, uint16_t shift, uint16_t field_value);

  bool configure_gpios(std::error_code &ec);
  bool transfer_frame(uint16_t tx_frame, uint16_t &rx_frame, std::error_code &ec);
  bool send_frame(uint16_t frame, std::error_code &ec);
  bool receive_frame(uint16_t &frame, std::error_code &ec);
  bool unlock_protected_registers(uint16_t &gate_drive_hs, uint16_t &original_lock,
                                  std::error_code &ec);
  bool restore_protected_register_lock(uint16_t gate_drive_hs, uint16_t original_lock,
                                       std::error_code &ec);
  bool write_protected_register(Register reg, uint16_t data, std::error_code &ec);
  bool modify_register(Register reg, uint16_t mask, uint16_t value, std::error_code &ec,
                       bool requires_unlock = false);

  Config config_{};
};
} // namespace espp
