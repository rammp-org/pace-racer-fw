#include "drv8353.hpp"

#include <thread>

using namespace espp;

Drv8353::BasePeripheral::write_fn Drv8353::make_write_fn(const Config &config) {
  if (config.write) {
    return config.write;
  }
  if (!config.transfer) {
    return nullptr;
  }
  auto transfer = config.transfer;
  return [transfer](const uint8_t *data, size_t length) {
    return transfer(std::span<const uint8_t>(data, length), std::span<uint8_t>{});
  };
}

Drv8353::BasePeripheral::read_fn Drv8353::make_read_fn(const Config &config) {
  if (config.read) {
    return config.read;
  }
  if (!config.transfer) {
    return nullptr;
  }
  auto transfer = config.transfer;
  return [transfer](uint8_t *data, size_t length) {
    return transfer(std::span<const uint8_t>{}, std::span<uint8_t>(data, length));
  };
}

Drv8353::Drv8353(const Config &config)
    : BasePeripheral({.write = make_write_fn(config), .read = make_read_fn(config)}, "Drv8353",
                     config.log_level)
    , config_(config) {
  if (config.auto_init) {
    std::error_code ec;
    if (!initialize(ec)) {
      logger_.error("Failed to initialize DRV8353: {}", ec.message());
    }
  }
}

bool Drv8353::initialize(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (!config_.transfer && (!config_.write || !config_.read)) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  if (!configure_gpios(ec)) {
    return false;
  }

  if (config_.enable_gpio != GPIO_NUM_NC) {
    if (config_.reset_before_init) {
      if (!reset(ec)) {
        return false;
      }
    } else {
      set_enabled(true, ec);
      if (ec) {
        return false;
      }
      std::this_thread::sleep_for(config_.startup_delay);
    }
  }

  [[maybe_unused]] auto driver_control = read_register(Register::DRIVER_CONTROL, ec);
  if (ec) {
    return false;
  }

  if (config_.high_side_gate_drive_current &&
      !set_high_side_gate_drive_current(*config_.high_side_gate_drive_current, ec)) {
    return false;
  }

  if (config_.low_side_gate_drive_current &&
      !set_low_side_gate_drive_current(*config_.low_side_gate_drive_current, ec)) {
    return false;
  }

  return clear_faults(ec);
}

void Drv8353::set_enabled(bool enabled, std::error_code &ec) {
  if (config_.enable_gpio == GPIO_NUM_NC) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return;
  }
  auto err = gpio_set_level(config_.enable_gpio, enabled ? 1 : 0);
  if (err != ESP_OK) {
    ec = std::make_error_code(std::errc::io_error);
    return;
  }
  ec.clear();
}

bool Drv8353::reset(std::error_code &ec) {
  if (config_.enable_gpio == GPIO_NUM_NC) {
    ec = std::make_error_code(std::errc::operation_not_supported);
    return false;
  }

  set_enabled(false, ec);
  if (ec) {
    return false;
  }
  std::this_thread::sleep_for(config_.startup_delay);

  set_enabled(true, ec);
  if (ec) {
    return false;
  }
  std::this_thread::sleep_for(config_.startup_delay);
  return true;
}

bool Drv8353::fault_pin_active() const {
  if (config_.fault_gpio == GPIO_NUM_NC) {
    return false;
  }
  return gpio_get_level(config_.fault_gpio) == 0;
}

bool Drv8353::clear_faults(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  auto driver_control = read_register(Register::DRIVER_CONTROL, ec);
  if (ec) {
    return false;
  }

  if (!write_protected_register(Register::DRIVER_CONTROL,
                                driver_control | DRIVER_CONTROL_CLEAR_FAULT_MASK, ec)) {
    return false;
  }
  return write_protected_register(Register::DRIVER_CONTROL,
                                  driver_control & ~DRIVER_CONTROL_CLEAR_FAULT_MASK, ec);
}

bool Drv8353::clear_fault(std::error_code &ec) { return clear_faults(ec); }

Drv8353::DriverControl Drv8353::read_driver_control(std::error_code &ec) {
  return {.raw = read_register(Register::DRIVER_CONTROL, ec)};
}

bool Drv8353::write_driver_control(const DriverControl &control, std::error_code &ec) {
  auto raw = (control.raw & DATA_MASK) & ~DRIVER_CONTROL_CLEAR_FAULT_MASK;
  return write_protected_register(Register::DRIVER_CONTROL, raw, ec);
}

Drv8353::OcpControl Drv8353::read_ocp_control(std::error_code &ec) {
  return {.raw = read_register(Register::OCP_CONTROL, ec)};
}

bool Drv8353::write_ocp_control(const OcpControl &control, std::error_code &ec) {
  return write_protected_register(Register::OCP_CONTROL, control.raw & DATA_MASK, ec);
}

Drv8353::CsaControl Drv8353::read_csa_control(std::error_code &ec) {
  return {.raw = read_register(Register::CSA_CONTROL, ec)};
}

bool Drv8353::write_csa_control(const CsaControl &control, std::error_code &ec) {
  return write_protected_register(Register::CSA_CONTROL, control.raw & DATA_MASK, ec);
}

uint16_t Drv8353::read_register(Register reg, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  uint16_t response = 0;
  if (!transfer_frame(make_read_frame(reg), response, ec)) {
    return 0;
  }
  return response & DATA_MASK;
}

bool Drv8353::write_register(Register reg, uint16_t data, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  return send_frame(make_write_frame(reg, data), ec);
}

Drv8353::FaultStatus Drv8353::fault_status(std::error_code &ec) {
  return {.raw = read_register(Register::FAULT_STATUS_1, ec)};
}

Drv8353::VgsStatus Drv8353::vgs_status(std::error_code &ec) {
  return {.raw = read_register(Register::VGS_STATUS_2, ec)};
}

Drv8353::GateDriveCurrent Drv8353::high_side_gate_drive_current(std::error_code &ec) {
  return decode_gate_drive_current(read_register(Register::GATE_DRIVE_HS, ec));
}

Drv8353::GateDriveCurrent Drv8353::low_side_gate_drive_current(std::error_code &ec) {
  return decode_gate_drive_current(read_register(Register::GATE_DRIVE_LS, ec));
}

bool Drv8353::set_high_side_gate_drive_current(const GateDriveCurrent &current,
                                               std::error_code &ec) {
  auto encoded = encode_gate_drive_current(current);
  if (!encoded) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

  auto gate_drive_hs = read_register(Register::GATE_DRIVE_HS, ec);
  if (ec) {
    return false;
  }

  gate_drive_hs &= ~(GATE_DRIVE_SOURCE_MASK | GATE_DRIVE_SINK_MASK);
  gate_drive_hs |= *encoded;
  return write_protected_register(Register::GATE_DRIVE_HS, gate_drive_hs, ec);
}

bool Drv8353::set_low_side_gate_drive_current(const GateDriveCurrent &current,
                                              std::error_code &ec) {
  auto encoded = encode_gate_drive_current(current);
  if (!encoded) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

  auto gate_drive_ls = read_register(Register::GATE_DRIVE_LS, ec);
  if (ec) {
    return false;
  }

  gate_drive_ls &= ~(GATE_DRIVE_SOURCE_MASK | GATE_DRIVE_SINK_MASK);
  gate_drive_ls |= *encoded;
  return write_protected_register(Register::GATE_DRIVE_LS, gate_drive_ls, ec);
}

bool Drv8353::set_coast(bool enabled, std::error_code &ec) {
  return modify_register(Register::DRIVER_CONTROL, DRIVER_CONTROL_COAST_MASK,
                         enabled ? DRIVER_CONTROL_COAST_MASK : 0u, ec, true);
}

bool Drv8353::set_brake(bool enabled, std::error_code &ec) {
  return modify_register(Register::DRIVER_CONTROL, DRIVER_CONTROL_BRAKE_MASK,
                         enabled ? DRIVER_CONTROL_BRAKE_MASK : 0u, ec, true);
}

bool Drv8353::set_pwm_mode(PwmMode mode, std::error_code &ec) {
  return modify_register(Register::DRIVER_CONTROL, DRIVER_CONTROL_PWM_MODE_MASK,
                         static_cast<uint16_t>(mode) << DRIVER_CONTROL_PWM_MODE_SHIFT, ec, true);
}

Drv8353::PeakDriveTime Drv8353::peak_drive_time(std::error_code &ec) {
  auto gate_drive_ls = read_register(Register::GATE_DRIVE_LS, ec);
  return static_cast<PeakDriveTime>(
      get_field(gate_drive_ls, GATE_DRIVE_TDRIVE_MASK, GATE_DRIVE_TDRIVE_SHIFT));
}

bool Drv8353::set_peak_drive_time(PeakDriveTime peak_drive_time, std::error_code &ec) {
  return modify_register(Register::GATE_DRIVE_LS, GATE_DRIVE_TDRIVE_MASK,
                         static_cast<uint16_t>(peak_drive_time) << GATE_DRIVE_TDRIVE_SHIFT, ec,
                         true);
}

bool Drv8353::cycle_by_cycle_enabled(std::error_code &ec) {
  auto gate_drive_ls = read_register(Register::GATE_DRIVE_LS, ec);
  return !ec && bit_is_set(gate_drive_ls, GATE_DRIVE_CBC_MASK);
}

bool Drv8353::set_cycle_by_cycle(bool enabled, std::error_code &ec) {
  return modify_register(Register::GATE_DRIVE_LS, GATE_DRIVE_CBC_MASK,
                         enabled ? GATE_DRIVE_CBC_MASK : 0u, ec, true);
}

Drv8353::RetryTime Drv8353::retry_time(std::error_code &ec) {
  auto control = read_ocp_control(ec);
  return control.retry_time();
}

bool Drv8353::set_retry_time(RetryTime retry_time, std::error_code &ec) {
  return modify_register(Register::OCP_CONTROL, OCP_CONTROL_RETRY_MASK,
                         static_cast<uint16_t>(retry_time) << 10, ec, true);
}

Drv8353::DeadTime Drv8353::dead_time(std::error_code &ec) {
  auto control = read_ocp_control(ec);
  return control.dead_time();
}

bool Drv8353::set_dead_time(DeadTime dead_time, std::error_code &ec) {
  return modify_register(Register::OCP_CONTROL, OCP_CONTROL_DEAD_TIME_MASK,
                         static_cast<uint16_t>(dead_time) << OCP_CONTROL_DEAD_TIME_SHIFT, ec, true);
}

Drv8353::OcpMode Drv8353::ocp_mode(std::error_code &ec) {
  auto control = read_ocp_control(ec);
  return control.ocp_mode();
}

bool Drv8353::set_ocp_mode(OcpMode mode, std::error_code &ec) {
  return modify_register(Register::OCP_CONTROL, OCP_CONTROL_MODE_MASK,
                         static_cast<uint16_t>(mode) << OCP_CONTROL_MODE_SHIFT, ec, true);
}

Drv8353::OcpDeglitch Drv8353::ocp_deglitch(std::error_code &ec) {
  auto control = read_ocp_control(ec);
  return control.deglitch();
}

bool Drv8353::set_ocp_deglitch(OcpDeglitch deglitch, std::error_code &ec) {
  return modify_register(Register::OCP_CONTROL, OCP_CONTROL_DEGLITCH_MASK,
                         static_cast<uint16_t>(deglitch) << OCP_CONTROL_DEGLITCH_SHIFT, ec, true);
}

Drv8353::VdsLevel Drv8353::vds_level(std::error_code &ec) {
  auto control = read_ocp_control(ec);
  return control.vds_level();
}

bool Drv8353::set_vds_level(VdsLevel level, std::error_code &ec) {
  return modify_register(Register::OCP_CONTROL, OCP_CONTROL_VDS_LEVEL_MASK,
                         static_cast<uint16_t>(level), ec, true);
}

Drv8353::SenseLevel Drv8353::sense_level(std::error_code &ec) {
  auto control = read_csa_control(ec);
  return control.sense_level();
}

bool Drv8353::set_sense_level(SenseLevel level, std::error_code &ec) {
  return modify_register(Register::CSA_CONTROL, CSA_CONTROL_SENSE_LEVEL_MASK,
                         static_cast<uint16_t>(level), ec, true);
}

Drv8353::CsaGain Drv8353::csa_gain(std::error_code &ec) {
  auto control = read_csa_control(ec);
  return control.gain();
}

bool Drv8353::set_csa_gain(CsaGain gain, std::error_code &ec) {
  return modify_register(Register::CSA_CONTROL, CSA_CONTROL_GAIN_MASK,
                         static_cast<uint16_t>(gain) << CSA_CONTROL_GAIN_SHIFT, ec, true);
}

bool Drv8353::set_csa_fet(bool enabled, std::error_code &ec) {
  return modify_register(Register::CSA_CONTROL, CSA_CONTROL_FET_MASK,
                         enabled ? CSA_CONTROL_FET_MASK : 0u, ec, true);
}

bool Drv8353::set_vref_divider_enabled(bool enabled, std::error_code &ec) {
  return modify_register(Register::CSA_CONTROL, CSA_CONTROL_VREF_DIV_MASK,
                         enabled ? CSA_CONTROL_VREF_DIV_MASK : 0u, ec, true);
}

bool Drv8353::set_low_side_reference_to_snx(bool enabled, std::error_code &ec) {
  return modify_register(Register::CSA_CONTROL, CSA_CONTROL_LS_REF_MASK,
                         enabled ? CSA_CONTROL_LS_REF_MASK : 0u, ec, true);
}

bool Drv8353::set_sense_overcurrent_enabled(bool enabled, std::error_code &ec) {
  return modify_register(Register::CSA_CONTROL, CSA_CONTROL_DIS_SEN_MASK,
                         enabled ? 0u : CSA_CONTROL_DIS_SEN_MASK, ec, true);
}

bool Drv8353::set_csa_calibration(bool phase_a_enabled, bool phase_b_enabled, bool phase_c_enabled,
                                  std::error_code &ec) {
  auto value = 0u;
  if (phase_a_enabled) {
    value |= CSA_CONTROL_CAL_A_MASK;
  }
  if (phase_b_enabled) {
    value |= CSA_CONTROL_CAL_B_MASK;
  }
  if (phase_c_enabled) {
    value |= CSA_CONTROL_CAL_C_MASK;
  }
  return modify_register(Register::CSA_CONTROL,
                         CSA_CONTROL_CAL_A_MASK | CSA_CONTROL_CAL_B_MASK | CSA_CONTROL_CAL_C_MASK,
                         value, ec, true);
}

Drv8353::RegisterValues Drv8353::read_all_registers(std::error_code &ec) {
  RegisterValues values{};
  values.fault_status_1 = read_register(Register::FAULT_STATUS_1, ec);
  if (ec)
    return values;
  values.vgs_status_2 = read_register(Register::VGS_STATUS_2, ec);
  if (ec)
    return values;
  values.driver_control = read_register(Register::DRIVER_CONTROL, ec);
  if (ec)
    return values;
  values.gate_drive_hs = read_register(Register::GATE_DRIVE_HS, ec);
  if (ec)
    return values;
  values.gate_drive_ls = read_register(Register::GATE_DRIVE_LS, ec);
  if (ec)
    return values;
  values.ocp_control = read_register(Register::OCP_CONTROL, ec);
  if (ec)
    return values;
  values.csa_control = read_register(Register::CSA_CONTROL, ec);
  if (ec)
    return values;
  values.driver_configuration = read_register(Register::DRIVER_CONFIGURATION, ec);
  return values;
}

bool Drv8353::write_registers(const RegisterValues &values, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);

  uint16_t original_gate_drive_hs = 0;
  uint16_t original_lock = GATE_DRIVE_UNLOCK;
  if (!unlock_protected_registers(original_gate_drive_hs, original_lock, ec)) {
    return false;
  }

  auto driver_control = (values.driver_control & DATA_MASK) & ~DRIVER_CONTROL_CLEAR_FAULT_MASK;
  auto gate_drive_hs = values.gate_drive_hs & DATA_MASK;
  auto gate_drive_ls = values.gate_drive_ls & DATA_MASK;
  auto ocp_control = values.ocp_control & DATA_MASK;
  auto csa_control = values.csa_control & DATA_MASK;
  auto driver_configuration = values.driver_configuration & DATA_MASK;

  if (!write_register(Register::DRIVER_CONTROL, driver_control, ec) ||
      !write_register(Register::GATE_DRIVE_HS,
                      with_gate_drive_lock(gate_drive_hs, GATE_DRIVE_UNLOCK), ec) ||
      !write_register(Register::GATE_DRIVE_LS, gate_drive_ls, ec) ||
      !write_register(Register::OCP_CONTROL, ocp_control, ec) ||
      !write_register(Register::CSA_CONTROL, csa_control, ec) ||
      !write_register(Register::DRIVER_CONFIGURATION, driver_configuration, ec)) {
    if (original_lock != GATE_DRIVE_UNLOCK) {
      std::error_code restore_ec;
      restore_protected_register_lock(original_gate_drive_hs, original_lock, restore_ec);
    }
    return false;
  }

  return restore_protected_register_lock(gate_drive_hs, original_lock, ec);
}

std::array<uint8_t, 2> Drv8353::to_bytes(uint16_t word) {
  return {static_cast<uint8_t>((word >> 8) & 0xFF), static_cast<uint8_t>(word & 0xFF)};
}

uint16_t Drv8353::from_bytes(const uint8_t *data) {
  return (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
}

uint16_t Drv8353::make_read_frame(Register reg) {
  return READ_BIT | ((static_cast<uint16_t>(reg) & REGISTER_MASK) << REGISTER_SHIFT);
}

uint16_t Drv8353::make_write_frame(Register reg, uint16_t data) {
  return ((static_cast<uint16_t>(reg) & REGISTER_MASK) << REGISTER_SHIFT) | (data & DATA_MASK);
}

std::optional<uint8_t> Drv8353::gate_drive_source_code(uint16_t milliamps) {
  for (size_t i = 0; i < SOURCE_CURRENT_MILLIAMPS.size(); i++) {
    if (SOURCE_CURRENT_MILLIAMPS[i] == milliamps) {
      return static_cast<uint8_t>(i);
    }
  }
  return std::nullopt;
}

std::optional<uint8_t> Drv8353::gate_drive_sink_code(uint16_t milliamps) {
  for (size_t i = 0; i < SINK_CURRENT_MILLIAMPS.size(); i++) {
    if (SINK_CURRENT_MILLIAMPS[i] == milliamps) {
      return static_cast<uint8_t>(i);
    }
  }
  return std::nullopt;
}

Drv8353::GateDriveCurrent Drv8353::decode_gate_drive_current(uint16_t value) {
  auto source_code =
      static_cast<uint8_t>((value & GATE_DRIVE_SOURCE_MASK) >> GATE_DRIVE_SOURCE_SHIFT);
  auto sink_code = static_cast<uint8_t>(value & GATE_DRIVE_SINK_MASK);
  return {.source_milliamps = SOURCE_CURRENT_MILLIAMPS[source_code],
          .sink_milliamps = SINK_CURRENT_MILLIAMPS[sink_code]};
}

std::optional<uint16_t> Drv8353::encode_gate_drive_current(const GateDriveCurrent &current) {
  auto source_code = gate_drive_source_code(current.source_milliamps);
  auto sink_code = gate_drive_sink_code(current.sink_milliamps);
  if (!source_code || !sink_code) {
    return std::nullopt;
  }
  return (static_cast<uint16_t>(*source_code) << GATE_DRIVE_SOURCE_SHIFT) |
         static_cast<uint16_t>(*sink_code);
}

uint16_t Drv8353::with_gate_drive_lock(uint16_t value, uint16_t lock_bits) {
  return (value & ~GATE_DRIVE_LOCK_MASK) | (lock_bits & GATE_DRIVE_LOCK_MASK);
}

bool Drv8353::bit_is_set(uint16_t value, uint16_t mask) { return (value & mask) != 0; }

uint16_t Drv8353::get_field(uint16_t value, uint16_t mask, uint16_t shift) {
  return (value & mask) >> shift;
}

uint16_t Drv8353::set_bit(uint16_t value, uint16_t mask, bool enabled) {
  return (value & ~mask) | (enabled ? mask : 0u);
}

uint16_t Drv8353::set_field(uint16_t value, uint16_t mask, uint16_t shift, uint16_t field_value) {
  return (value & ~mask) | ((field_value << shift) & mask);
}

bool Drv8353::configure_gpios(std::error_code &ec) {
  if (config_.enable_gpio != GPIO_NUM_NC) {
    gpio_config_t io_conf{};
    io_conf.pin_bit_mask = 1ULL << config_.enable_gpio;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    auto err = gpio_config(&io_conf);
    if (err != ESP_OK) {
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
  }

  if (config_.fault_gpio != GPIO_NUM_NC) {
    gpio_config_t io_conf{};
    io_conf.pin_bit_mask = 1ULL << config_.fault_gpio;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    auto err = gpio_config(&io_conf);
    if (err != ESP_OK) {
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
  }

  ec.clear();
  return true;
}

bool Drv8353::transfer_frame(uint16_t tx_frame, uint16_t &rx_frame, std::error_code &ec) {
  if (config_.transfer) {
    // The DRV8353 answers in the SAME 16-bit frame as the command: data bits
    // shift out on SDO while the command shifts in. A second "dummy" frame is
    // wrong — the chip decodes 0x0000 as an access to register 0 and reports
    // FAULT_STATUS_1, so every read would return fault bits instead of the
    // requested register.
    auto tx = to_bytes(tx_frame);
    std::array<uint8_t, 2> rx = {0, 0};
    if (!config_.transfer(std::span<const uint8_t>(tx.data(), tx.size()),
                          std::span<uint8_t>(rx.data(), rx.size()))) {
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }

    rx_frame = from_bytes(rx.data());
    ec.clear();
    return true;
  }

  if (!send_frame(tx_frame, ec)) {
    return false;
  }
  if (config_.inter_frame_delay.count() > 0) {
    std::this_thread::sleep_for(config_.inter_frame_delay);
  }
  return receive_frame(rx_frame, ec);
}

bool Drv8353::send_frame(uint16_t frame, std::error_code &ec) {
  if (config_.transfer) {
    auto tx = to_bytes(frame);
    if (!config_.transfer(std::span<const uint8_t>(tx.data(), tx.size()), std::span<uint8_t>{})) {
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    ec.clear();
    return true;
  }
  auto tx = to_bytes(frame);
  write(tx.data(), tx.size(), ec);
  return !ec;
}

bool Drv8353::receive_frame(uint16_t &frame, std::error_code &ec) {
  uint8_t rx[2] = {0, 0};
  read(rx, sizeof(rx), ec);
  if (ec) {
    return false;
  }
  frame = from_bytes(rx);
  return true;
}

bool Drv8353::unlock_protected_registers(uint16_t &gate_drive_hs, uint16_t &original_lock,
                                         std::error_code &ec) {
  gate_drive_hs = read_register(Register::GATE_DRIVE_HS, ec);
  if (ec) {
    return false;
  }

  original_lock = gate_drive_hs & GATE_DRIVE_LOCK_MASK;
  if (original_lock != GATE_DRIVE_UNLOCK &&
      !write_register(Register::GATE_DRIVE_HS,
                      with_gate_drive_lock(gate_drive_hs, GATE_DRIVE_UNLOCK), ec)) {
    return false;
  }

  ec.clear();
  return true;
}

bool Drv8353::restore_protected_register_lock(uint16_t gate_drive_hs, uint16_t original_lock,
                                              std::error_code &ec) {
  auto current_lock = gate_drive_hs & GATE_DRIVE_LOCK_MASK;
  if (current_lock == original_lock) {
    ec.clear();
    return true;
  }
  return write_register(Register::GATE_DRIVE_HS, with_gate_drive_lock(gate_drive_hs, original_lock),
                        ec);
}

bool Drv8353::write_protected_register(Register reg, uint16_t data, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);

  uint16_t gate_drive_hs = 0;
  uint16_t original_lock = GATE_DRIVE_UNLOCK;
  if (!unlock_protected_registers(gate_drive_hs, original_lock, ec)) {
    return false;
  }

  if (!write_register(reg, data & DATA_MASK, ec)) {
    if (original_lock != GATE_DRIVE_UNLOCK) {
      std::error_code restore_ec;
      restore_protected_register_lock(gate_drive_hs, original_lock, restore_ec);
    }
    return false;
  }

  if (reg == Register::GATE_DRIVE_HS) {
    gate_drive_hs = data & DATA_MASK;
  }
  return restore_protected_register_lock(gate_drive_hs, original_lock, ec);
}

bool Drv8353::modify_register(Register reg, uint16_t mask, uint16_t value, std::error_code &ec,
                              bool requires_unlock) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);

  auto register_value = read_register(reg, ec);
  if (ec) {
    return false;
  }

  register_value = (register_value & ~mask) | (value & mask);
  if (reg == Register::DRIVER_CONTROL) {
    register_value &= ~DRIVER_CONTROL_CLEAR_FAULT_MASK;
  }

  if (requires_unlock) {
    return write_protected_register(reg, register_value, ec);
  }
  return write_register(reg, register_value, ec);
}
