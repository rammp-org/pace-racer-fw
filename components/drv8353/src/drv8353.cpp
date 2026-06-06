#include "drv8353.hpp"

#include <thread>

using namespace espp;

Drv8353::Drv8353(const Config &config)
    : BasePeripheral({.write = config.write, .read = config.read}, "Drv8353", config.log_level)
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

  if (!write_register(Register::DRIVER_CONTROL, driver_control | CLEAR_FAULT_MASK, ec)) {
    return false;
  }
  return write_register(Register::DRIVER_CONTROL, driver_control & ~CLEAR_FAULT_MASK, ec);
}

uint16_t Drv8353::read_register(Register reg, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (!send_frame(make_read_frame(reg), ec)) {
    return 0;
  }
  if (config_.inter_frame_delay.count() > 0) {
    std::this_thread::sleep_for(config_.inter_frame_delay);
  }

  uint16_t response = 0;
  if (!receive_frame(response, ec)) {
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
  return write_register(Register::GATE_DRIVE_HS, gate_drive_hs, ec);
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
  return write_register(Register::GATE_DRIVE_LS, gate_drive_ls, ec);
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
  return write_register(Register::DRIVER_CONTROL, values.driver_control, ec) &&
         write_register(Register::GATE_DRIVE_HS, values.gate_drive_hs, ec) &&
         write_register(Register::GATE_DRIVE_LS, values.gate_drive_ls, ec) &&
         write_register(Register::OCP_CONTROL, values.ocp_control, ec) &&
         write_register(Register::CSA_CONTROL, values.csa_control, ec) &&
         write_register(Register::DRIVER_CONFIGURATION, values.driver_configuration, ec);
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

bool Drv8353::send_frame(uint16_t frame, std::error_code &ec) {
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
