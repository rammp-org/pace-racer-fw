#include "pace-racer-board.hpp"

#include <algorithm>

using namespace espp;

PaceRacerBoard::PaceRacerBoard()
    : BaseComponent("PACE RACER BSP") {
  always_init();
}

I2c &PaceRacerBoard::get_internal_i2c() { return internal_i2c_; }

espp::Interrupt &PaceRacerBoard::interrupts() { return interrupts_; }

espp::Led::ChannelConfig &PaceRacerBoard::blue_led() { return led_channels_[0]; }

espp::Led::ChannelConfig &PaceRacerBoard::led_channel0() { return led_channels_[0]; }

void PaceRacerBoard::set_blue_led_duty(float duty) {
  led_.set_duty(led_channels_[0].channel, duty);
}

espp::Led::ChannelConfig &PaceRacerBoard::green_led() { return led_channels_[1]; }

espp::Led::ChannelConfig &PaceRacerBoard::led_channel1() { return led_channels_[1]; }

void PaceRacerBoard::set_green_led_duty(float duty) {
  led_.set_duty(led_channels_[1].channel, duty);
}

espp::Led &PaceRacerBoard::led() { return led_; }

espp::Gaussian &PaceRacerBoard::gaussian() { return gaussian_; }

void PaceRacerBoard::start_breathing() {
  blue_breathe_start_us = esp_timer_get_time();
  green_breathe_start_us = esp_timer_get_time();
  static constexpr uint64_t led_timer_period_us = 30 * 1000; // 30 ms
  led_timer_.periodic(led_timer_period_us);
}

void PaceRacerBoard::stop_breathing() {
  led_timer_.stop();
  led_.set_duty(led_channels_[0].channel, 0.0f);
  led_.set_duty(led_channels_[1].channel, 0.0f);
}

espp::OneshotAdc &PaceRacerBoard::adc1() { return adc_1; }

bool PaceRacerBoard::init_temperature_sensors(std::error_code &ec) {
  if (temperature_sensors_initialized()) {
    ec.clear();
    return true;
  }

  for (auto &sensor : temperature_sensors_) {
    sensor.reset();
  }

  for (size_t i = 0; i < NUM_TEMPERATURE_SENSORS; i++) {
    auto address = TEMPERATURE_SENSOR_ADDRESSES[i];
    auto lm75_device = internal_i2c_.add_device<uint8_t>(
        {.device_address = address,
         .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
         .scl_speed_hz = internal_i2c_.config().clk_speed,
         .log_level = espp::Logger::Verbosity::WARN},
        ec);
    if (!lm75_device) {
      logger_.error("Failed to create LM75ADP device at 0x{:02X}: {}", address, ec.message());
      for (auto &sensor : temperature_sensors_) {
        sensor.reset();
      }
      return false;
    }

    auto sensor = std::make_shared<TemperatureSensor>(TemperatureSensor::Config{
        .device_address = address,
        .probe = espp::make_i2c_addressed_probe(lm75_device),
        .write = espp::make_i2c_addressed_write(lm75_device),
        .read_register = espp::make_i2c_addressed_read_register(lm75_device),
        .write_then_read = espp::make_i2c_addressed_write_then_read(lm75_device),
        .auto_init = false,
        .log_level = get_log_level(),
    });
    if (!sensor->initialize(ec)) {
      logger_.error("Failed to initialize LM75ADP at 0x{:02X}: {}", address, ec.message());
      for (auto &temperature_sensor : temperature_sensors_) {
        temperature_sensor.reset();
      }
      return false;
    }
    temperature_sensors_[i] = std::move(sensor);
  }

  ec.clear();
  return true;
}

bool PaceRacerBoard::temperature_sensors_initialized() const {
  return std::all_of(temperature_sensors_.begin(), temperature_sensors_.end(),
                     [](const auto &sensor) { return static_cast<bool>(sensor); });
}

const std::array<std::shared_ptr<PaceRacerBoard::TemperatureSensor>,
                 PaceRacerBoard::NUM_TEMPERATURE_SENSORS> &
PaceRacerBoard::temperature_sensors() const {
  return temperature_sensors_;
}

std::shared_ptr<PaceRacerBoard::TemperatureSensor>
PaceRacerBoard::temperature_sensor(size_t index) const {
  if (index >= temperature_sensors_.size()) {
    return nullptr;
  }
  return temperature_sensors_[index];
}

float PaceRacerBoard::board_temperature_c(size_t index, std::error_code &ec) const {
  if (index >= temperature_sensors_.size()) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return 0.0f;
  }
  auto sensor = temperature_sensors_[index];
  if (!sensor) {
    ec = std::make_error_code(std::errc::no_such_device);
    return 0.0f;
  }
  return sensor->temperature_c(ec);
}

PaceRacerBoard::TemperatureReadings
PaceRacerBoard::board_temperatures_c(PaceRacerBoard::TemperatureErrors &errors) const {
  TemperatureReadings readings{};
  for (size_t i = 0; i < temperature_sensors_.size(); i++) {
    errors[i].clear();
    readings[i] = board_temperature_c(i, errors[i]);
  }
  return readings;
}

bool PaceRacerBoard::init_motor(const PaceRacerBoard::BldcMotor::Config &motor_config) {
  return init_motor(motor_config,
                    DriverConfig{.power_supply_voltage = 5.0f, .limit_voltage = 5.0f});
}

bool PaceRacerBoard::init_motor(const PaceRacerBoard::BldcMotor::Config &motor_config,
                                const PaceRacerBoard::DriverConfig &driver_config) {
  if (motor_ || motor_driver_ || encoder_ || gate_driver_) {
    logger_.error("Motor subsystem already initialized");
    return false;
  }

  // Reject an out-of-range interrupt priority here rather than letting it fail
  // deeper in the driver stack, where the error is harder to trace back to
  // this call site. Levels 4-7 require assembly-only handlers.
  if (driver_config.intr_priority < 0 || driver_config.intr_priority > 3) {
    logger_.error("Invalid MCPWM timer interrupt priority {}; valid range is [0, 3]",
                  driver_config.intr_priority);
    return false;
  }

  auto cleanup_motor_subsystem = [this]() {
    if (gate_driver_) {
      std::error_code disable_ec;
      gate_driver_->set_enabled(false, disable_ec);
      if (disable_ec) {
        logger_.warn("Failed to disable DRV8353 during cleanup: {}", disable_ec.message());
      }
    }
    motor_.reset();
    motor_driver_.reset();
    gate_driver_.reset();
    encoder_.reset();
  };

  bool run_task = true;
  std::error_code ec;
  encoder_ = std::make_shared<Encoder>(encoder_config_);
  encoder_->initialize(run_task, ec);
  if (ec) {
    logger_.error("Could not initialize encoder: {}", ec.message());
    cleanup_motor_subsystem();
    return false;
  }

  if (!comm_spi_) {
    logger_.error("Communications SPI bus is not available");
    cleanup_motor_subsystem();
    return false;
  }

  auto gate_driver_device = comm_spi_->add_device({.mode = 1,
                                                   .clock_speed_hz = DRIVER_SPI_CLK_SPEED,
                                                   .cs_io_num = DRIVER_CS_PIN,
                                                   .queue_size = 1},
                                                  ec);
  if (!gate_driver_device) {
    logger_.error("Failed to initialize DRV8353 SPI device: {}", ec.message());
    cleanup_motor_subsystem();
    return false;
  }

  gate_driver_ = std::make_shared<GateDriver>(GateDriver::Config{
      .transfer =
          [gate_driver_device](std::span<const uint8_t> tx_data, std::span<uint8_t> rx_data) {
            std::error_code transfer_ec;
            return gate_driver_device->transfer(tx_data, rx_data, {}, transfer_ec);
          },
      .enable_gpio = MOTOR_ENABLE_PIN,
      .fault_gpio = MOTOR_FAULT_PIN,
      .auto_init = false,
      .log_level = get_log_level(),
  });
  if (!gate_driver_->initialize(ec)) {
    logger_.error("Could not initialize DRV8353: {}", ec.message());
    cleanup_motor_subsystem();
    return false;
  }

  motor_driver_config_.power_supply_voltage = driver_config.power_supply_voltage;
  motor_driver_config_.limit_voltage = driver_config.limit_voltage;
  // The MCPWM timer is created inside BldcDriver, so its interrupt priority can
  // only be set here — an application that samples from that timer's callback
  // has no other way to reach it.
  motor_driver_config_.intr_priority = driver_config.intr_priority;
  motor_driver_ = std::make_shared<BldcDriver>(motor_driver_config_);

  auto configured_motor = motor_config;
  configured_motor.driver = motor_driver_;
  configured_motor.sensor = encoder_;
  motor_ = std::make_shared<BldcMotor>(configured_motor);
  motor_->initialize();

  return true;
}

std::shared_ptr<PaceRacerBoard::GateDriver> PaceRacerBoard::gate_driver() { return gate_driver_; }

std::shared_ptr<PaceRacerBoard::Encoder> PaceRacerBoard::encoder() { return encoder_; }

void PaceRacerBoard::reset_encoder_accumulator() {
  if (!encoder_) {
    logger_.error("Cannot reset encoder accumulator: encoder not initialized");
    return;
  }
  encoder_->reset_accumulator();
}

std::shared_ptr<espp::BldcDriver> PaceRacerBoard::motor_driver() { return motor_driver_; }

std::shared_ptr<PaceRacerBoard::BldcMotor> PaceRacerBoard::motor() { return motor_; }

PaceRacerBoard::VelocityFilter &PaceRacerBoard::motor_velocity_filter() {
  return motor_velocity_filter_;
}

PaceRacerBoard::AngleFilter &PaceRacerBoard::motor_angle_filter() { return motor_angle_filter_; }

// Oversample to average out switching transients from asynchronous ADC reads.
// Proper fix is MCPWM-triggered center-aligned sampling; this is the pragmatic alternative.
static constexpr int kCurrentOversample = 8;

static float read_avg(espp::OneshotAdc &adc, const espp::AdcConfig &ch) {
  float sum = 0;
  for (int i = 0; i < kCurrentOversample; i++) sum += adc.read_mv(ch).value();
  return sum / kCurrentOversample;
}

float PaceRacerBoard::motor_current_a_amps() {
  return (read_avg(adc_1, current_sense_m_a_) - motor_current_sense_vref()) * CURRENT_SENSE_MV_TO_A;
}

float PaceRacerBoard::motor_current_b_amps() {
  return (read_avg(adc_1, current_sense_m_b_) - motor_current_sense_vref()) * CURRENT_SENSE_MV_TO_A;
}

float PaceRacerBoard::motor_current_c_amps() {
  return (read_avg(adc_1, current_sense_m_c_) - motor_current_sense_vref()) * CURRENT_SENSE_MV_TO_A;
}

float PaceRacerBoard::motor_current_sense_vref() {
  return read_avg(adc_1, current_sense_vref_);
}

void PaceRacerBoard::always_init() {
  start_breathing();
  init_spi();
}

void PaceRacerBoard::init_spi() {
  comm_spi_ = std::make_unique<Spi>(Spi::Config{
      .host = COMM_SPI_HOST,
      .sclk_io_num = DRIVER_SPI_SCLK_PIN,
      .mosi_io_num = DRIVER_SPI_MOSI_PIN,
      .miso_io_num = DRIVER_SPI_MISO_PIN,
      .max_transfer_sz = COMM_SPI_MAX_TRANSFER_SIZE,
      .log_level = get_log_level(),
  });
  encoder_spi_ = std::make_unique<Spi>(Spi::Config{
      .host = ENCODER_SPI_HOST,
      .sclk_io_num = ENCODER_SPI_SCLK_PIN,
      .mosi_io_num = GPIO_NUM_NC,
      .miso_io_num = ENCODER_SPI_MISO_PIN,
      .max_transfer_sz = 100,
      .log_level = get_log_level(),
  });
  std::error_code ec;
  encoder_spi_device_ = encoder_spi_->add_device(
      Spi::DeviceConfig{
          .mode = 0,
          .clock_speed_hz = ENCODER_SPI_CLK_SPEED,
          .cs_io_num = ENCODER_CS_PIN,
          .queue_size = 1,
      },
      ec);
  if (ec || !encoder_spi_device_) {
    logger_.error("Failed to initialize Encoder SPI device: {}", ec.message());
    return;
  }
}

float PaceRacerBoard::breathe(float breathing_period, uint64_t start_us, bool restart) {
  auto now_us = esp_timer_get_time();
  if (restart) {
    start_us = now_us;
  }
  auto elapsed_us = now_us - start_us;
  float elapsed = elapsed_us / 1e6f;
  float t = std::fmod(elapsed, breathing_period) / breathing_period;
  return gaussian_(t);
}

bool IRAM_ATTR PaceRacerBoard::read_encoder(const std::shared_ptr<Spi::Device> &encoder_device,
                                            uint8_t *data, size_t size) {
  if (!encoder_device) {
    return false;
  }
  std::error_code ec;
  return encoder_device->read(std::span<uint8_t>(data, size), {}, ec);
}
