#include "pace-racer-board.hpp"

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

bool PaceRacerBoard::init_motor(const PaceRacerBoard::BldcMotor::Config &motor_config,
                                const PaceRacerBoard::DriverConfig &driver_config) {
  if (motor_ || motor_driver_ || encoder_) {
    logger_.error("Motor, driver, or encoder already initialized");
    return false;
  }

  bool run_task = true;
  std::error_code ec;
  // make the encoder
  encoder_ = std::make_shared<Encoder>(encoder_config_);
  // initialize the encoder
  encoder_->initialize(run_task, ec);
  if (ec) {
    logger_.error("Could not initialize encoder: {}", ec.message());
    encoder_.reset();
    return false;
  }

  // copy the config data for the driver
  motor_driver_config_.power_supply_voltage = driver_config.power_supply_voltage;
  motor_driver_config_.limit_voltage = driver_config.limit_voltage;
  // make the driver
  motor_driver_ = std::make_shared<BldcDriver>(motor_driver_config_);

  // now copy the relevant configs into the motor config
  auto _motor_config = motor_config;
  _motor_config.driver = motor_driver_;
  _motor_config.sensor = encoder_;
  // now make the motor
  motor_ = std::make_shared<BldcMotor>(_motor_config);
  motor_->initialize();

  return true;
}

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

espp::OneshotAdc &PaceRacerBoard::adc1() { return adc_1; }

float PaceRacerBoard::motor_current_a_amps() {
  return adc_1.read_mv(current_sense_m_a_).value() * CURRENT_SENSE_MV_TO_A;
}

float PaceRacerBoard::motor_current_b_amps() {
  return adc_1.read_mv(current_sense_m_b_).value() * CURRENT_SENSE_MV_TO_A;
}

float PaceRacerBoard::motor_current_c_amps() {
  return adc_1.read_mv(current_sense_m_c_).value() * CURRENT_SENSE_MV_TO_A;
}

float PaceRacerBoard::motor_current_sense_vref() {
  return adc_1.read_mv(current_sense_vref_).value();
}

void PaceRacerBoard::always_init() {
  start_breathing();
  init_spi();
}

void PaceRacerBoard::init_spi() {
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
