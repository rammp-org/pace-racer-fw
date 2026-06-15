#pragma once

#include <array>
#include <memory>
#include <string>
#include <system_error>
#include <vector>

#include <driver/spi_master.h>

#include "base_component.hpp"
#include "bldc_driver.hpp"
#include "bldc_motor.hpp"
#include "drv8353.hpp"
#include "gaussian.hpp"
#include "i2c.hpp"
#include "interrupt.hpp"
#include "led.hpp"
#include "lm75adp.hpp"
#include "mt6701.hpp"
#include "oneshot_adc.hpp"
#include "simple_lowpass_filter.hpp"
#include "spi.hpp"
#include "timer.hpp"

namespace espp {
/// This class acts as a board support component for the PACE RACER board.
/// It provides a high-level interface to the board's functionality.
/// The PACE RACER board is a motor driver board that can drive a BLDC motor.
/// More information about the board can be found at:
/// https://github.com/rammp-org/pace-racer-hardware
///
/// High level overview of the board:
/// - ESP32s3 module
/// - Two LEDs (blue(42) and green(41))
/// - One magnetic encoder (off-board) via SPI3
/// - One BLDC motor driver (DRV8353) via SPI2
/// - Ethernet breakout via SPI2
/// - Expansion headers for I/O / SPI / I2C / UART
/// - Current sense resistors for each motor on the A, B and C phases
///
/// The class is a singleton and can be accessed using the get() method.
///
/// \section pace_racer_board_ex1 PACE RACER Example
/// \snippet pace_racer_board_example.cpp pace-racer-board example
class PaceRacerBoard : public BaseComponent {
public:
  /// Alias for the encoder type
  using Encoder = espp::Mt6701<espp::Mt6701Interface::SSI>;

  /// Alias for the BLDC motor type
  using BldcMotor = espp::BldcMotor<espp::BldcDriver, Encoder>;

  /// Alias for the DRV8353 gate-driver control component.
  using GateDriver = espp::Drv8353;

  /// Alias for the board temperature sensors.
  using TemperatureSensor = espp::Lm75adp;

  /// Alias for the velocity filter type
  using VelocityFilter = espp::SimpleLowpassFilter;

  /// Alias for the angle filter type
  using AngleFilter = espp::SimpleLowpassFilter;

  static constexpr size_t NUM_TEMPERATURE_SENSORS = 4;
  using TemperatureReadings = std::array<float, NUM_TEMPERATURE_SENSORS>;
  using TemperatureErrors = std::array<std::error_code, NUM_TEMPERATURE_SENSORS>;
  inline static constexpr std::array<uint8_t, NUM_TEMPERATURE_SENSORS> TEMPERATURE_SENSOR_ADDRESSES{
      0x4C,
      0x4D,
      0x4E,
      0x4F,
  };

  /// @brief Access the singleton instance of the PaceRacerBoard class
  /// @return Reference to the singleton instance of the PaceRacerBoard class
  static PaceRacerBoard &get() {
    static PaceRacerBoard instance;
    return instance;
  }

  PaceRacerBoard(const PaceRacerBoard &) = delete;
  PaceRacerBoard &operator=(const PaceRacerBoard &) = delete;
  PaceRacerBoard(PaceRacerBoard &&) = delete;
  PaceRacerBoard &operator=(PaceRacerBoard &&) = delete;

  /// Get a reference to the internal I2C bus
  /// \return A reference to the internal I2C bus
  I2c &get_internal_i2c();

  /// Get a reference to the interrupts
  /// \return A reference to the interrupts
  espp::Interrupt &interrupts();

  /////////////////////////////////////////////////////////////////////////////
  // LEDs
  /////////////////////////////////////////////////////////////////////////////

  /// Get a reference to the blue LED channel (channel 0)
  /// \return A reference to the blue LED channel (channel 0)
  espp::Led::ChannelConfig &blue_led();

  /// Get a reference to the blue LED channel (channel 0)
  /// \return A reference to the blue LED channel (channel 0)
  espp::Led::ChannelConfig &led_channel0();

  /// Set the duty cycle of the blue LED
  /// \param duty The duty cycle of the blue LED (0.0 - 100.0)
  /// \note The duty cycle is a percentage of the maximum duty cycle
  ///      (which is 100.0)
  ///      0.0 is off, 100.0 is fully on
  /// \note For this function to have an effect, the LED timer must NOT be running
  ///      (i.e. stop_breathing() must be called)
  void set_blue_led_duty(float duty);

  /// Get a reference to the green LED channel (channel 1)
  /// \return A reference to the green LED channel (channel 1)
  espp::Led::ChannelConfig &green_led();

  /// Get a reference to the green LED channel (channel 1)
  /// \return A reference to the green LED channel (channel 1)
  espp::Led::ChannelConfig &led_channel1();

  /// Set the duty cycle of the green LED
  /// \param duty The duty cycle of the green LED (0.0 - 100.0)
  /// \note The duty cycle is a percentage of the maximum duty cycle
  ///      (which is 100.0)
  ///      0.0 is off, 100.0 is fully on
  /// \note For this function to have an effect, the LED timer must NOT be running
  ///      (i.e. stop_breathing() must be called)
  void set_green_led_duty(float duty);

  /// Get a reference to the LED object which controls the LEDs
  /// \return A reference to the Led object
  espp::Led &led();

  /// \brief Get a reference to the Gaussian object which is used for breathing
  ///        the LEDs.
  /// \return A reference to the Gaussian object
  espp::Gaussian &gaussian();

  /// Start breathing the LEDs
  /// \details This function starts the LED timer which will periodically update
  ///          the LED duty cycle using the gaussian to create a breathing
  ///          effect.
  void start_breathing();

  /// Stop breathing the LEDs
  /// \details This function stops the LED timer which will stop updating the
  ///         LED duty cycle. It will also set the LED duty cycle to 0,
  ///         effectively turning off the LEDs.
  void stop_breathing();

  /////////////////////////////////////////////////////////////////////////////
  // ADCs
  /////////////////////////////////////////////////////////////////////////////

  /// Get a reference to the ADC_UNIT_1 OneshotAdc object
  /// \return A reference to the ADC_UNIT_1 OneshotAdc object
  espp::OneshotAdc &adc1();

  /////////////////////////////////////////////////////////////////////////////
  // Temperature Sensors
  /////////////////////////////////////////////////////////////////////////////

  /// Initialize the four on-board LM75 temperature sensors on the internal I2C bus.
  /// \return True if all four sensors are available and initialized.
  bool init_temperature_sensors(std::error_code &ec);

  /// Return true when all four board temperature sensors have been initialized.
  bool temperature_sensors_initialized() const;

  /// Get the shared pointers for all board temperature sensors.
  const std::array<std::shared_ptr<TemperatureSensor>, NUM_TEMPERATURE_SENSORS> &
  temperature_sensors() const;

  /// Get one board temperature sensor by index [0, NUM_TEMPERATURE_SENSORS).
  std::shared_ptr<TemperatureSensor> temperature_sensor(size_t index) const;

  /// Read one board temperature in degrees Celsius.
  float board_temperature_c(size_t index, std::error_code &ec) const;

  /// Read all board temperatures in degrees Celsius.
  TemperatureReadings board_temperatures_c(TemperatureErrors &errors) const;

  /////////////////////////////////////////////////////////////////////////////
  // Motors
  /////////////////////////////////////////////////////////////////////////////

  /// Driver Configuration for the PACE RACER Motor Driver(s)
  struct DriverConfig {
    float power_supply_voltage; ///< The power supply voltage in volts
    float limit_voltage;        ///< The limit voltage in volts
  };

  /// Default configuration for the PACE RACER's BLDC motor
  const BldcMotor::Config default_motor_config{
      .num_pole_pairs = 7,
      .phase_resistance = 4.0f,
      .kv_rating = 320,
      .current_limit = 1.0f,
      .foc_type = espp::detail::FocType::SPACE_VECTOR_PWM,
      .driver = {}, // set by init_motor()
      .sensor = {}, // set by init_motor()
      .velocity_pid_config =
          {
              .kp = 0.020f,
              .ki = 0.700f,
              .kd = 0.000f,
              .integrator_min = -1.0f, // same scale as output_min (so same scale as current)
              .integrator_max = 1.0f,  // same scale as output_max (so same scale as current)
              .output_min = -1.0, // velocity pid works on current (if we have phase resistance)
              .output_max = 1.0,  // velocity pid works on current (if we have phase resistance)
          },
      .angle_pid_config =
          {
              .kp = 5.000f,
              .ki = 1.000f,
              .kd = 0.000f,
              .integrator_min = -10.0f, // same scale as output_min (so same scale as velocity)
              .integrator_max = 10.0f,  // same scale as output_max (so same scale as velocity)
              .output_min = -20.0,      // angle pid works on velocity (rad/s)
              .output_max = 20.0,       // angle pid works on velocity (rad/s)
          },
      .velocity_filter = [this](float v) { return motor_velocity_filter_(v); },
      .angle_filter = [this](float a) { return motor_angle_filter_(a); },
  };

  /// Initialize the PACE RACER's components for the BLDC motor
  /// \details This function initializes the encoder, driver, and motor. This
  ///          consists of initializing encoder, DRV8353 gate-driver control,
  ///          motor_driver, and motor.
  /// \param motor_config The motor configuration
  /// \param driver_config The driver configuration
  /// \return True if the motor was successfully initialized, false otherwise
  bool init_motor(const BldcMotor::Config &motor_config,
                  const DriverConfig &driver_config = {.power_supply_voltage = 5.0f,
                                                       .limit_voltage = 5.0f});

  /// Get a shared pointer to the DRV8353 gate-driver control component.
  std::shared_ptr<GateDriver> gate_driver();

  /// Get a shared pointer to the motor driver
  /// \return A shared pointer to the motor driver
  std::shared_ptr<espp::BldcDriver> motor_driver();

  /// Get a shared pointer to the motor
  /// \return A shared pointer to the motor
  std::shared_ptr<BldcMotor> motor();

  /// Get a reference to the motor velocity filter
  /// \return A reference to the motor velocity filter
  VelocityFilter &motor_velocity_filter();

  /// Get a reference to the motor angle filter
  /// \return A reference to the motor angle filter
  AngleFilter &motor_angle_filter();

  /////////////////////////////////////////////////////////////////////////////
  // Encoders
  /////////////////////////////////////////////////////////////////////////////

  /// Get a shared pointer to the encoder 1
  /// \return A shared pointer to the encoder 1
  std::shared_ptr<Encoder> encoder();

  /// Reset the encoder 1 accumulator
  /// \details This function resets the encoder 1 accumulator to 0.
  ///          This will reset the encoder's position to be within the range
  ///          of 0 to 2*pi.
  void reset_encoder_accumulator();

  /////////////////////////////////////////////////////////////////////////////
  // Motor Current Sense
  /////////////////////////////////////////////////////////////////////////////

  /// Get the current sense reference voltage
  /// \return The current sense reference voltage in volts
  float motor_current_sense_vref();

  /// Get the current sense value for phase A
  /// \return The current sense value for phase A in amps
  float motor_current_a_amps();

  /// Get the current sense value for phase B
  /// \return The current sense value for phase B in amps
  float motor_current_b_amps();

  /// Get the current sense value for phase C
  /// \return The current sense value for phase C in amps
  float motor_current_c_amps();

  /// Get the conversion factor used by `motor_current_*_amps()`.
  /// \return Current-sense conversion factor in amps per millivolt.
  static constexpr float motor_current_sense_mv_to_a() { return CURRENT_SENSE_MV_TO_A; }

protected:
  static constexpr auto I2C_PORT = I2C_NUM_0;
  static constexpr auto I2C_SDA_PIN = GPIO_NUM_45;
  static constexpr auto I2C_SCL_PIN = GPIO_NUM_48;

  static constexpr auto COMM_SPI_HOST = SPI2_HOST;
  static constexpr size_t COMM_SPI_MAX_TRANSFER_SIZE = 1600;
  static constexpr auto COMM_CS_PIN = GPIO_NUM_10;
  static constexpr auto COMM_RESET_PIN = GPIO_NUM_21;
  static constexpr auto COMM_IRQ_PIN = GPIO_NUM_14;

  static constexpr auto DRIVER_SPI_HOST = SPI2_HOST;
  static constexpr auto DRIVER_SPI_CLK_SPEED = 10 * 1000 * 1000; // max is 10 MHz
  static constexpr auto DRIVER_SPI_MISO_PIN = GPIO_NUM_13;
  static constexpr auto DRIVER_SPI_MOSI_PIN = GPIO_NUM_11;
  static constexpr auto DRIVER_SPI_SCLK_PIN = GPIO_NUM_12;
  static constexpr auto DRIVER_CS_PIN = GPIO_NUM_39;

  static constexpr uint64_t core_update_period_us = 1000; // 1 ms
  static constexpr auto ENCODER_SPI_HOST = SPI3_HOST;
  static constexpr auto ENCODER_SPI_CLK_SPEED = 8 * 1000 * 1000; // max is 8 MHz
  static constexpr auto ENCODER_SPI_MISO_PIN = GPIO_NUM_35;
  static constexpr auto ENCODER_SPI_SCLK_PIN = GPIO_NUM_37;
  static constexpr auto ENCODER_CS_PIN = GPIO_NUM_38;

  static constexpr auto MOTOR_FAULT_PIN = GPIO_NUM_40;
  static constexpr auto MOTOR_ENABLE_PIN = GPIO_NUM_47;

  static constexpr auto MOTOR_HALL_A_PIN = GPIO_NUM_3;
  static constexpr auto MOTOR_HALL_B_PIN = GPIO_NUM_46;
  static constexpr auto MOTOR_HALL_C_PIN = GPIO_NUM_9;

  static constexpr auto MOTOR_A_H = GPIO_NUM_8;
  static constexpr auto MOTOR_A_L = GPIO_NUM_18;
  static constexpr auto MOTOR_B_H = GPIO_NUM_17;
  static constexpr auto MOTOR_B_L = GPIO_NUM_16;
  static constexpr auto MOTOR_C_H = GPIO_NUM_15;
  static constexpr auto MOTOR_C_L = GPIO_NUM_7;

  static constexpr auto BLUE_LED_GPIO = GPIO_NUM_42;
  static constexpr auto GREEN_LED_GPIO = GPIO_NUM_41;

  // TODO: figure this out and update it :)
  static constexpr float CURRENT_SENSE_MV_TO_A = 1.0f;

  /// Constructor
  PaceRacerBoard();

  void always_init();
  void init_spi();

  float breathe(float breathing_period, uint64_t start_us, bool restart = false);

  bool read_encoder(const std::shared_ptr<Spi::Device> &encoder_device, uint8_t *data, size_t size);

  /// I2C bus for internal communication with temperature sensor, etc.
  I2c internal_i2c_{{.port = I2C_PORT,
                     .sda_io_num = I2C_SDA_PIN,
                     .scl_io_num = I2C_SCL_PIN,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  std::unique_ptr<Spi> comm_spi_;
  std::unique_ptr<Spi> encoder_spi_;
  std::shared_ptr<Spi::Device> encoder_spi_device_;

  std::array<std::shared_ptr<TemperatureSensor>, NUM_TEMPERATURE_SENSORS> temperature_sensors_{};

  // Encoders
  Encoder::Config encoder_config_{.read = [this](uint8_t *data, size_t size) -> bool {
                                    return read_encoder(encoder_spi_device_, data, size);
                                  },
                                  .update_period =
                                      std::chrono::duration<float>(core_update_period_us / 1e6f),
                                  .log_level = get_log_level()};

  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<Encoder> encoder_{(Encoder *)(nullptr)};

  // Gate-driver IC control
  std::shared_ptr<GateDriver> gate_driver_{(GateDriver *)(nullptr)};

  // Drivers
  espp::BldcDriver::Config motor_driver_config_{
      .gpio_a_h = MOTOR_A_H,
      .gpio_a_l = MOTOR_A_L,
      .gpio_b_h = MOTOR_B_H,
      .gpio_b_l = MOTOR_B_L,
      .gpio_c_h = MOTOR_C_H,
      .gpio_c_l = MOTOR_C_L,
      .gpio_enable = -1,             // pulled up, not connected
      .gpio_fault = -1,              // not connected
      .power_supply_voltage = 48.0f, // NOTE: can be replaced by user
      .limit_voltage = 48.0f,        // NOTE: can be replaced by user
      .log_level = get_log_level()};
  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<espp::BldcDriver> motor_driver_{(espp::BldcDriver *)(nullptr)};

  // Filters
  VelocityFilter motor_velocity_filter_{{.time_constant = 0.005f}};
  AngleFilter motor_angle_filter_{{.time_constant = 0.001f}};

  // Motors
  // NOTE: use explicit type of nullptr to force allocation of control block, so
  // that it can be shared even if it's nullptr;
  std::shared_ptr<BldcMotor> motor_{(BldcMotor *)(nullptr)};

  // current sense reference voltage
  espp::AdcConfig current_sense_vref_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_0,      // GPIO1
      .attenuation = ADC_ATTEN_DB_6, // NOTE: range is [0, 1.6V], so attenuation range of [0, 1.8V]
                                     // is appropriate
  };

  // current sense phase A
  espp::AdcConfig current_sense_m_a_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_3,      // GPIO4
      .attenuation = ADC_ATTEN_DB_6, // NOTE: range is [0, 1.6V], so attenuation range of [0, 1.8V]
                                     // is appropriate
  };
  // current sense phase B
  espp::AdcConfig current_sense_m_b_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_4,      // GPIO5
      .attenuation = ADC_ATTEN_DB_6, // NOTE: range is [0, 1.6V], so attenuation range of [0, 1.8V]
                                     // is appropriate
  };
  // current sense phase C
  espp::AdcConfig current_sense_m_c_ = {
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_5,      // GPIO6
      .attenuation = ADC_ATTEN_DB_6, // NOTE: range is [0, 1.6V], so attenuation range of [0, 1.8V]
                                     // is appropriate
  };

  // ADC object for reading the current sense values. We put all three phases on the same ADC unit
  // to allow for simultaneous sampling, which is important for accurate current sensing.
  espp::OneshotAdc adc_1{{
      .unit = ADC_UNIT_1,
      .channels = {current_sense_vref_, current_sense_m_a_, current_sense_m_b_, current_sense_m_c_},
  }};

  // Interrupts
  // we'll only add each interrupt pin if the initialize method is called
  espp::Interrupt interrupts_{
      {.interrupts = {},
       .task_config = {.name = "pace racer interrupts",
                       .stack_size_bytes = CONFIG_PACE_RACER_INTERRUPT_STACK_SIZE}}};

  // led
  std::vector<espp::Led::ChannelConfig> led_channels_{
      {.gpio = (int)BLUE_LED_GPIO, .channel = LEDC_CHANNEL_0, .timer = LEDC_TIMER_2},
      {.gpio = (int)GREEN_LED_GPIO, .channel = LEDC_CHANNEL_1, .timer = LEDC_TIMER_2},
  };
  espp::Led led_{espp::Led::Config{
      .timer = LEDC_TIMER_2,
      .frequency_hz = 5000,
      .channels = led_channels_,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .clock_config = LEDC_USE_RC_FAST_CLK, // to support light sleep
  }};
  espp::Gaussian gaussian_{{.gamma = 0.1f, .alpha = 1.0f, .beta = 0.5f}};
  uint64_t blue_breathe_start_us = 0;
  uint64_t green_breathe_start_us = 0;
  espp::HighResolutionTimer led_timer_{
      {.name = "PACE RACER LED Timer",
       .callback = [this]() -> void {
         led_.set_duty(led_channels_[0].channel, 100.0f * breathe(1.0f, blue_breathe_start_us));
         led_.set_duty(led_channels_[1].channel, 100.0f * breathe(1.0f, green_breathe_start_us));
       },
       .log_level = espp::Logger::Verbosity::WARN}};
};
} // namespace espp
