#include <chrono>
#include <sdkconfig.h>
#include <thread>

#include "i2c.hpp"
#include "lm75adp.hpp"

using namespace std::chrono_literals;

//! [lm75adp example]
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "LM75ADP Example", .level = espp::Logger::Verbosity::INFO});

  espp::I2c i2c({
      .port = I2C_NUM_0,
      .sda_io_num = static_cast<gpio_num_t>(CONFIG_EXAMPLE_I2C_SDA_GPIO),
      .scl_io_num = static_cast<gpio_num_t>(CONFIG_EXAMPLE_I2C_SCL_GPIO),
      .clk_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ,
  });

  std::error_code ec;
  auto lm75_device =
      i2c.add_device<uint8_t>({.device_address = CONFIG_EXAMPLE_I2C_ADDRESS,
                               .timeout_ms = static_cast<int>(i2c.config().timeout_ms),
                               .scl_speed_hz = i2c.config().clk_speed,
                               .log_level = espp::Logger::Verbosity::WARN},
                              ec);
  if (!lm75_device) {
    logger.error("LM75ADP I2C device initialization failed: {}", ec.message());
    return;
  }

  espp::Lm75adp lm75({
      .device_address = static_cast<uint8_t>(CONFIG_EXAMPLE_I2C_ADDRESS),
      .probe = espp::make_i2c_addressed_probe(lm75_device),
      .write = espp::make_i2c_addressed_write(lm75_device),
      .read_register = espp::make_i2c_addressed_read_register(lm75_device),
      .write_then_read = espp::make_i2c_addressed_write_then_read(lm75_device),
      .log_level = espp::Logger::Verbosity::WARN,
  });

  auto configuration = lm75.configuration(ec);
  if (ec) {
    logger.error("Failed to read LM75ADP configuration: {}", ec.message());
    return;
  }

  logger.info("LM75ADP configuration register: 0x{:02X}", configuration.raw());

  while (true) {
    auto temperature_c = lm75.temperature_c(ec);
    if (ec) {
      logger.error("Failed to read temperature: {}", ec.message());
      return;
    }

    logger.info("Temperature: {:.3f} C", temperature_c);
    std::this_thread::sleep_for(500ms);
  }
}
//! [lm75adp example]
