#include <chrono>
#include <sdkconfig.h>
#include <span>
#include <thread>

#include "drv8353.hpp"
#include "spi.hpp"

using namespace std::chrono_literals;

//! [drv8353 example]
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "DRV8353 Example", .level = espp::Logger::Verbosity::INFO});

  espp::Spi spi({
      .host = SPI2_HOST,
      .sclk_io_num = static_cast<gpio_num_t>(CONFIG_EXAMPLE_SPI_SCLK_GPIO),
      .mosi_io_num = static_cast<gpio_num_t>(CONFIG_EXAMPLE_SPI_MOSI_GPIO),
      .miso_io_num = static_cast<gpio_num_t>(CONFIG_EXAMPLE_SPI_MISO_GPIO),
      .max_transfer_sz = 4,
      .log_level = espp::Logger::Verbosity::WARN,
  });

  std::error_code ec;
  auto drv_device =
      spi.add_device({.mode = 1,
                      .clock_speed_hz = CONFIG_EXAMPLE_SPI_CLOCK_SPEED,
                      .cs_io_num = static_cast<gpio_num_t>(CONFIG_EXAMPLE_SPI_CS_GPIO),
                      .queue_size = 1},
                     ec);
  if (!drv_device) {
    logger.error("DRV8353 SPI device initialization failed: {}", ec.message());
    return;
  }

  espp::Drv8353 drv({
      .write = [drv_device](const uint8_t *data, size_t length) -> bool {
        std::error_code tx_ec;
        return drv_device->write(std::span<const uint8_t>(data, length), {}, tx_ec);
      },
      .read = [drv_device](uint8_t *data, size_t length) -> bool {
        std::error_code rx_ec;
        return drv_device->read(std::span<uint8_t>(data, length), {}, rx_ec);
      },
      .enable_gpio = static_cast<gpio_num_t>(CONFIG_EXAMPLE_ENABLE_GPIO),
      .fault_gpio = static_cast<gpio_num_t>(CONFIG_EXAMPLE_FAULT_GPIO),
      .high_side_gate_drive_current =
          espp::Drv8353::GateDriveCurrent{.source_milliamps = 100, .sink_milliamps = 200},
      .low_side_gate_drive_current =
          espp::Drv8353::GateDriveCurrent{.source_milliamps = 100, .sink_milliamps = 200},
      .log_level = espp::Logger::Verbosity::WARN,
  });

  auto registers = drv.read_all_registers(ec);
  if (ec) {
    logger.error("Failed to read DRV8353 registers: {}", ec.message());
    return;
  }

  logger.info("DRV8353 driver control: 0x{:03X}", registers.driver_control);

  auto hs_gate_drive = drv.high_side_gate_drive_current(ec);
  if (ec) {
    logger.error("Failed to read high-side gate-drive current: {}", ec.message());
    return;
  }

  auto ls_gate_drive = drv.low_side_gate_drive_current(ec);
  if (ec) {
    logger.error("Failed to read low-side gate-drive current: {}", ec.message());
    return;
  }

  logger.info("Gate drive currents: HS {}mA/{}mA, LS {}mA/{}mA", hs_gate_drive.source_milliamps,
              hs_gate_drive.sink_milliamps, ls_gate_drive.source_milliamps,
              ls_gate_drive.sink_milliamps);

  while (true) {
    auto fault_status = drv.fault_status(ec);
    if (ec) {
      logger.error("Failed to read fault status: {}", ec.message());
      return;
    }

    auto vgs_status = drv.vgs_status(ec);
    if (ec) {
      logger.error("Failed to read VGS status: {}", ec.message());
      return;
    }

    logger.info("FAULT_STATUS_1=0x{:03X}, VGS_STATUS_2=0x{:03X}, nFAULT={}", fault_status.raw,
                vgs_status.raw, drv.fault_pin_active());
    std::this_thread::sleep_for(500ms);
  }
}
//! [drv8353 example]
