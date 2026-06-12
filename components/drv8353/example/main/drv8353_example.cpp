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
      .transfer =
          [drv_device](std::span<const uint8_t> tx_data, std::span<uint8_t> rx_data) {
            std::error_code transfer_ec;
            return drv_device->transfer(tx_data, rx_data, {}, transfer_ec);
          },
      .enable_gpio = static_cast<gpio_num_t>(CONFIG_EXAMPLE_ENABLE_GPIO),
      .fault_gpio = static_cast<gpio_num_t>(CONFIG_EXAMPLE_FAULT_GPIO),
      .high_side_gate_drive_current =
          espp::Drv8353::GateDriveCurrent{.source_milliamps = 100, .sink_milliamps = 200},
      .low_side_gate_drive_current =
          espp::Drv8353::GateDriveCurrent{.source_milliamps = 100, .sink_milliamps = 200},
      .log_level = espp::Logger::Verbosity::WARN,
  });

  if (!drv.set_peak_drive_time(espp::Drv8353::PeakDriveTime::NS_2000, ec) ||
      !drv.set_dead_time(espp::Drv8353::DeadTime::NS_200, ec) ||
      !drv.set_ocp_deglitch(espp::Drv8353::OcpDeglitch::US_4, ec) ||
      !drv.set_csa_gain(espp::Drv8353::CsaGain::GAIN_10, ec) ||
      !drv.set_sense_level(espp::Drv8353::SenseLevel::V_0_50, ec)) {
    logger.error("Failed to configure DRV8353 control registers: {}", ec.message());
    return;
  }

  auto registers = drv.read_all_registers(ec);
  if (ec) {
    logger.error("Failed to read DRV8353 registers: {}", ec.message());
    return;
  }

  logger.info("DRV8353 driver control: 0x{:03X}", registers.driver_control);

  auto ocp_control = drv.read_ocp_control(ec);
  if (ec) {
    logger.error("Failed to read OCP control: {}", ec.message());
    return;
  }

  auto csa_control = drv.read_csa_control(ec);
  if (ec) {
    logger.error("Failed to read CSA control: {}", ec.message());
    return;
  }

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
  logger.info("OCP dead time={} OCP deglitch={} CSA gain={} sense level={}",
              static_cast<int>(ocp_control.dead_time()), static_cast<int>(ocp_control.deglitch()),
              static_cast<int>(csa_control.gain()), static_cast<int>(csa_control.sense_level()));

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
