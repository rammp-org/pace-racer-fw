#include <chrono>
#include <thread>

#include "format.hpp"
#include "logger.hpp"
#include "pace-racer-board.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "PACE RACER Ethernet", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting Ethernet example");
  //! [pace-racer-ethernet example]
  using Bsp = espp::PaceRacerBoard;
  auto &bsp = Bsp::get();
  bsp.set_log_level(espp::Logger::Verbosity::INFO);

  // Configure the on-board WIZnet W5500 Ethernet interface. By default it uses
  // DHCP; set use_dhcp = false and fill in static_ip / netmask / gateway to use
  // a static address instead.
  Bsp::EthernetConfig eth_config;
  eth_config.hostname = "pace-racer";
  // eth_config.use_dhcp = false;
  // eth_config.static_ip = "192.168.1.50";
  // eth_config.netmask = "255.255.255.0";
  // eth_config.gateway = "192.168.1.1";

  std::error_code ec;
  if (!bsp.init_ethernet(eth_config, ec)) {
    logger.error("Failed to initialize Ethernet: {}", ec.message());
    return;
  }

  Bsp::MacAddress mac{};
  if (bsp.ethernet_mac_address(mac, ec)) {
    logger.info("Ethernet MAC: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", mac[0], mac[1], mac[2],
                mac[3], mac[4], mac[5]);
  }

  // Wait for the physical link to come up (i.e. a cable to be connected).
  logger.info("Waiting for Ethernet link...");
  while (!bsp.ethernet_link_up()) {
    std::this_thread::sleep_for(200ms);
  }
  logger.info("Ethernet link is up");

  // Wait for an IP address to be assigned (via DHCP, or immediately for static).
  logger.info("Waiting for IP address...");
  while (!bsp.ethernet_has_ip()) {
    std::this_thread::sleep_for(200ms);
  }
  logger.info("Ethernet ready with IP address: {}", bsp.ethernet_ip_address());

  // Periodically report link and address status.
  while (true) {
    logger.info("link_up={}, has_ip={}, ip={}", bsp.ethernet_link_up(), bsp.ethernet_has_ip(),
                bsp.ethernet_ip_address());
    std::this_thread::sleep_for(5s);
  }
  //! [pace-racer-ethernet example]
}
