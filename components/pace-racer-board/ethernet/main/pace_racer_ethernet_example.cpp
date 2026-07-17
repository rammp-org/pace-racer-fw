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

  // Register application callbacks so other parts of the system can react to
  // link and IP changes without polling. These run in the ESP-IDF event-loop
  // task context, so keep them short and non-blocking (e.g. set a flag, notify
  // a task, or post to a queue).
  eth_config.on_link_up = [&logger]() { logger.info("[callback] Ethernet link is up"); };
  eth_config.on_link_down = [&logger]() { logger.warn("[callback] Ethernet link is down"); };
  eth_config.on_got_ip = [&logger](const std::string &ip) {
    logger.info("[callback] Ethernet ready with IP address: {}", ip);
    // This is where an application would start network services (e.g. bring up
    // a socket, connect to a broker, start telemetry) now that the link is up.
  };
  eth_config.on_ip_lost = [&logger]() {
    logger.warn("[callback] Ethernet lost its IP address");
    // This is where an application would tear down / pause network services
    // until connectivity is restored.
  };

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

  // Nothing else to do here: the callbacks above drive the application's
  // reaction to link and IP events, so there is no need to poll. The status
  // accessors (ethernet_link_up(), ethernet_has_ip(), ethernet_ip_address())
  // remain available if a snapshot is needed.
  logger.info("Ethernet initialized; waiting for link/IP events via callbacks");
  while (true) {
    std::this_thread::sleep_for(5s);
  }
  //! [pace-racer-ethernet example]
}
