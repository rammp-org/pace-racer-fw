#include "pace-racer-board.hpp"

using namespace espp;

bool PaceRacerBoard::init_ethernet(std::error_code &ec) {
  return init_ethernet(EthernetConfig{}, ec);
}

bool PaceRacerBoard::init_ethernet(const PaceRacerBoard::EthernetConfig &config,
                                   std::error_code &ec) {
  ec.clear();
  if (ethernet_ && ethernet_->is_initialized()) {
    return true;
  }

  if (!comm_spi_) {
    logger_.error("Communications SPI bus is not available for Ethernet");
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }

  // Translate the dotted-quad static-IP strings into the ip_info that
  // espp::Ethernet expects. A non-zero ip address disables its DHCP client.
  esp_netif_ip_info_t ip_info = {};
  if (!config.use_dhcp) {
    if (esp_netif_str_to_ip4(config.static_ip.c_str(), &ip_info.ip) != ESP_OK ||
        esp_netif_str_to_ip4(config.netmask.c_str(), &ip_info.netmask) != ESP_OK) {
      logger_.error("Invalid static IP configuration");
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
    if (!config.gateway.empty() &&
        esp_netif_str_to_ip4(config.gateway.c_str(), &ip_info.gw) != ESP_OK) {
      logger_.error("Invalid static gateway configuration");
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }
  }

  // Adapt the BSP's string-based got-IP callback to espp::Ethernet's
  // esp_ip4_addr_t-based one.
  espp::Ethernet::IpCallback on_got_ip{};
  if (config.on_got_ip) {
    on_got_ip = [callback = config.on_got_ip](esp_ip4_addr_t ip) {
      char ip_str[16] = {0};
      esp_ip4addr_ntoa(&ip, ip_str, sizeof(ip_str));
      callback(std::string(ip_str));
    };
  }

  // The W5500 shares the communications SPI bus (already initialized) with the
  // DRV8353; espp::Ethernet attaches its own device to that bus via COMM_CS_PIN.
  ethernet_ = std::make_unique<espp::Ethernet>(espp::Ethernet::Config{
      .interface =
          espp::Ethernet::SpiConfig{
              .host = comm_spi_->host(),
              .cs_gpio = COMM_CS_PIN,
              .int_gpio = COMM_IRQ_PIN,
              .reset_gpio = COMM_RESET_PIN,
              .clock_speed_hz = ETH_SPI_CLOCK_SPEED_HZ,
              .phy_addr = ETH_PHY_ADDR,
              .chip = espp::Ethernet::SpiChip::W5500,
          },
      .mac_address = config.mac_address,
      .hostname = config.hostname,
      .ip_info = ip_info,
      .on_link_up = config.on_link_up,
      .on_link_down = config.on_link_down,
      .on_got_ip = std::move(on_got_ip),
      .on_lost_ip = config.on_ip_lost,
      .log_level = get_log_level(),
  });

  if (!ethernet_->initialize(ec)) {
    logger_.error("Failed to initialize Ethernet: {}", ec.message());
    ethernet_.reset();
    return false;
  }

  logger_.info("Ethernet (W5500) initialized on SPI host {} with MAC {}",
               static_cast<int>(comm_spi_->host()), ethernet_->get_mac_address());
  return true;
}

bool PaceRacerBoard::ethernet_initialized() const {
  return ethernet_ && ethernet_->is_initialized();
}

bool PaceRacerBoard::ethernet_link_up() const { return ethernet_ && ethernet_->link_up(); }

bool PaceRacerBoard::ethernet_has_ip() const { return ethernet_ && ethernet_->is_connected(); }

std::string PaceRacerBoard::ethernet_ip_address() const {
  return ethernet_ ? ethernet_->get_ip_address() : std::string("0.0.0.0");
}

bool PaceRacerBoard::ethernet_mac_address(PaceRacerBoard::MacAddress &mac,
                                          std::error_code &ec) const {
  if (!ethernet_ || !ethernet_->native_handle()) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  esp_err_t err = esp_eth_ioctl(ethernet_->native_handle(), ETH_CMD_G_MAC_ADDR, mac.data());
  if (err != ESP_OK) {
    logger_.error("Failed to read Ethernet MAC address: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  ec.clear();
  return true;
}

espp::Ethernet *PaceRacerBoard::ethernet() const { return ethernet_.get(); }

esp_eth_handle_t PaceRacerBoard::eth_handle() const {
  return ethernet_ ? ethernet_->native_handle() : nullptr;
}

esp_netif_t *PaceRacerBoard::eth_netif() const { return ethernet_ ? ethernet_->netif() : nullptr; }
