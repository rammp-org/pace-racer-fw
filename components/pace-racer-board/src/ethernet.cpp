#include "pace-racer-board.hpp"

#include <driver/gpio.h>
#include <esp_eth_mac_w5500.h>
#include <esp_eth_phy_w5500.h>
#include <esp_mac.h>

using namespace espp;

bool PaceRacerBoard::ensure_netif_stack(std::error_code &ec) {
  esp_err_t err = esp_netif_init();
  if (err != ESP_OK) {
    logger_.error("Failed to initialize TCP/IP stack: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  // The default event loop is a process-wide singleton; ESP_ERR_INVALID_STATE
  // simply means someone else already created it, which is fine.
  err = esp_event_loop_create_default();
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    logger_.error("Failed to create default event loop: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  ec.clear();
  return true;
}

bool PaceRacerBoard::init_ethernet(std::error_code &ec) {
  return init_ethernet(EthernetConfig{}, ec);
}

bool PaceRacerBoard::init_ethernet(const PaceRacerBoard::EthernetConfig &config,
                                   std::error_code &ec) {
  ec.clear();
  if (eth_initialized_.load()) {
    return true;
  }

  if (!comm_spi_) {
    logger_.error("Communications SPI bus is not available for Ethernet");
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }

  if (!ensure_netif_stack(ec)) {
    return false;
  }

  // The WIZnet driver adds a GPIO ISR handler for its interrupt line but does
  // not install the GPIO ISR service itself, so ensure it is installed here.
  // ESP_ERR_INVALID_STATE means it was already installed elsewhere, which is
  // fine.
  esp_err_t isr_err = gpio_install_isr_service(0);
  if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
    logger_.error("Failed to install GPIO ISR service: {}", esp_err_to_name(isr_err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }

  esp_eth_mac_t *mac = nullptr;
  esp_eth_phy_t *phy = nullptr;
  bool handlers_registered = false;

  auto fail = [&](const char *message, esp_err_t err, std::errc code) {
    logger_.error("{}: {}", message, esp_err_to_name(err));
    if (handlers_registered) {
      esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &PaceRacerBoard::eth_event_handler);
      esp_event_handler_unregister(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                   &PaceRacerBoard::eth_got_ip_handler);
    }
    if (eth_glue_) {
      esp_eth_del_netif_glue(eth_glue_);
      eth_glue_ = nullptr;
    }
    if (eth_handle_) {
      esp_eth_driver_uninstall(eth_handle_);
      eth_handle_ = nullptr;
    }
    if (phy) {
      phy->del(phy);
    }
    if (mac) {
      mac->del(mac);
    }
    if (eth_netif_) {
      esp_netif_destroy(eth_netif_);
      eth_netif_ = nullptr;
    }
    ec = std::make_error_code(code);
    return false;
  };

  // Create the default Ethernet network interface.
  esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
  eth_netif_ = esp_netif_new(&netif_cfg);
  if (!eth_netif_) {
    logger_.error("Failed to create Ethernet netif");
    ec = std::make_error_code(std::errc::not_enough_memory);
    return false;
  }

  // The W5500 shares the communications SPI bus (already initialized) with the
  // DRV8353. The driver attaches its own device to that bus using COMM_CS_PIN.
  spi_device_interface_config_t spi_devcfg = {};
  spi_devcfg.mode = 0; // W5500 operates in SPI mode 0
  spi_devcfg.clock_speed_hz = ETH_SPI_CLOCK_SPEED_HZ;
  spi_devcfg.queue_size = ETH_SPI_QUEUE_SIZE;
  spi_devcfg.spics_io_num = COMM_CS_PIN;

  eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(comm_spi_->host(), &spi_devcfg);
  w5500_config.base.int_gpio_num = COMM_IRQ_PIN;

  eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
  mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
  if (!mac) {
    return fail("Failed to create W5500 MAC", ESP_FAIL, std::errc::io_error);
  }

  eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
  phy_config.phy_addr = ETH_PHY_ADDR;
  phy_config.reset_gpio_num = COMM_RESET_PIN;
  phy = esp_eth_phy_new_w5500(&phy_config);
  if (!phy) {
    return fail("Failed to create W5500 PHY", ESP_FAIL, std::errc::io_error);
  }

  esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
  esp_err_t err = esp_eth_driver_install(&eth_config, &eth_handle_);
  if (err != ESP_OK) {
    eth_handle_ = nullptr;
    return fail("Failed to install Ethernet driver", err, std::errc::io_error);
  }

  // The W5500 has no factory-programmed MAC address, so one must be assigned.
  MacAddress mac_addr{};
  if (config.mac_address) {
    mac_addr = *config.mac_address;
  } else {
    err = esp_read_mac(mac_addr.data(), ESP_MAC_ETH);
    if (err != ESP_OK) {
      return fail("Failed to derive Ethernet MAC address", err, std::errc::io_error);
    }
  }
  err = esp_eth_ioctl(eth_handle_, ETH_CMD_S_MAC_ADDR, mac_addr.data());
  if (err != ESP_OK) {
    return fail("Failed to set Ethernet MAC address", err, std::errc::io_error);
  }

  // Attach the driver to the TCP/IP stack.
  eth_glue_ = esp_eth_new_netif_glue(eth_handle_);
  if (!eth_glue_) {
    return fail("Failed to create Ethernet netif glue", ESP_FAIL, std::errc::not_enough_memory);
  }
  err = esp_netif_attach(eth_netif_, eth_glue_);
  if (err != ESP_OK) {
    return fail("Failed to attach Ethernet driver to netif", err, std::errc::io_error);
  }

  err = esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &PaceRacerBoard::eth_event_handler,
                                   this);
  if (err != ESP_OK) {
    return fail("Failed to register Ethernet event handler", err, std::errc::io_error);
  }
  err = esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP,
                                   &PaceRacerBoard::eth_got_ip_handler, this);
  if (err != ESP_OK) {
    esp_event_handler_unregister(ETH_EVENT, ESP_EVENT_ANY_ID, &PaceRacerBoard::eth_event_handler);
    return fail("Failed to register Ethernet IP event handler", err, std::errc::io_error);
  }
  handlers_registered = true;

  if (!config.hostname.empty()) {
    err = esp_netif_set_hostname(eth_netif_, config.hostname.c_str());
    if (err != ESP_OK) {
      logger_.warn("Failed to set Ethernet hostname '{}': {}", config.hostname,
                   esp_err_to_name(err));
    }
  }

  if (!config.use_dhcp) {
    esp_err_t stop_err = esp_netif_dhcpc_stop(eth_netif_);
    if (stop_err != ESP_OK && stop_err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED) {
      return fail("Failed to stop Ethernet DHCP client", stop_err, std::errc::io_error);
    }
    esp_netif_ip_info_t ip_info = {};
    if (esp_netif_str_to_ip4(config.static_ip.c_str(), &ip_info.ip) != ESP_OK ||
        esp_netif_str_to_ip4(config.netmask.c_str(), &ip_info.netmask) != ESP_OK) {
      return fail("Invalid static IP configuration", ESP_ERR_INVALID_ARG,
                  std::errc::invalid_argument);
    }
    if (!config.gateway.empty() &&
        esp_netif_str_to_ip4(config.gateway.c_str(), &ip_info.gw) != ESP_OK) {
      return fail("Invalid static gateway configuration", ESP_ERR_INVALID_ARG,
                  std::errc::invalid_argument);
    }
    err = esp_netif_set_ip_info(eth_netif_, &ip_info);
    if (err != ESP_OK) {
      return fail("Failed to apply static IP configuration", err, std::errc::io_error);
    }
  }

  err = esp_eth_start(eth_handle_);
  if (err != ESP_OK) {
    return fail("Failed to start Ethernet", err, std::errc::io_error);
  }

  eth_initialized_.store(true);
  logger_.info("Ethernet (W5500) initialized on SPI host {} with MAC "
               "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
               static_cast<int>(comm_spi_->host()), mac_addr[0], mac_addr[1], mac_addr[2],
               mac_addr[3], mac_addr[4], mac_addr[5]);
  return true;
}

void PaceRacerBoard::eth_event_handler(void *arg, esp_event_base_t /*event_base*/, int32_t event_id,
                                       void * /*event_data*/) {
  auto *self = static_cast<PaceRacerBoard *>(arg);
  if (!self) {
    return;
  }
  switch (event_id) {
  case ETHERNET_EVENT_CONNECTED:
    self->eth_link_up_.store(true);
    self->logger_.info("Ethernet link up");
    break;
  case ETHERNET_EVENT_DISCONNECTED:
    self->eth_link_up_.store(false);
    self->eth_has_ip_.store(false);
    self->eth_ip_addr_.store(0);
    self->logger_.warn("Ethernet link down");
    break;
  case ETHERNET_EVENT_START:
    self->logger_.info("Ethernet started");
    break;
  case ETHERNET_EVENT_STOP:
    self->eth_link_up_.store(false);
    self->eth_has_ip_.store(false);
    self->eth_ip_addr_.store(0);
    self->logger_.info("Ethernet stopped");
    break;
  default:
    break;
  }
}

void PaceRacerBoard::eth_got_ip_handler(void *arg, esp_event_base_t /*event_base*/,
                                        int32_t /*event_id*/, void *event_data) {
  auto *self = static_cast<PaceRacerBoard *>(arg);
  auto *event = static_cast<ip_event_got_ip_t *>(event_data);
  if (!self || !event) {
    return;
  }
  self->eth_ip_addr_.store(event->ip_info.ip.addr);
  self->eth_has_ip_.store(true);
  char ip_str[16] = {0};
  esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
  self->logger_.info("Ethernet got IP address: {}", ip_str);
}

bool PaceRacerBoard::ethernet_initialized() const { return eth_initialized_.load(); }

bool PaceRacerBoard::ethernet_link_up() const { return eth_link_up_.load(); }

bool PaceRacerBoard::ethernet_has_ip() const { return eth_has_ip_.load(); }

std::string PaceRacerBoard::ethernet_ip_address() const {
  esp_ip4_addr_t addr{};
  addr.addr = eth_ip_addr_.load();
  char buf[16] = {0};
  esp_ip4addr_ntoa(&addr, buf, sizeof(buf));
  return std::string(buf);
}

bool PaceRacerBoard::ethernet_mac_address(PaceRacerBoard::MacAddress &mac,
                                          std::error_code &ec) const {
  if (!eth_handle_) {
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }
  esp_err_t err = esp_eth_ioctl(eth_handle_, ETH_CMD_G_MAC_ADDR, mac.data());
  if (err != ESP_OK) {
    logger_.error("Failed to read Ethernet MAC address: {}", esp_err_to_name(err));
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }
  ec.clear();
  return true;
}

esp_eth_handle_t PaceRacerBoard::eth_handle() const { return eth_handle_; }

esp_netif_t *PaceRacerBoard::eth_netif() const { return eth_netif_; }
