# PACE RACER Ethernet Example

This example demonstrates bringing up the on-board WIZnet W5500 Ethernet
interface on the PACE RACER board and waiting for a network connection.

The W5500 breakout is connected to the PACE RACER SPI expansion header, which
shares the high-speed communications SPI bus (SPI2) with the DRV8353 gate
driver. The board support package (`espp::PaceRacerBoard`) installs the
ESP-IDF `esp_eth` W5500 driver on that shared bus, using the following board
signals:

| Function      | ESP32-S3 GPIO |
| ------------- | ------------- |
| SPI clock     | IO12 (shared) |
| SPI MOSI      | IO11 (shared) |
| SPI MISO      | IO13 (shared) |
| Chip select   | IO10          |
| Reset         | IO21          |
| Interrupt     | IO14          |

## What it does

1. Gets the board singleton.
2. Registers `on_link_up` / `on_link_down` / `on_got_ip` / `on_ip_lost`
   callbacks so the application reacts to connectivity changes without polling.
3. Calls `init_ethernet(...)` to install and start the W5500 driver.
4. Prints the assigned MAC address (derived from the ESP32-S3 eFuse if not
   explicitly configured, since the W5500 has no factory-burned MAC).
5. Idles; the callbacks log link and IP changes as they happen.

The callbacks run in the ESP-IDF event-loop task context, so they must return
quickly and must not block. Use them to set a flag, notify a task, or post to a
queue rather than doing heavy work inline.

If you prefer to poll instead, the `ethernet_link_up()`, `ethernet_has_ip()`,
and `ethernet_ip_address()` accessors remain available.

## Static IP

By default the interface uses DHCP. To use a static address, set
`use_dhcp = false` and fill in the `static_ip`, `netmask`, and `gateway` fields
of `EthernetConfig` before calling `init_ethernet(...)`:

```cpp
Bsp::EthernetConfig eth_config;
eth_config.use_dhcp = false;
eth_config.static_ip = "192.168.1.50";
eth_config.netmask = "255.255.255.0";
eth_config.gateway = "192.168.1.1";
```

## Build and flash

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use. To exit the serial
monitor, type `Ctrl-]`.)
