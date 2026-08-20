# PACE RACER Board Support Package

The `pace-racer-board` component provides a board support package for the
[PACE RACER hardware platform](https://github.com/rammp-org/pace-racer-hardware).
It wraps the board peripherals behind a single `espp::PaceRacerBoard`
singleton and provides helpers for LEDs, SPI-connected peripherals, current
sense ADC channels, on-board temperature sensing, and BLDC motor bring-up.

## Features

1. Singleton board accessor via `espp::PaceRacerBoard::get()`
2. Blue and green LED control, including the built-in breathing animation
3. Internal I2C bus access
4. External MT6701 encoder access over SPI
5. Four on-board LM75ADP temperature sensors on the internal I2C bus
6. DRV8353 gate-driver control exposure for motor-driver configuration
7. BLDC motor driver and motor initialization through `init_motor(...)`
8. Motor current-sense ADC helpers for phases A/B/C and the reference channel
9. WIZnet W5500 Ethernet bring-up over the SPI expansion header through
   `init_ethernet(...)`, with link / IP status helpers

## Hardware overview

The BSP is written for the PACE RACER board described in
`include/pace-racer-board.hpp`. At a high level it expects:

1. An ESP32-S3 target
2. Two on-board LEDs
3. One external MT6701 magnetic encoder on SPI3
4. Four LM75ADP temperature sensors on I2C addresses `0x4C` through `0x4F`
5. One DRV8353 gate driver on SPI2
6. One BLDC motor stage driven through the board's motor-control circuitry
7. Board-connected current-sense signals on ADC1
8. One WIZnet W5500 Ethernet breakout on the SPI expansion header, sharing the
   SPI2 communications bus with the DRV8353 (CS on IO10, reset on IO21, and
   interrupt on IO14)

## Usage

Include the component header and access the singleton:

```cpp
#include "pace-racer-board.hpp"

auto &bsp = espp::PaceRacerBoard::get();
```

The board singleton performs its always-on initialization in the constructor,
including LED breathing and SPI bus setup. Temperature sensors are initialized
on demand through `init_temperature_sensors(...)`. Motor-related objects are
not created until `init_motor(...)` is called.

Typical motor bring-up looks like:

```cpp
using Bsp = espp::PaceRacerBoard;
auto &bsp = Bsp::get();

auto motor_config = bsp.default_motor_config;
motor_config.phase_resistance = 4.0f;
motor_config.current_limit = 1.0f;

if (!bsp.init_motor(motor_config)) {
  return;
}

auto motor = bsp.motor();
motor->enable();
```

Temperature-sensor bring-up looks like:

```cpp
std::error_code ec;
if (!bsp.init_temperature_sensors(ec)) {
  return;
}

Bsp::TemperatureErrors errors;
auto temperatures_c = bsp.board_temperatures_c(errors);
```

Ethernet bring-up (WIZnet W5500 over the SPI expansion header) looks like:

```cpp
std::error_code ec;
if (!bsp.init_ethernet(ec)) { // DHCP by default
  return;
}

// wait for a cable / link and a DHCP-assigned address
while (!bsp.ethernet_has_ip()) {
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}
auto ip = bsp.ethernet_ip_address();
```

Pass an `EthernetConfig` to `init_ethernet(...)` to set a hostname, supply a
specific MAC address, or use a static IP instead of DHCP.

To react to link and address changes without polling, provide callbacks in the
`EthernetConfig`:

```cpp
Bsp::EthernetConfig eth_config;
eth_config.on_link_up = []() { /* link came up */ };
eth_config.on_link_down = []() { /* link went down */ };
eth_config.on_got_ip = [](const std::string &ip) { /* start network services */ };
eth_config.on_ip_lost = []() { /* pause network services */ };
bsp.init_ethernet(eth_config, ec);
```

The callbacks run in the ESP-IDF event-loop task context, so they must return
quickly and must not block; use them to set a flag, notify a task, or post to a
queue.

## Notes

1. The component requires `esp32s3`, as declared in `CMakeLists.txt` and
   `idf_component.yml`.
2. `init_motor(...)` initializes the encoder, DRV8353 gate-driver control
   object, BLDC motor driver, and motor together.
3. `init_temperature_sensors(...)` probes and initializes the four LM75ADP
   devices at `0x4C` through `0x4F`.
4. `gate_driver()` exposes the DRV8353 component so application code can adjust
   gate-drive, OCP, CSA, and related register settings after motor bring-up.
5. `board_temperatures_c(...)` reports per-sensor errors so application code
   can tell which thermal zone failed to read.
6. The current-sense conversion factor is still a placeholder in the
   implementation (`CURRENT_SENSE_MV_TO_A`), so current readings should be
   treated accordingly until that calibration is finalized.
7. `init_ethernet(...)` brings up the W5500 on the shared SPI2 communications
   bus via the [espp::Ethernet](https://github.com/esp-cpp/espp/tree/main/components/ethernet)
   component. Because the W5500 has no factory-burned MAC,
   a locally-administered address is derived from the ESP32-S3 eFuse unless one
   is supplied in `EthernetConfig`. Link and address availability are reported
   asynchronously via `ethernet_link_up()`, `ethernet_has_ip()`, and
   `ethernet_ip_address()`, or via the optional `on_link_up` / `on_link_down` /
   `on_got_ip` / `on_ip_lost` callbacks in `EthernetConfig` so the application
   can react without polling. The W5500 SPI clock and default hostname are
   configurable through Kconfig (`PACE_RACER_ETH_SPI_CLOCK_MHZ`,
   `PACE_RACER_ETH_HOSTNAME`).

## Examples

See the [bringup](./bringup) project for a conservative board-check executable
that:

1. Exercises LED control
2. Samples the current-sense ADC channels
3. Probes and reads the four LM75ADP temperature sensors
4. Initializes the motor subsystem without enabling motion
5. Reads DRV8353 and encoder status
6. Prints a pass / warn / fail summary as CSV

See the [example](./example) project for a more active motor-control demo that:

1. Gets the board singleton
2. Copies and adjusts `default_motor_config`
3. Calls `init_motor(...)`
4. Runs FOC control in a periodic timer
5. Streams target, angle, and speed values as CSV over the serial console

See the [ethernet](./ethernet) project for a focused Ethernet demo that:

1. Gets the board singleton
2. Calls `init_ethernet(...)` to bring up the W5500 interface
3. Prints the interface MAC address
4. Waits for link and a DHCP-assigned IP address
5. Periodically reports link and address status
