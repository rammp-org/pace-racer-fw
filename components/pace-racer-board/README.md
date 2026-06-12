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

## Example

See the [example](./example) for a complete project that:

1. Gets the board singleton
2. Copies and adjusts `default_motor_config`
3. Calls `init_motor(...)`
4. Runs FOC control in a periodic timer
5. Streams target, angle, and speed values as CSV over the serial console
