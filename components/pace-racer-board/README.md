# PACE RACER Board Support Package

The `pace-racer-board` component provides a board support package for the
[PACE RACER hardware platform](https://github.com/rammp-org/pace-racer-hardware).
It wraps the board peripherals behind a single `espp::PaceRacerBoard`
singleton and provides helpers for LEDs, SPI-connected peripherals, current
sense ADC channels, and BLDC motor bring-up.

## Features

1. Singleton board accessor via `espp::PaceRacerBoard::get()`
2. Blue and green LED control, including the built-in breathing animation
3. Internal I2C bus access
4. External MT6701 encoder access over SPI
5. BLDC motor driver and motor initialization through `init_motor(...)`
6. Motor current-sense ADC helpers for phases A/B/C and the reference channel

## Hardware overview

The BSP is written for the PACE RACER board described in
`include/pace-racer-board.hpp`. At a high level it expects:

1. An ESP32-S3 target
2. Two on-board LEDs
3. One external MT6701 magnetic encoder on SPI3
4. One BLDC motor stage driven through the board's motor-control circuitry
5. Board-connected current-sense signals on ADC1

## Usage

Include the component header and access the singleton:

```cpp
#include "pace-racer-board.hpp"

auto &bsp = espp::PaceRacerBoard::get();
```

The board singleton performs its always-on initialization in the constructor,
including LED breathing and encoder SPI setup. Motor-related objects are not
created until `init_motor(...)` is called.

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

## Notes

1. The component requires `esp32s3`, as declared in `CMakeLists.txt` and
   `idf_component.yml`.
2. `init_motor(...)` initializes the encoder, motor driver, and motor together.
   It returns `false` if any of those objects are already initialized or if the
   encoder initialization fails.
3. The current-sense conversion factor is still a placeholder in the
   implementation (`CURRENT_SENSE_MV_TO_A`), so current readings should be
   treated accordingly until that calibration is finalized.

## Example

See the [example](./example) for a complete project that:

1. Gets the board singleton
2. Copies and adjusts `default_motor_config`
3. Calls `init_motor(...)`
4. Runs FOC control in a periodic timer
5. Streams target, angle, and speed values as CSV over the serial console
