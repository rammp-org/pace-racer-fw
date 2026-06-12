# PACE RACER Firmware

This repository contains the firmware for the PACE RACER, an open source
field-oriented control (FOC) BLDC motor controller built on top of ESPP for the
ESP32-S3. The firmware is designed to be modular and extensible, allowing for
easy customization and addition of new features.

The electronics design can be found here:

[PACE RACER GitHub Repository](https://github.com/rammp-org/pace-racer-hardware)

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [PACE RACER Firmware](#pace-racer-firmware)
  - [Functionality](#functionality)
  - [Development](#development)
    - [Environment](#environment)
    - [Build and Flash](#build-and-flash)
  - [Output](#output)
  - [Contributing](#contributing)
    - [Code style](#code-style)

<!-- markdown-toc end -->

## Functionality

This firmware provides the following functionality:
- Field-oriented control (FOC) for BLDC motors, allowing for efficient and precise
  control of motor speed and torque.
- I2C Board temperature sensor (x4) reading and reporting.
- Ethernet communication for receiving commands and sending telemetry data.
- SPI driver for controlling the motor driver IC
- High-resolution low-side per-phase current sensing for accurate current
  control and monitoring.

## Development

If you wish to modify / recompile the code, you will need to set up your
development environment to be able to build and flash your target hardware.

### Environment

This project is an ESP-IDF project, currently [ESP-IDF
v.6.0](https://github.com/espressif/esp-idf).

For information about setting up `ESP-IDF v6.0`, please see [the official
ESP-IDF getting started
documentation](https://docs.espressif.com/projects/esp-idf/en/v6.0/esp32s3/get-started/index.html).

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Output

Example screenshot of the console output from this app:

![CleanShot 2023-07-12 at 14 01 21](https://github.com/esp-cpp/template/assets/213467/7f8abeae-121b-4679-86d8-7214a76f1b75)

## Contributing

If you're developing code to contribute to this repository, it's recommended to
configure your development environment:

### Code style

1. Ensure `clang-format` is installed
2. Ensure [pre-commit](https://pre-commit.com) is installed
3. Set up `pre-commit` for this repository:

  ``` console
  pre-commit install
  ```

This helps ensure that consistent code formatting is applied, by running
`clang-format` each time you change the code (via a git pre-commit hook) using
the [./.clang-format](./.clang-format) code style configuration file.

If you ever want to re-run the code formatting on all files in the repository,
you can do so:

``` console
pre-commit run --all-files
```
