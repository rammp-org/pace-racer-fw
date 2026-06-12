# PACE RACER BSP Example

This example demonstrates how to use the `espp::PaceRacerBoard` component to
initialize the hardware on the [PACE RACER
board](https://github.com/rammp-org/pace-racer-hardware) with one external
encoder and one BLDC motor. The example initializes the board singleton,
configures the motor, starts FOC control, and prints motor target / angle /
speed data as CSV.

## How to use example

### Hardware Required

This example requires:

1. A PACE RACER board
2. A compatible external MT6701 encoder connected to the board's encoder SPI
   interface
3. A BLDC motor connected to the board's motor outputs

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

After flashing, the example logs startup messages and then prints CSV rows in
the form:

```text
time(s), motor target, motor angle (radians), motor speed (rpm)
0.000, ...
```
