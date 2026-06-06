# PACE RACER BSP Example

This example demonstrates how to use the `espp::PaceRacerBoard` component to
initialize the hardware on the [PACE RACER
board](https://github.com/rammp-org/pace-racer-hardware) which is connected to
one encoder and one BLDC motor. It uses those hardware to drive the motor and
outputs the state as a CSV.

## How to use example

### Hardware Required

This example requires a PACE RACER board, one Encoder boards, and one motor. The
PACE RACER board should be connected to the Encoder boards and the motor.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output
