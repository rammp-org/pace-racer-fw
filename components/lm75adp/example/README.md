# LM75ADP Example

This example shows how to use the `Lm75adp` component to communicate with an
LM75ADP over I2C and periodically print the measured temperature.

## How to use example

### Hardware Required

An LM75ADP connected to the selected I2C port.

### Build and Flash

Configure the I2C pins with `idf.py menuconfig`, then build the project and
flash it to the board:

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with the serial port for your board.
