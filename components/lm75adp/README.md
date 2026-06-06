# LM75ADP I2C Temperature Sensor

The `Lm75adp` component provides simple APIs for communicating with an NXP
LM75ADP over I2C. It derives from `espp::BasePeripheral` and includes helpers
for reading the temperature register, the configuration register, and the
threshold registers (`THYST` and `TOS`).

## Example

The [example](./example) shows how to configure an I2C bus, instantiate the
component, and periodically read the measured temperature.
