# DRV8353 Example

This example shows how to use the `Drv8353` component to communicate with a
DRV8353 gate driver over SPI, configure its gate-drive / OCP / CSA registers,
and periodically read the two status registers.

## How to use example

### Hardware Required

A DRV8353 connected to the selected SPI bus. The DRV8353 requires **SPI mode
1**. If `nSLEEP` is not tied high in hardware, configure `enable_gpio` in
`menuconfig`.

### Build and Flash

Configure the SPI pins with `idf.py menuconfig`, then build the project and
flash it to the board:

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with the serial port for your board.
