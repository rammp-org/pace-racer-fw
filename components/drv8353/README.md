# DRV8353 SPI Gate Driver

The `Drv8353` component provides register-level access to a TI DRV8353 gate
driver over SPI. It derives from `espp::BasePeripheral`, uses custom raw SPI
transactions for the DRV8353 frame format, and includes helpers for hardware
enable / reset, clearing faults, gate-drive current configuration, and reading
the device status registers.

## Notes

1. The DRV8353 requires **SPI mode 1**.
2. If the board does not tie `nSLEEP` high in hardware, set `enable_gpio` so
   the component can release the driver from sleep before using SPI.
3. Gate-drive current helpers use the DRV8353 datasheet current tables. The
   supported source-current settings are
   `50, 50, 100, 150, 300, 350, 400, 450, 550, 600, 650, 700, 850, 900, 950, 1000` mA
   and the supported sink-current settings are
   `100, 100, 200, 300, 600, 700, 800, 900, 1100, 1200, 1300, 1400, 1700, 1800, 1900, 2000` mA.
   Use `Drv8353::SOURCE_CURRENT_MILLIAMPS` and `Drv8353::SINK_CURRENT_MILLIAMPS`
   when selecting values; unsupported current requests return `std::errc::invalid_argument`.

## Example

The [example](./example) shows how to configure an SPI bus and periodically
read the DRV8353 fault and gate-driver status registers after applying
datasheet-based high-side and low-side gate-drive current settings.
