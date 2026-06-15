# PACE RACER BSP Bringup

This bringup app performs a conservative, step-by-step check of the
`espp::PaceRacerBoard` subsystems and prints a summary that is easy to copy from
the serial console.

The bringup sequence:

1. Acquires the BSP singleton and switches the LEDs from the default breathing
   pattern into manual test mode
2. Exercises blue / green LED duty control
3. Samples the board current-sense ADC channels and reference node
4. Probes and reads the four on-board LM75 temperature sensors
5. Initializes the motor subsystem without enabling motor motion
6. Reads DRV8353 fault / VGS status and MT6701 encoder state
7. Prints a human-readable summary plus a CSV block containing all checks

## How to use bringup

### Hardware Required

This bringup app expects:

1. A PACE RACER board
2. A compatible external MT6701 encoder connected to the board's encoder SPI
   interface
3. Optional motor power if you want the DRV8353 UVLO / GDUV checks to clear

### Build and Flash

Build the project and flash it to the board, then run the serial monitor:

```bash
idf.py -p PORT flash monitor
```

Replace `PORT` with the serial port for your board.

## Output

The app prints:

1. Per-check log messages while bringup runs
2. A summary table with subsystem, check name, status, and details
3. A CSV block bounded by `bringup_csv_begin` / `bringup_csv_end`

Statuses are:

1. `PASS` - the check completed successfully
2. `WARN` - the check completed but reported a condition worth reviewing
3. `FAIL` - the check failed
4. `SKIP` - the check was not run because a dependency failed earlier

At the end of bringup, the LEDs indicate overall status:

1. Solid green - all checks passed
2. Solid blue - one or more checks failed
3. Solid blue + green - checks completed with warnings but no failures

## Safety notes

1. This bringup app does **not** call `motor->enable()` and does not run the
   FOC loop.
2. It **does** call `init_motor(...)`, which brings up the encoder, DRV8353,
   PWM driver, and motor objects. That leaves the DRV8353 enabled and the PWM
   outputs configured at 0% duty so status can be checked without intentionally
   commanding motion.
