| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- |

# Get Started Example for CS1237 ADC Driver

This example demonstrates basic usage of the CS1237 ADC driver component.

## Overview

The example performs the following operations:

1. Initializes the CS1237 on GPIO 18 (DIO) and GPIO 19 (CLK)
2. Configures: Channel A, PGA=2, Speed=1280Hz, Reference=ON
3. Reads and logs the configuration back
4. Enters a loop that:
   - Reads raw ADC values
   - Converts to voltage (assuming 3.3V VDD)
   - Detects saturation conditions
   - Demonstrates power-down mode

## How to Use

### Hardware Required

- ESP32 development board
- CS1237 ADC module
- Connect CLK to GPIO 19, DIO to GPIO 18

### Build and Flash

```bash
idf.py set-target <your-target>  # e.g., esp32, esp32c3, etc.
idf.py menuconfig                 # Optional: configure CS1237 settings
idf.py build
idf.py flash monitor
```

### Expected Output

```
I (xxx) app_main: start
I (xxx) app_main: Set config: 0x36
I (xxx) app_main: Get config: 0x36
I (xxx) app_main: Adc raw data: 0x001234, value: 4660, voltage:   94.50mV
...
```

## Configuration

You can adjust CS1237 settings in `main.c`:

```c
cs1237_config_t config = {
    .ch = CS1237_CH_A,           // Channel A, Temperature, or Short
    .pga = CS1237_PGA_2,         // Gain: 1, 2, 64, or 128
    .speed = CS1237_SPEED_1280HZ, // Speed: 10, 40, 640, or 1280 Hz
    .refo = CS1237_REFO_ON,      // Reference output: ON or OFF
};
```

## Troubleshooting

- **"wait dio ready failed"**: Check wiring, ensure CS1237 is powered
- **Saturation warnings**: Input voltage may exceed VDD/GND
- **INT_WDT errors**: Disable watchdog or increase timeout for 10Hz mode
