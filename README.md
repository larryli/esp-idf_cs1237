# CS1237 ADC Driver Component for ESP-IDF

[![Component Registry](https://components.espressif.com/components/larryli/cs1237/badge.svg)](https://components.espressif.com/components/larryli/cs1237)

A driver component for the **CS1237** 24-bit Sigma-Delta ADC chip, designed for use with [ESP-IDF](https://github.com/espressif/esp-idf) (v5.0+).

## Features

- Read 24-bit signed ADC values
- Configure input channel, PGA gain, sampling speed, and reference output
- Power-down mode for energy saving
- Thread-safe operation (uses FreeRTOS critical sections)
- Supports ESP32, ESP32-S2, ESP32-S3, ESP32-C2, ESP32-C3, ESP32-C6

## Installation

```bash
idf.py add-dependency "larryli/cs1237"
```

## Hardware Connection

Connect the CS1237 to your ESP32:

| CS1237 Pin | ESP32 Pin | Description |
|------------|-----------|-------------|
| CLK        | Any GPIO  | Clock signal |
| DIO        | Any GPIO  | Bidirectional data signal |
| VDD        | 3.3V      | Power supply |
| GND        | GND       | Ground |

## Getting Started

### 1. Create a CS1237 Device

```c
#include "cs1237.h"

cs1237_handle_t handle;
cs1237_new(CLK_IO_PIN, DIO_IO_PIN, &handle);
```

### 2. Configure the Device

```c
cs1237_config_t config = {
    .ch = CS1237_CH_A,        // Channel A (analog input)
    .pga = CS1237_PGA_2,      // Gain = 2
    .speed = CS1237_SPEED_1280HZ, // 1280 Hz sampling rate
    .refo = CS1237_REFO_ON,   // Reference output enabled
};
cs1237_set_config(handle, config);
```

### 3. Read Configuration Back

```c
cs1237_config_t config;
cs1237_get_config(handle, &config);
```

### 4. Read ADC Values

```c
int32_t raw;
cs1237_get_raw(handle, &raw);
```

### 5. Convert to Voltage

```c
int vdd = 3300;  // 3.3V in millivolts
int gain = cs1237_get_gain(config.pga);
float mv = cs1237_get_voltage(raw, vdd, gain);
```

### 6. Check for Saturation

```c
if (cs1237_is_positive_full(raw)) {
    // Value at positive full-scale (clipped)
} else if (cs1237_is_negative_full(raw)) {
    // Value at negative full-scale (clipped)
}
```

### 7. Power Management

```c
// Enter power-down mode
cs1237_power_down(handle, true);

// Wake up from power-down mode
cs1237_power_down(handle, false);
```

### 8. Clean Up

```c
cs1237_del(handle);
```

## Configuration Options (Kconfig)

The following options can be configured via `idf.py menuconfig` under "CS1237 Adc Driver":

| Option | Description | Default |
|--------|-------------|---------|
| `CONFIG_CS1237_DELAY_US` | CLK pulse half-period delay (1-1000 us) | 1 us |
| `CONFIG_CS1237_POWER_DOWN_DELAY_US` | Delay before power-down (1-1000 us) | 100 us |
| `CONFIG_CS1237_POWER_UP_DELAY_US` | Delay after power-up (1-1000 us) | 10 us |

## Important Notes

### Interrupt Watchdog Timeout

When using `CS1237_SPEED_10HZ`, the conversion time (~300ms) may exceed the default interrupt watchdog timeout. You have two options:

1. **Disable INT_WDT** in menuconfig: `Component config → ESP System Settings → Interrupt watchdog timeout` → disable
2. **Increase INT_WDT timeout** to at least 310ms

### Voltage Calculation

The `cs1237_get_voltage()` function uses the formula:

```
V = 0.5 * raw * VDD / gain / 2^23
```

The 0.5 factor comes from the CS1237's internal reference voltage (Vref = VDD/2 when REFO is enabled).

### Sampling Speed vs Resolution

Higher sampling speeds result in lower effective resolution:

| Speed | Conversion Time | Recommended Use |
|-------|-----------------|-----------------|
| 10 Hz | ~100ms | High precision measurements |
| 40 Hz | ~25ms | General purpose (default) |
| 640 Hz | ~1.5ms | Moderate speed applications |
| 1280 Hz | ~0.8ms | High speed applications |

## API Reference

See the [header file](include/cs1237.h) for detailed API documentation.

## License

MIT License - see [LICENSE](LICENSE) file.

## Contributing

Contributions are welcome! Please open an issue or submit a pull request on [GitHub](https://github.com/larryli/esp-idf_cs1237).
