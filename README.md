# CS1237 adc driver component for esp-idf

## Installation

    idf.py add-dependency "larryli/cs1237"

## Getting Started

### New cs1237 device

```c
#include "cs1237.h"

cs1237_handle_t cs1237_handle;
cs1237_new(CLK_IO_PIN, DIO_IO_PIN, &cs1237_handle);
```

### Set the configuration

```c
cs1237_config_t config = {
    .ch = CS1237_CH_A,
    .pga = CS1237_PGA_2,
    .speed = CS1237_SPEED_1280HZ,
    .refo = CS1237_REFO_ON,
};
cs1237_set_config(cs1237_handle, config);
```

**Tips:** Please undefine `CONFIG_ESP_INT_WDT` or increase `CONFIG_ESP_INT_WDT_TIMEOUT_MS` to `310`ms when you use `CS1237_SPEED_10HZ`.

### Get the configuration

```c
cs1237_config_t config;
cs1237_set_config(cs1237_handle, &config);
```

### Get the adc raw data

```c
int32_t raw;
cs1237_get_raw(cs1237_handle, &raw);
```

### Calculating voltage 

```c
int vdd = 3300;
int gain = cs1237_get_gain(config.pga);
float mv = cs1237_get_voltage(raw, vdd, gain);
```

### Power down

```c
cs1237_power_down(cs1237_handle, true);
```
