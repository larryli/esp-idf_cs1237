#pragma once

#include "esp_err.h"
#include "hal/gpio_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct cs1237_t *cs1237_handle_t;

enum {
    CS1237_CH_A = 0,
    CS1237_CH_RESERVED = 1,
    CS1237_CH_TEMP = 2,
    CS1237_CH_SHORT = 3,
};

enum {
    CS1237_PGA_1 = 0,
    CS1237_PGA_2 = 1,
    CS1237_PGA_64 = 2,
    CS1237_PGA_128 = 3,
};

enum {
    CS1237_SPEED_10HZ = 0,
    CS1237_SPEED_40HZ = 1,
    CS1237_SPEED_640HZ = 2,
    CS1237_SPEED_1280HZ = 3,
};

enum {
    CS1237_REFO_ON = 0,
    CS1237_REFO_OFF = 1,
};

typedef union {
    struct {
        uint8_t ch : 2;
        uint8_t pga : 2;
        uint8_t speed : 2;
        uint8_t refo : 1;
    };
    uint8_t byte;
} cs1237_config_t;

esp_err_t cs1237_new(const gpio_num_t clk_io_num, const gpio_num_t dio_io_num,
                     cs1237_handle_t *ret_handle);
esp_err_t cs1237_del(cs1237_handle_t handle);

#define CS1237_MAX (0x7FFFFF)
#define CS1237_MIN ((int32_t)0xFF800000)

esp_err_t cs1237_get_raw(cs1237_handle_t handle, int32_t *raw);
esp_err_t cs1237_get_config(cs1237_handle_t handle, cs1237_config_t *cfg);
esp_err_t cs1237_set_config(cs1237_handle_t handle, cs1237_config_t cfg);
esp_err_t cs1237_power_down(cs1237_handle_t handle, bool down);

static inline int cs1237_get_gain(int pga)
{
    return (pga > CS1237_PGA_2) ? (pga == CS1237_PGA_128 ? 128 : 64)
                                : (pga == CS1237_PGA_2 ? 2 : 1);
}

static inline float cs1237_get_voltage(int32_t raw, float vdd, int gain)
{
    return 0.5 * raw * vdd / gain / CS1237_MAX;
}

static inline bool cs1237_is_positive_full(int32_t raw)
{
    return raw >= CS1237_MAX;
}

static inline bool cs1237_is_negative_full(int32_t raw)
{
    return raw <= CS1237_MIN;
}

static inline bool cs1237_is_full(int32_t raw)
{
    return cs1237_is_positive_full(raw) || cs1237_is_negative_full(raw);
}

#ifdef __cplusplus
}
#endif
