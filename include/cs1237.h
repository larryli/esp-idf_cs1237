/**
 * @file cs1237.h
 * @brief CS1237 24-bit Sigma-Delta ADC driver for ESP-IDF
 *
 * This driver provides an interface to the CS1237 ADC chip via a 2-wire
 * serial protocol (CLK + DIO) using bit-banged GPIO.
 *
 * Copyright 2024 Larry Li <larryli@qq.com>
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include "esp_err.h"
#include "hal/gpio_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque handle for CS1237 device instance
 */
typedef struct cs1237_t *cs1237_handle_t;

/**
 * @brief CS1237 input channel selection
 *
 * Controls which analog input is connected to the ADC.
 * See CS1237 datasheet for details.
 */
enum {
    CS1237_CH_A = 0,        /**< Channel A (default analog input) */
    CS1237_CH_RESERVED = 1, /**< Reserved (do not use) */
    CS1237_CH_TEMP = 2,     /**< Internal temperature sensor */
    CS1237_CH_SHORT = 3,    /**< Short input (for offset calibration) */
};

/**
 * @brief CS1237 Programmable Gain Amplifier (PGA) settings
 *
 * Controls the gain of the internal amplifier.
 */
enum {
    CS1237_PGA_1 = 0,   /**< Gain = 1 */
    CS1237_PGA_2 = 1,   /**< Gain = 2 (default) */
    CS1237_PGA_64 = 2,  /**< Gain = 64 */
    CS1237_PGA_128 = 3, /**< Gain = 128 */
};

/**
 * @brief CS1237 output data rate (sampling speed) settings
 *
 * Controls the ADC conversion speed. Higher speed means lower resolution.
 */
enum {
    CS1237_SPEED_10HZ = 0,  /**< 10 Hz output rate */
    CS1237_SPEED_40HZ = 1,  /**< 40 Hz output rate (default) */
    CS1237_SPEED_640HZ = 2, /**< 640 Hz output rate */
    CS1237_SPEED_1280HZ = 3, /**< 1280 Hz output rate */
};

/**
 * @brief CS1237 reference voltage output control
 *
 * Controls whether the internal reference voltage is output on the REFO pin.
 */
enum {
    CS1237_REFO_ON = 0,  /**< Reference output enabled (default) */
    CS1237_REFO_OFF = 1, /**< Reference output disabled */
};

/**
 * @brief CS1237 configuration register structure
 *
 * This union represents the 8-bit configuration register of the CS1237.
 * It can be accessed as individual bitfields or as a raw byte.
 *
 * @note The bitfield layout matches the CS1237 datasheet specification.
 */
typedef union {
    struct {
        uint8_t ch : 2;    /**< Input channel selection (bits 0-1) */
        uint8_t pga : 2;   /**< PGA gain setting (bits 2-3) */
        uint8_t speed : 2; /**< Output data rate (bits 4-5) */
        uint8_t refo : 1;  /**< Reference output control (bit 6) */
    };
    uint8_t byte; /**< Raw byte access to the configuration register */
} cs1237_config_t;

/**
 * @brief Create a new CS1237 device instance
 *
 * Initializes GPIO pins and allocates resources for the CS1237 driver.
 *
 * @param[in]  clk_io_num   GPIO number for the CLK signal
 * @param[in]  dio_io_num   GPIO number for the DIO signal
 * @param[out] ret_handle   Pointer to receive the created handle
 *
 * @return
 *     - ESP_OK                Success
 *     - ESP_ERR_INVALID_ARG   Invalid GPIO number or NULL pointer
 *     - ESP_ERR_NO_MEM        Memory allocation failed
 */
esp_err_t cs1237_new(const gpio_num_t clk_io_num, const gpio_num_t dio_io_num,
                     cs1237_handle_t *ret_handle);

/**
 * @brief Delete a CS1237 device instance
 *
 * Releases resources allocated for the CS1237 driver.
 *
 * @param[in] handle  Handle created by cs1237_new()
 *
 * @return
 *     - ESP_OK                Success
 *     - ESP_ERR_INVALID_ARG   Invalid handle
 */
esp_err_t cs1237_del(cs1237_handle_t handle);

/** @brief Maximum positive raw ADC value (24-bit signed: 0x7FFFFF) */
#define CS1237_MAX (0x7FFFFF)

/** @brief Minimum negative raw ADC value (24-bit signed: -8388608) */
#define CS1237_MIN ((int32_t)0xFF800000)

/**
 * @brief Read raw ADC value from CS1237
 *
 * Reads a 24-bit signed value from the ADC. The value is sign-extended
 * to 32 bits for easier processing.
 *
 * @param[in]  handle  Handle created by cs1237_new()
 * @param[out] raw     Pointer to receive the 24-bit signed raw value
 *
 * @return
 *     - ESP_OK                   Success
 *     - ESP_ERR_INVALID_ARG      Invalid handle
 *     - ESP_ERR_INVALID_RESPONSE Timeout waiting for data ready
 */
esp_err_t cs1237_get_raw(cs1237_handle_t handle, int32_t *raw);

/**
 * @brief Read configuration register from CS1237
 *
 * Reads the current configuration from the CS1237 chip and updates
 * the internal timeout based on the configured speed.
 *
 * @param[in]  handle  Handle created by cs1237_new()
 * @param[out] cfg     Pointer to receive the configuration
 *
 * @return
 *     - ESP_OK                   Success
 *     - ESP_ERR_INVALID_ARG      Invalid handle
 *     - ESP_ERR_INVALID_RESPONSE Timeout waiting for data ready
 *     - ESP_ERR_INVALID_STATE    Speed exceeds INT_WDT timeout
 */
esp_err_t cs1237_get_config(cs1237_handle_t handle, cs1237_config_t *cfg);

/**
 * @brief Write configuration register to CS1237
 *
 * Writes a new configuration to the CS1237 chip and updates
 * the internal timeout based on the configured speed.
 *
 * @param[in] handle  Handle created by cs1237_new()
 * @param[in] cfg     Configuration to write
 *
 * @return
 *     - ESP_OK                   Success
 *     - ESP_ERR_INVALID_ARG      Invalid handle or speed exceeds INT_WDT timeout
 *     - ESP_ERR_INVALID_RESPONSE Timeout waiting for data ready
 */
esp_err_t cs1237_set_config(cs1237_handle_t handle, cs1237_config_t cfg);

/**
 * @brief Control CS1237 power mode
 *
 * Puts the CS1237 into power-down mode or wakes it up.
 *
 * @note In power-down mode, the CLK line is held HIGH.
 *       To wake up, the CLK line is held LOW for a specified delay.
 *
 * @param[in] handle  Handle created by cs1237_new()
 * @param[in] down    true to power down, false to power up
 *
 * @return
 *     - ESP_OK              Success
 *     - ESP_ERR_INVALID_ARG Invalid handle
 */
esp_err_t cs1237_power_down(cs1237_handle_t handle, bool down);

/**
 * @brief Convert PGA enum value to numeric gain
 *
 * @param[in] pga  PGA enum value (CS1237_PGA_xxx)
 *
 * @return Numeric gain value (1, 2, 64, or 128)
 */
static inline int cs1237_get_gain(int pga)
{
    return (pga > CS1237_PGA_2) ? (pga == CS1237_PGA_128 ? 128 : 64)
                                : (pga == CS1237_PGA_2 ? 2 : 1);
}

/**
 * @brief Convert raw ADC value to voltage in millivolts
 *
 * Calculates the actual voltage from the raw ADC reading using the formula:
 * V = 0.5 * raw * VDD / gain / 2^23
 *
 * The 0.5 factor comes from the CS1237's internal reference voltage
 * (Vref = VDD / 2 when REFO is enabled).
 *
 * @param[in] raw  Raw 24-bit signed ADC value from cs1237_get_raw()
 * @param[in] vdd  Supply voltage in millivolts (e.g., 3300 for 3.3V)
 * @param[in] gain PGA gain value from cs1237_get_gain()
 *
 * @return Voltage in millivolts
 */
static inline float cs1237_get_voltage(int32_t raw, float vdd, int gain)
{
    return 0.5 * raw * vdd / gain / CS1237_MAX;
}

/**
 * @brief Check if ADC reading is at positive full-scale
 *
 * @param[in] raw  Raw 24-bit signed ADC value
 *
 * @return true if value is at or above positive full-scale (0x7FFFFF)
 */
static inline bool cs1237_is_positive_full(int32_t raw)
{
    return raw >= CS1237_MAX;
}

/**
 * @brief Check if ADC reading is at negative full-scale
 *
 * @param[in] raw  Raw 24-bit signed ADC value
 *
 * @return true if value is at or below negative full-scale (-8388608)
 */
static inline bool cs1237_is_negative_full(int32_t raw)
{
    return raw <= CS1237_MIN;
}

/**
 * @brief Check if ADC reading is at either full-scale limit
 *
 * @param[in] raw  Raw 24-bit signed ADC value
 *
 * @return true if value is at positive or negative full-scale
 */
static inline bool cs1237_is_full(int32_t raw)
{
    return cs1237_is_positive_full(raw) || cs1237_is_negative_full(raw);
}

#ifdef __cplusplus
}
#endif
