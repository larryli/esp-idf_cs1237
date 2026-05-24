/**
 * @file cs1237.c
 * @brief CS1237 ADC driver implementation
 *
 * Implements bit-banged 2-wire serial protocol for CS1237 communication.
 * Protocol: CLK (clock) + DIO (data bidirectional)
 *
 * CS1237 Protocol Overview:
 * - Data is clocked on rising edge of CLK
 * - DIO changes on falling edge of CLK
 * - 24 clock cycles for ADC data, 7 cycles for command
 * - Total frame: 24 (data) + 2 (unused) + 1 (direction) + 2 (unused) + 7 (command) = 36 cycles
 *
 * Copyright 2024 Larry Li <larryli@qq.com>
 * SPDX-License-Identifier: MIT
 */
#include "cs1237.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char TAG[] = "cs1237";

/**
 * @brief Internal CS1237 device structure
 */
struct cs1237_t {
    gpio_num_t clk_num; /**< GPIO number for CLK signal */
    gpio_num_t dio_num; /**< GPIO number for DIO signal */
    uint32_t wait_us;   /**< Timeout in microseconds for waiting DIO ready */
};

/**
 * @brief Global spinlock for critical sections
 *
 * @note This lock is shared across all CS1237 instances.
 *       If multiple devices are used, consider moving this into struct cs1237_t.
 */
static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

/** @brief CLK pulse half-period delay in microseconds (from Kconfig) */
#define DELAY_US CONFIG_CS1237_DELAY_US

/** @brief Delay before entering power-down mode (from Kconfig) */
#define POWER_DOWN_DELAY_US CONFIG_CS1237_POWER_DOWN_DELAY_US

/** @brief Delay after exiting power-down mode (from Kconfig) */
#define POWER_UP_DELAY_US CONFIG_CS1237_POWER_UP_DELAY_US

/*
 * Timeout values for different sampling speeds.
 * These are slightly longer than the actual conversion periods to ensure
 * the ADC has completed a conversion before we try to read it.
 */
#define WAIT_US_10HZ (305 * 1000)   /**< ~305ms timeout for 10Hz mode */
#define WAIT_US_40HZ (78 * 1000)    /**< ~78ms timeout for 40Hz mode */
#define WAIT_US_640HZ (8000)        /**< ~8ms timeout for 640Hz mode */
#define WAIT_US_1280HZ (5200)       /**< ~5.2ms timeout for 1280Hz mode */

/**
 * @brief Get timeout value for the given speed setting
 *
 * @param[in] speed  Speed enum value (CS1237_SPEED_xxx)
 *
 * @return Timeout in microseconds
 */
static inline int get_wait_us(int speed)
{
    return speed > CS1237_SPEED_40HZ
               ? (speed == CS1237_SPEED_1280HZ ? WAIT_US_1280HZ : WAIT_US_640HZ)
               : (speed == CS1237_SPEED_40HZ ? WAIT_US_40HZ : WAIT_US_10HZ);
}

esp_err_t cs1237_new(const gpio_num_t clk_io_num, const gpio_num_t dio_io_num,
                     cs1237_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(clk_io_num), ESP_ERR_INVALID_ARG,
                        TAG, "invalid CLK pin number");
    ESP_RETURN_ON_FALSE(GPIO_IS_VALID_GPIO(dio_io_num), ESP_ERR_INVALID_ARG,
                        TAG, "invalid DIO pin number");

    esp_err_t ret = ESP_OK;
    cs1237_handle_t handle =
        (cs1237_handle_t)calloc(1, sizeof(struct cs1237_t));
    ESP_GOTO_ON_FALSE(handle, ESP_ERR_NO_MEM, err, TAG, "no memory");
    handle->clk_num = clk_io_num;
    handle->dio_num = dio_io_num;
    handle->wait_us = WAIT_US_40HZ; /**< Default timeout for 40Hz mode */

    /* Configure CLK pin as output, initially LOW */
    const gpio_config_t clk_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = false,
        .pin_bit_mask = 1ULL << handle->clk_num,
    };
    ESP_GOTO_ON_ERROR(gpio_set_level(handle->clk_num, 0), err, TAG,
                      "CLK pin set level failed");
    ESP_GOTO_ON_ERROR(gpio_config(&clk_conf), err, TAG, "config GPIO failed");

    /* Configure DIO pin as input (bidirectional, starts as input) */
    const gpio_config_t dio_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = false,
        .pin_bit_mask = 1ULL << handle->dio_num,
    };
    ESP_GOTO_ON_ERROR(gpio_set_level(handle->dio_num, 1), err, TAG,
                      "DIO pin set level failed");
    ESP_GOTO_ON_ERROR(gpio_config(&dio_conf), err, TAG, "config GPIO failed");

    *ret_handle = handle;
    return ESP_OK;

err:
    free(handle);
    return ret;
}

esp_err_t cs1237_del(cs1237_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    free(handle);
    return ESP_OK;
}

/**
 * @brief Wait for DIO to become ready (LOW)
 *
 * The CS1237 pulls DIO LOW when it has completed a conversion and data
 * is ready to be read. This function polls DIO with a timeout.
 *
 * @param[in] handle  CS1237 device handle
 *
 * @return true if DIO is ready, false on timeout
 */
static inline bool _wait_dio_ready(cs1237_handle_t handle)
{
    int64_t now = esp_timer_get_time();
    int64_t timeout = now + handle->wait_us;
    /* Poll DIO pin in a tight loop with periodic timeout check.
     * Inner loop of 63356 iterations is an optimization to reduce
     * the overhead of calling esp_timer_get_time() on each check.
     */
    do {
        for (int i = 0; i < 63356; i++) {
            if (gpio_get_level(handle->dio_num) == 0) {
                return true;
            }
        }
        now = esp_timer_get_time();
    } while (now <= timeout);
    return false;
}

/**
 * @brief Generate one CLK pulse (HIGH then LOW with delays)
 *
 * @param[in] handle  CS1237 device handle
 */
static inline void _set_clk(cs1237_handle_t handle)
{
    gpio_set_level(handle->clk_num, 1);
    esp_rom_delay_us(DELAY_US);
    gpio_set_level(handle->clk_num, 0);
    esp_rom_delay_us(DELAY_US);
}

/**
 * @brief Set DIO value and generate one CLK pulse
 *
 * Sets DIO to the specified value while CLK is HIGH, then generates
 * a complete clock cycle.
 *
 * @param[in] handle  CS1237 device handle
 * @param[in] dio     Value to set on DIO (0 or 1)
 */
static inline void _set_dio(cs1237_handle_t handle, uint8_t dio)
{
    gpio_set_level(handle->clk_num, 1);
    gpio_set_level(handle->dio_num, dio);
    esp_rom_delay_us(DELAY_US);
    gpio_set_level(handle->clk_num, 0);
    esp_rom_delay_us(DELAY_US);
}

/**
 * @brief Send a 7-bit command to CS1237
 *
 * Implements the CS1237 command transmission protocol:
 * - Cycles 1-26: 26 CLK pulses (waiting period)
 * - Cycle 27: DIO set HIGH (direction change to output)
 * - Cycles 28-29: 2 CLK pulses
 * - Cycles 30-36: 7-bit command, MSB first
 *
 * Command codes:
 * - 0x56 (01010110b): Read configuration register
 * - 0x65 (01100101b): Write configuration register
 *
 * @param[in] handle  CS1237 device handle
 * @param[in] cmd     7-bit command to send
 */
static inline void _send_command(cs1237_handle_t handle, uint8_t cmd)
{
    for (int i = 0; i < 26; i++) { // 1 ~ 26
        _set_clk(handle);
    }
    gpio_set_direction(handle->dio_num, GPIO_MODE_OUTPUT);
    _set_dio(handle, 1);          // 27
    for (int i = 0; i < 2; i++) { // 28 ~ 29
        _set_clk(handle);
    }
    for (int i = 6; i >= 0; i--) { // 30 ~ 36
        _set_dio(handle, (cmd >> i) & 1);
    }
    gpio_set_direction(handle->dio_num, GPIO_MODE_INPUT);
}

esp_err_t cs1237_get_raw(cs1237_handle_t handle, int32_t *raw)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid handle");

    portENTER_CRITICAL(&g_lock);
    /* Wait for ADC conversion complete (DIO goes LOW) */
    if (!_wait_dio_ready(handle)) {
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGE(TAG, "wait dio ready failed");
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Read 24-bit ADC value, MSB first */
    uint32_t d = 0;
    for (int i = 0; i < 24; i++) { // 1 ~ 24
        _set_clk(handle);
        d <<= 1;
        d |= gpio_get_level(handle->dio_num) & 1;
    }
    /* Sign-extend from 24-bit to 32-bit */
    *raw = (int32_t)(d << 8) >> 8;

    /* Send 2 CLK pulses (cycles 25-26, unused) */
    for (int i = 0; i < 2; i++) { // 25 ~ 26
        _set_clk(handle);
    }

    /* Cycle 27: DIO set HIGH to indicate end of read */
    gpio_set_direction(handle->dio_num, GPIO_MODE_OUTPUT);
    _set_dio(handle, 1); // 27
    gpio_set_direction(handle->dio_num, GPIO_MODE_INPUT);

    portEXIT_CRITICAL(&g_lock);

    return ESP_OK;
}

esp_err_t cs1237_get_config(cs1237_handle_t handle, cs1237_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid handle");

    portENTER_CRITICAL(&g_lock);
    /* Wait for any pending conversion to complete */
    if (!_wait_dio_ready(handle)) {
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGE(TAG, "wait dio ready failed");
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Send read config command (0x56 = 01010110b) */
    _send_command(handle, 0x56); // 1 ~ 36

    /* Cycle 37: Set DIO HIGH (ready for read) */
    _set_dio(handle, 1); // 37

    /* Read 8-bit configuration register, MSB first (cycles 38-45) */
    uint8_t d = 0;
    for (int i = 0; i < 8; i++) { // 38 ~ 45
        _set_clk(handle);
        d <<= 1;
        d |= gpio_get_level(handle->dio_num) & 1;
    }
    cfg->byte = d;

    /* Cycle 46: Set DIO HIGH to end transaction */
    _set_dio(handle, 1); // 46

    portEXIT_CRITICAL(&g_lock);

    /* Update internal timeout based on new speed setting */
    handle->wait_us = get_wait_us(cfg->speed);

    /* Validate that the timeout doesn't exceed the interrupt watchdog timer */
    ESP_RETURN_ON_FALSE(handle->wait_us < CONFIG_ESP_INT_WDT_TIMEOUT_MS * 1000,
                        ESP_ERR_INVALID_STATE, TAG,
                        "invalid speed config. Please speed up or disable "
                        "INT_WDT or increase INT_WDT_TIMEOUT value");

    return ESP_OK;
}

esp_err_t cs1237_set_config(cs1237_handle_t handle, cs1237_config_t cfg)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid handle");

    uint32_t wait_us = get_wait_us(cfg.speed);

    /* Validate speed doesn't exceed interrupt watchdog timeout */
#ifdef CONFIG_ESP_INT_WDT
    ESP_RETURN_ON_FALSE(wait_us < CONFIG_ESP_INT_WDT_TIMEOUT_MS * 1000,
                        ESP_ERR_INVALID_ARG, TAG,
                        "invalid speed config. Please speed up or disable "
                        "INT_WDT or increase INT_WDT_TIMEOUT value");
#endif

    portENTER_CRITICAL(&g_lock);
    /* Wait for any pending conversion to complete */
    if (!_wait_dio_ready(handle)) {
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGE(TAG, "wait dio ready failed");
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Send write config command (0x65 = 01100101b) */
    _send_command(handle, 0x65); // 1 ~ 36

    /* Cycle 37: CLK pulse */
    _set_clk(handle); // 37

    /* Write 8-bit configuration register, MSB first (cycles 38-45) */
    gpio_set_direction(handle->dio_num, GPIO_MODE_OUTPUT);
    for (int i = 7; i >= 0; i--) { // 38 ~ 45
        _set_dio(handle, (cfg.byte >> i) & 1);
    }

    /* Cycle 46: Set DIO HIGH to end transaction */
    _set_dio(handle, 1); // 46
    gpio_set_direction(handle->dio_num, GPIO_MODE_INPUT);

    /* Update internal timeout based on new speed setting */
    handle->wait_us = wait_us;

    portEXIT_CRITICAL(&g_lock);

    return ESP_OK;
}

esp_err_t cs1237_power_down(cs1237_handle_t handle, bool down)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid handle");

    portENTER_CRITICAL(&g_lock);
    if (down) {
        /* Power down: hold CLK HIGH */
        gpio_set_level(handle->clk_num, 1);
        esp_rom_delay_us(POWER_DOWN_DELAY_US);
    } else {
        /* Power up: hold CLK LOW */
        gpio_set_level(handle->clk_num, 0);
        esp_rom_delay_us(POWER_UP_DELAY_US);
    }
    portEXIT_CRITICAL(&g_lock);

    return ESP_OK;
}
