#include "cs1237.h"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char TAG[] = "cs1237";

struct cs1237_t {
    gpio_num_t clk_num;
    gpio_num_t dio_num;
    uint32_t wait_us;
};

static portMUX_TYPE g_lock = portMUX_INITIALIZER_UNLOCKED;

#define DELAY_US CONFIG_CS1237_DELAY_US
#define POWER_DOWN_DELAY_US CONFIG_CS1237_POWER_DOWN_DELAY_US
#define POWER_UP_DELAY_US CONFIG_CS1237_POWER_UP_DELAY_US

#define WAIT_US_10HZ (305 * 1000)
#define WAIT_US_40HZ (78 * 1000)
#define WAIT_US_640HZ (8000)
#define WAIT_US_1280HZ (5200)

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
    handle->wait_us = WAIT_US_40HZ;

    const gpio_config_t clk_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = false,
        .pin_bit_mask = 1ULL << handle->clk_num,
    };
    ESP_GOTO_ON_ERROR(gpio_set_level(handle->clk_num, 0), err, TAG,
                      "CLK pin set level failed");
    ESP_GOTO_ON_ERROR(gpio_config(&clk_conf), err, TAG, "config GPIO failed");
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

static inline bool _wait_dio_ready(cs1237_handle_t handle)
{
    int64_t now = esp_timer_get_time();
    int64_t timeout = now + handle->wait_us;
    // wait low
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

static inline void _set_clk(cs1237_handle_t handle)
{
    gpio_set_level(handle->clk_num, 1);
    esp_rom_delay_us(DELAY_US);
    gpio_set_level(handle->clk_num, 0);
    esp_rom_delay_us(DELAY_US);
}

static inline void _set_dio(cs1237_handle_t handle, uint8_t dio)
{
    gpio_set_level(handle->clk_num, 1);
    gpio_set_level(handle->dio_num, dio);
    esp_rom_delay_us(DELAY_US);
    gpio_set_level(handle->clk_num, 0);
    esp_rom_delay_us(DELAY_US);
}

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
    if (!_wait_dio_ready(handle)) {
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGE(TAG, "wait dio ready failed");
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint32_t d = 0;
    for (int i = 0; i < 24; i++) { // 1 ~ 24
        _set_clk(handle);
        d <<= 1;
        d |= gpio_get_level(handle->dio_num) & 1;
    }
    *raw = (int32_t)(d << 8) >> 8; // 32bits --> 24bits
    for (int i = 0; i < 2; i++) {  // 25 ~ 26
        _set_clk(handle);
    }
    gpio_set_direction(handle->dio_num, GPIO_MODE_OUTPUT);
    _set_dio(handle, 1); // 27
    gpio_set_direction(handle->dio_num, GPIO_MODE_INPUT);
    portEXIT_CRITICAL(&g_lock);

    return ESP_OK;
}

static inline int get_wait_us(int speed)
{
    return speed > CS1237_SPEED_40HZ
               ? (speed == CS1237_SPEED_1280HZ ? WAIT_US_1280HZ : WAIT_US_640HZ)
               : (speed == CS1237_SPEED_40HZ ? WAIT_US_40HZ : WAIT_US_10HZ);
}

esp_err_t cs1237_get_config(cs1237_handle_t handle, cs1237_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid handle");

    portENTER_CRITICAL(&g_lock);
    if (!_wait_dio_ready(handle)) {
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGE(TAG, "wait dio ready failed");
        return ESP_ERR_INVALID_RESPONSE;
    }
    _send_command(handle, 0x56); // 1 ~ 36
    _set_dio(handle, 1);         // 37
    uint8_t d = 0;
    for (int i = 0; i < 8; i++) { // 38 ~ 45
        _set_clk(handle);
        d <<= 1;
        d |= gpio_get_level(handle->dio_num) & 1;
    }
    cfg->byte = d;
    _set_dio(handle, 1); // 46

    portEXIT_CRITICAL(&g_lock);

    handle->wait_us = get_wait_us(cfg->speed);
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
#ifdef CONFIG_ESP_INT_WDT
    ESP_RETURN_ON_FALSE(wait_us < CONFIG_ESP_INT_WDT_TIMEOUT_MS * 1000,
                        ESP_ERR_INVALID_ARG, TAG,
                        "invalid speed config. Please speed up or disable "
                        "INT_WDT or increase INT_WDT_TIMEOUT value");
#endif

    portENTER_CRITICAL(&g_lock);
    if (!_wait_dio_ready(handle)) {
        portEXIT_CRITICAL(&g_lock);
        ESP_LOGE(TAG, "wait dio ready failed");
        return ESP_ERR_INVALID_RESPONSE;
    }
    _send_command(handle, 0x65); // 1 ~ 36
    _set_clk(handle);            // 37
    gpio_set_direction(handle->dio_num, GPIO_MODE_OUTPUT);
    for (int i = 7; i >= 0; i--) { // 38 ~ 45
        _set_dio(handle, (cfg.byte >> i) & 1);
    }
    _set_dio(handle, 1); // 46
    gpio_set_direction(handle->dio_num, GPIO_MODE_INPUT);

    handle->wait_us = wait_us;

    portEXIT_CRITICAL(&g_lock);

    return ESP_OK;
}

esp_err_t cs1237_power_down(cs1237_handle_t handle, bool down)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "invalid handle");
    portENTER_CRITICAL(&g_lock);
    if (down) {
        gpio_set_level(handle->clk_num, 1);
        esp_rom_delay_us(POWER_DOWN_DELAY_US);
    } else {
        gpio_set_level(handle->clk_num, 0);
        esp_rom_delay_us(POWER_UP_DELAY_US);
    }
    portEXIT_CRITICAL(&g_lock);
    return ESP_OK;
}
