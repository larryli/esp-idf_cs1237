#include "cs1237.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define CLK_IO_PIN GPIO_NUM_19
#define DIO_IO_PIN GPIO_NUM_18

static const char TAG[] = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "start");
    cs1237_handle_t handle;
    ESP_ERROR_CHECK(cs1237_new(CLK_IO_PIN, DIO_IO_PIN, &handle));

    cs1237_config_t config = {
        .ch = CS1237_CH_A,
        .pga = CS1237_PGA_2,
        .speed = CS1237_SPEED_1280HZ,
        .refo = CS1237_REFO_ON,
    };
    ESP_LOGI(TAG, "Set config: 0x%02x", config.byte);
    ESP_ERROR_CHECK(cs1237_set_config(handle, config));
    ESP_ERROR_CHECK(cs1237_get_config(handle, &config));
    ESP_LOGI(TAG, "Get config: 0x%02x", config.byte);
    int gain = cs1237_get_gain(config.pga);

    int32_t raw;
    while (1) {
        ESP_ERROR_CHECK(cs1237_get_raw(handle, &raw));
        if (cs1237_is_positive_full(raw)) {
            ESP_LOGW(TAG, "The adc raw value is positive full");
        } else if (cs1237_is_negative_full(raw)) {
            ESP_LOGW(TAG, "The adc raw value is negative full");
        } else {
            ESP_LOGI(TAG, "Adc raw data: 0x%lx, value: %ld, voltage: %7.2fmV",
                     (uint32_t)raw, raw, cs1237_get_voltage(raw, 3300, gain));
        }
        cs1237_power_down(handle, true);
        vTaskDelay(pdMS_TO_TICKS(1000));
        cs1237_power_down(handle, false);
    }
}
