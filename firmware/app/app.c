#include "app.h"
#include "app_config.h"
#include "logger.h"
#include "led_platform.h"
#include "sensor_platform.h"
#include "commissioning.h"
#include "dashboard.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "app";

void app_start(void)
{
    ESP_LOGI(TAG, "CraftiAI Sentinel starting");
    if (led_platform_init() == ESP_OK) {
        led_platform_start();
    } else {
        ESP_LOGE(TAG, "LED initialization failed");
    }
    if (sensor_platform_init() == ESP_OK) {
        sensor_platform_start();
        commissioning_start();
        if (dashboard_start() != ESP_OK) {
            ESP_LOGW(TAG, "dashboard startup failed; serial control remains active");
        }
    } else {
        ESP_LOGW(TAG, "sensor initialization failed");
    }
}
