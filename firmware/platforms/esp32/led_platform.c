#include "led_platform.h"
#include "app_config.h"
#include "logger.h"

#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "led_strip_rmt.h"

static led_strip_handle_t s_led;
static volatile led_status_t s_status = LED_STATUS_BOOT;

static void status_color(led_status_t status, uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = 0; *g = 0; *b = 0;
    switch (status) {
    case LED_STATUS_BOOT: *b = APP_LED_BRIGHTNESS; break;
    case LED_STATUS_BASELINE: *r = APP_LED_BRIGHTNESS; *g = APP_LED_BRIGHTNESS; break;
    case LED_STATUS_RUNNING: *g = APP_LED_BRIGHTNESS; break;
    case LED_STATUS_WARNING: *r = APP_LED_BRIGHTNESS; *g = APP_LED_BRIGHTNESS / 2; break;
    case LED_STATUS_CRITICAL: *r = APP_LED_BRIGHTNESS; break;
    case LED_STATUS_SENSOR_FAULT: *r = APP_LED_BRIGHTNESS; *b = APP_LED_BRIGHTNESS; break;
    default: break;
    }
}

esp_err_t led_platform_init(void)
{
    const led_strip_config_t config = {.strip_gpio_num = APP_LED_GPIO, .max_leds = APP_LED_COUNT};
    const led_strip_rmt_config_t rmt = {.resolution_hz = 10 * 1000 * 1000, .flags.with_dma = false};
    esp_err_t err = led_strip_new_rmt_device(&config, &rmt, &s_led);
    if (err == ESP_OK) err = led_strip_clear(s_led);
    if (err == ESP_OK) ESP_LOGI("led", "WS2812 LED ready on GPIO %d", APP_LED_GPIO);
    return err;
}

static void led_platform_task(void *arg)
{
    (void)arg;
    bool pulse = false;
    for (;;) {
        uint8_t r, g, b;
        status_color(s_status, &r, &g, &b);
        if (s_status == LED_STATUS_BASELINE || s_status == LED_STATUS_WARNING ||
            s_status == LED_STATUS_CRITICAL || s_status == LED_STATUS_SENSOR_FAULT) {
            pulse = !pulse;
            if (!pulse) r = g = b = 0;
        }
        led_strip_set_pixel(s_led, 0, r, g, b);
        led_strip_refresh(s_led);
        vTaskDelay(pdMS_TO_TICKS(s_status == LED_STATUS_CRITICAL ? 150 : 500));
    }
}

void led_platform_start(void)
{
    xTaskCreate(led_platform_task, "led", 2048, NULL, 4, NULL);
}

void led_platform_set_status(led_status_t status)
{
    s_status = status;
}
