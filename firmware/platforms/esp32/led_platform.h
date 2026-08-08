#ifndef LED_PLATFORM_H
#define LED_PLATFORM_H

#include "esp_err.h"

typedef enum {
    LED_STATUS_OFF,
    LED_STATUS_BOOT,
    LED_STATUS_BASELINE,
    LED_STATUS_RUNNING,
    LED_STATUS_WARNING,
    LED_STATUS_CRITICAL,
    LED_STATUS_SENSOR_FAULT
} led_status_t;

esp_err_t led_platform_init(void);
void led_platform_start(void);
void led_platform_set_status(led_status_t status);

#endif /* LED_PLATFORM_H */
