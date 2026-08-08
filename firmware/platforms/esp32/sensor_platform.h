#ifndef SENSOR_PLATFORM_H
#define SENSOR_PLATFORM_H

#include "esp_err.h"

esp_err_t sensor_platform_init(void);
void sensor_platform_start(void);

#endif /* SENSOR_PLATFORM_H */
