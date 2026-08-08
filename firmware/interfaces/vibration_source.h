#ifndef VIBRATION_SOURCE_H
#define VIBRATION_SOURCE_H

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    float x_g;
    float y_g;
    float z_g;
    float magnitude_g;
} vibration_sample_t;

esp_err_t vibration_source_init(void);
esp_err_t vibration_source_read(vibration_sample_t *sample);
bool vibration_source_is_present(void);

#endif /* VIBRATION_SOURCE_H */
