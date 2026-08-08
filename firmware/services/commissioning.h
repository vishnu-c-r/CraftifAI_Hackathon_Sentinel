#ifndef COMMISSIONING_H
#define COMMISSIONING_H

#include <stdbool.h>
#include "esp_err.h"

#define COMMISSIONING_COMMAND_MAX 32

typedef struct {
    bool learning;
    bool baseline_valid;
    unsigned cycles;
} commissioning_status_t;

void commissioning_start(void);
esp_err_t commissioning_dispatch_command(const char *command);
void commissioning_get_status(commissioning_status_t *status);

#endif /* COMMISSIONING_H */
