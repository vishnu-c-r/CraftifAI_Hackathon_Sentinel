#ifndef OPERATING_STATE_H
#define OPERATING_STATE_H

#include <stdint.h>

typedef enum {
    SENTINEL_STATE_OFF,
    SENTINEL_STATE_STARTING,
    SENTINEL_STATE_RUNNING,
    SENTINEL_STATE_STOPPING,
    SENTINEL_STATE_FAULT
} sentinel_state_t;

void operating_state_init(void);
sentinel_state_t operating_state_update(float rms_g, uint64_t now_ms);
const char *operating_state_name(sentinel_state_t state);

#endif /* OPERATING_STATE_H */
