#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "operating_state.h"
#include "anomaly_detector.h"

typedef struct {
    uint32_t uptime_ms;
    float vibration_rms_g;
    float vibration_peak_g;
    float temperature_c;
    bool temperature_valid;
    uint32_t samples;
    uint32_t errors;
    sentinel_state_t phase;
    anomaly_level_t anomaly;
} telemetry_snapshot_t;

void telemetry_publish(const telemetry_snapshot_t *snapshot);
bool telemetry_get_latest(telemetry_snapshot_t *snapshot);
size_t telemetry_history_json(char *buffer, size_t length);

#endif /* TELEMETRY_H */
