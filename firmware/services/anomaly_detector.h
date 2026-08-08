#ifndef ANOMALY_DETECTOR_H
#define ANOMALY_DETECTOR_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ANOMALY_NONE,
    ANOMALY_WARNING,
    ANOMALY_CRITICAL
} anomaly_level_t;

void anomaly_detector_init(void);
anomaly_level_t anomaly_detector_update(float rms_g, uint64_t now_ms, bool baseline_valid);
anomaly_level_t anomaly_detector_level(void);
const char *anomaly_detector_name(anomaly_level_t level);

#endif /* ANOMALY_DETECTOR_H */
