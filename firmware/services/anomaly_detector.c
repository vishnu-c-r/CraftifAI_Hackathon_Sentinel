#include "anomaly_detector.h"
#include "baseline_learner.h"
#include "logger.h"

static anomaly_level_t s_level;
static uint64_t s_high_since;
static uint64_t s_recovery_since;

void anomaly_detector_init(void)
{
    s_level = ANOMALY_NONE;
    s_high_since = 0;
    s_recovery_since = 0;
}

const char *anomaly_detector_name(anomaly_level_t level)
{
    return level == ANOMALY_CRITICAL ? "CRITICAL" :
           level == ANOMALY_WARNING ? "WARNING" : "NONE";
}

anomaly_level_t anomaly_detector_update(float rms_g, uint64_t now_ms, bool baseline_valid)
{
    if (!baseline_valid) return s_level = ANOMALY_NONE;
    const float warning = baseline_learner_warning_g();
    const float critical = baseline_learner_critical_g();
    const float threshold = rms_g >= critical ? critical : warning;

    if (rms_g >= threshold) {
        if (s_high_since == 0) s_high_since = now_ms;
        s_recovery_since = 0;
        if (now_ms - s_high_since >= 2000) {
            s_level = rms_g >= critical ? ANOMALY_CRITICAL : ANOMALY_WARNING;
        }
    } else {
        s_high_since = 0;
        if (s_level != ANOMALY_NONE) {
            if (rms_g <= warning * 0.8f) {
                if (s_recovery_since == 0) s_recovery_since = now_ms;
                if (now_ms - s_recovery_since >= 5000) s_level = ANOMALY_NONE;
            } else {
                s_recovery_since = 0;
            }
        }
    }
    return s_level;
}

anomaly_level_t anomaly_detector_level(void) { return s_level; }
