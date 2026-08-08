#include "operating_state.h"
#include <stdbool.h>
#include "app_config.h"

static sentinel_state_t s_state;
static uint64_t s_transition_ms;

void operating_state_init(void)
{
    s_state = SENTINEL_STATE_OFF;
    s_transition_ms = 0;
}

const char *operating_state_name(sentinel_state_t state)
{
    switch (state) {
    case SENTINEL_STATE_OFF: return "OFF";
    case SENTINEL_STATE_STARTING: return "STARTING";
    case SENTINEL_STATE_RUNNING: return "RUNNING";
    case SENTINEL_STATE_STOPPING: return "STOPPING";
    case SENTINEL_STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
    }
}

sentinel_state_t operating_state_update(float rms_g, uint64_t now_ms)
{
    const bool active = rms_g >= APP_STATE_START_THRESHOLD_G;
    const bool stopped = rms_g <= APP_STATE_STOP_THRESHOLD_G;
    const bool fault = rms_g >= APP_STATE_FAULT_THRESHOLD_G;
    const bool recovered = rms_g <= APP_STATE_RECOVER_THRESHOLD_G;

    switch (s_state) {
    case SENTINEL_STATE_OFF:
        if (active) { s_state = SENTINEL_STATE_STARTING; s_transition_ms = now_ms; }
        break;
    case SENTINEL_STATE_STARTING:
        if (!active) s_state = SENTINEL_STATE_OFF;
        else if (now_ms - s_transition_ms >= APP_STATE_START_SETTLE_MS) s_state = SENTINEL_STATE_RUNNING;
        break;
    case SENTINEL_STATE_RUNNING:
        if (fault) { s_state = SENTINEL_STATE_FAULT; s_transition_ms = now_ms; }
        else if (stopped) { s_state = SENTINEL_STATE_STOPPING; s_transition_ms = now_ms; }
        break;
    case SENTINEL_STATE_STOPPING:
        if (active) s_state = SENTINEL_STATE_RUNNING;
        else if (now_ms - s_transition_ms >= APP_STATE_STOP_CONFIRM_MS) s_state = SENTINEL_STATE_OFF;
        break;
    case SENTINEL_STATE_FAULT:
        if (recovered && now_ms - s_transition_ms >= APP_STATE_CLEAR_PERSIST_MS) s_state = SENTINEL_STATE_RUNNING;
        break;
    }
    return s_state;
}
