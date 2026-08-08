#include "telemetry.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TELEMETRY_HISTORY_SIZE 120
static telemetry_snapshot_t s_history[TELEMETRY_HISTORY_SIZE];
static size_t s_next;
static size_t s_count;
static SemaphoreHandle_t s_lock;

void telemetry_publish(const telemetry_snapshot_t *snapshot)
{
    if (!snapshot) return;
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
        s_history[s_next] = *snapshot;
        s_next = (s_next + 1U) % TELEMETRY_HISTORY_SIZE;
        if (s_count < TELEMETRY_HISTORY_SIZE) ++s_count;
        xSemaphoreGive(s_lock);
    }
}

bool telemetry_get_latest(telemetry_snapshot_t *snapshot)
{
    if (!snapshot || !s_lock || s_count == 0) return false;
    bool ok = false;
    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
        *snapshot = s_history[(s_next + TELEMETRY_HISTORY_SIZE - 1U) % TELEMETRY_HISTORY_SIZE];
        ok = true;
        xSemaphoreGive(s_lock);
    }
    return ok;
}

size_t telemetry_history_json(char *buffer, size_t length)
{
    if (!buffer || length == 0 || !s_lock) return 0;
    size_t offset = 0;
    offset += (size_t)snprintf(buffer + offset, length - offset, "[");
    if (xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE) {
        const size_t first = s_count == TELEMETRY_HISTORY_SIZE ? s_next : 0;
        for (size_t i = 0; i < s_count && offset < length; ++i) {
            const telemetry_snapshot_t *s = &s_history[(first + i) % TELEMETRY_HISTORY_SIZE];
            offset += (size_t)snprintf(buffer + offset, length - offset,
                "%s{\"t\":%lu,\"rms\":%.4f,\"peak\":%.4f,\"temp\":%.2f}",
                i ? "," : "", (unsigned long)s->uptime_ms,
                s->vibration_rms_g, s->vibration_peak_g, s->temperature_c);
        }
        xSemaphoreGive(s_lock);
    }
    if (offset < length) offset += (size_t)snprintf(buffer + offset, length - offset, "]");
    return offset < length ? offset : length - 1U;
}
