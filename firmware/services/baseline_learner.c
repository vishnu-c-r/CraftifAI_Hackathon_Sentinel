#include "baseline_learner.h"
#include "app_config.h"
#include "logger.h"

#include <float.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include "nvs.h"
#include "nvs_flash.h"

#define BASELINE_MAGIC 0x53424C31U
#define BASELINE_VERSION 1U
#define BASELINE_NAMESPACE "sentinel"
#define BASELINE_KEY "baseline"

static baseline_phase_stats_t s_stats[5];
static uint32_t s_required_cycles;
static uint32_t s_cycles;
static bool s_valid;
static bool s_learning;
static float s_warning_g;
static float s_critical_g;
static sentinel_state_t s_last_state;
static uint8_t s_sequence_index;
static nvs_handle_t s_nvs;

static uint32_t checksum(const void *data, size_t length)
{
    const uint8_t *bytes = data;
    uint32_t value = 2166136261U;
    for (size_t i = 0; i < length; ++i) value = (value ^ bytes[i]) * 16777619U;
    return value;
}

static void calculate_thresholds(void)
{
    const baseline_phase_stats_t *running = &s_stats[SENTINEL_STATE_RUNNING];
    const float variance = running->windows > 1 ? running->m2_g / (running->windows - 1U) : 0.0f;
    const float sigma = sqrtf(fmaxf(0.0f, variance));
    s_warning_g = fmaxf(0.08f, running->mean_g + 3.0f * sigma);
    s_critical_g = fmaxf(0.30f, running->mean_g + 5.0f * sigma);
    if (s_critical_g <= s_warning_g) s_critical_g = s_warning_g + 0.10f;
}

void baseline_learner_init(uint32_t required_cycles)
{
    s_required_cycles = required_cycles;
    s_cycles = 0; s_valid = false; s_learning = false;
    s_warning_g = 0.08f; s_critical_g = 0.30f;
    s_last_state = SENTINEL_STATE_OFF; s_sequence_index = 0;
    memset(s_stats, 0, sizeof(s_stats));
    for (size_t i = 0; i < 5; ++i) s_stats[i].min_g = FLT_MAX;
    if (nvs_flash_init() == ESP_OK && nvs_open(BASELINE_NAMESPACE, NVS_READWRITE, &s_nvs) == ESP_OK) {
        (void)baseline_learner_load();
    }
}

void baseline_learner_observe(sentinel_state_t state, float rms_g)
{
    if (!s_learning || s_valid || !isfinite(rms_g) || state > SENTINEL_STATE_FAULT) return;
    baseline_phase_stats_t *stats = &s_stats[state];
    ++stats->windows;
    const float delta = rms_g - stats->mean_g;
    stats->mean_g += delta / (float)stats->windows;
    stats->m2_g += delta * (rms_g - stats->mean_g);
    if (rms_g < stats->min_g) stats->min_g = rms_g;
    if (rms_g > stats->max_g) stats->max_g = rms_g;
}

void baseline_learner_cycle_complete(void)
{
    if (s_learning && !s_valid && ++s_cycles >= s_required_cycles) {
        calculate_thresholds();
        s_valid = baseline_learner_save() == ESP_OK;
        s_learning = false;
    }
}

static void observe_sequence(sentinel_state_t state)
{
    static const sentinel_state_t sequence[] = {SENTINEL_STATE_OFF, SENTINEL_STATE_STARTING,
                                                 SENTINEL_STATE_RUNNING, SENTINEL_STATE_STOPPING,
                                                 SENTINEL_STATE_OFF};
    if (state == SENTINEL_STATE_FAULT) { s_sequence_index = 0; return; }
    if (state == sequence[s_sequence_index + 1U]) {
        ++s_sequence_index;
        if (s_sequence_index == 4U) { baseline_learner_cycle_complete(); s_sequence_index = 0; }
    } else if (state != sequence[s_sequence_index]) s_sequence_index = 0;
    s_last_state = state;
}

void baseline_learner_reset(void)
{
    s_learning = false; s_valid = false; s_cycles = 0; s_sequence_index = 0;
    memset(s_stats, 0, sizeof(s_stats));
    if (s_nvs) { nvs_erase_key(s_nvs, BASELINE_KEY); nvs_commit(s_nvs); }
}
void baseline_learner_set_learning(bool enabled) { s_learning = enabled; if (enabled) s_valid = false; }
bool baseline_learner_is_learning(void) { return s_learning; }
bool baseline_learner_is_valid(void) { return s_valid; }
uint32_t baseline_learner_cycles(void) { return s_cycles; }
float baseline_learner_warning_g(void) { return s_warning_g; }
float baseline_learner_critical_g(void) { return s_critical_g; }

esp_err_t baseline_learner_load(void)
{
    baseline_record_t record;
    size_t length = sizeof(record);
    esp_err_t err = nvs_get_blob(s_nvs, BASELINE_KEY, &record, &length);
    if (err != ESP_OK || length != sizeof(record) || record.magic != BASELINE_MAGIC ||
        record.version != BASELINE_VERSION || record.crc != checksum(&record, offsetof(baseline_record_t, crc))) {
        return ESP_ERR_INVALID_CRC;
    }
    memcpy(s_stats, record.phase, sizeof(s_stats));
    s_cycles = record.cycles; s_warning_g = record.warning_g; s_critical_g = record.critical_g; s_valid = true;
    return ESP_OK;
}

esp_err_t baseline_learner_save(void)
{
    baseline_record_t record = {.magic = BASELINE_MAGIC, .version = BASELINE_VERSION,
                                .cycles = (uint16_t)s_cycles, .warning_g = s_warning_g,
                                .critical_g = s_critical_g};
    memcpy(record.phase, s_stats, sizeof(s_stats));
    record.crc = checksum(&record, offsetof(baseline_record_t, crc));
    esp_err_t err = nvs_set_blob(s_nvs, BASELINE_KEY, &record, sizeof(record));
    if (err == ESP_OK) err = nvs_commit(s_nvs);
    return err;
}

const baseline_phase_stats_t *baseline_learner_stats(sentinel_state_t state)
{
    return state <= SENTINEL_STATE_FAULT ? &s_stats[state] : NULL;
}
