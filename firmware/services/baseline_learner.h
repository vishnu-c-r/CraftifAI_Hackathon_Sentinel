#ifndef BASELINE_LEARNER_H
#define BASELINE_LEARNER_H

#include <stdbool.h>
#include <stdint.h>
#include "operating_state.h"
#include "esp_err.h"

typedef struct {
    float mean_g;
    float min_g;
    float max_g;
    float m2_g;
    uint32_t windows;
} baseline_phase_stats_t;

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t cycles;
    baseline_phase_stats_t phase[5];
    float warning_g;
    float critical_g;
    uint32_t crc;
} baseline_record_t;

void baseline_learner_init(uint32_t required_cycles);
void baseline_learner_observe(sentinel_state_t state, float rms_g);
void baseline_learner_cycle_complete(void);
void baseline_learner_reset(void);
bool baseline_learner_is_learning(void);
void baseline_learner_set_learning(bool enabled);
bool baseline_learner_is_valid(void);
uint32_t baseline_learner_cycles(void);
float baseline_learner_warning_g(void);
float baseline_learner_critical_g(void);
esp_err_t baseline_learner_load(void);
esp_err_t baseline_learner_save(void);
const baseline_phase_stats_t *baseline_learner_stats(sentinel_state_t state);

#endif /* BASELINE_LEARNER_H */
