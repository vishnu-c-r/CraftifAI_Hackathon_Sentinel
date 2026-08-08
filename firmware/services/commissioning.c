#include "commissioning.h"
#include "baseline_learner.h"
#include "logger.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

static const char *TAG = "commission";
static QueueHandle_t s_commands;
static volatile bool s_learning;

static bool command_is_valid(const char *line)
{
    return strcmp(line, "START_BASELINE") == 0 || strcmp(line, "STOP_BASELINE") == 0 ||
           strcmp(line, "RESET_BASELINE") == 0 || strcmp(line, "BASELINE_STATUS") == 0 ||
           strcmp(line, "HELP") == 0;
}

esp_err_t commissioning_dispatch_command(const char *command)
{
    if (!command || !command_is_valid(command)) return ESP_ERR_INVALID_ARG;
    if (strcmp(command, "START_BASELINE") == 0) {
        baseline_learner_set_learning(true);
        s_learning = true;
        ESP_LOGI(TAG, "baseline learning requested");
    } else if (strcmp(command, "STOP_BASELINE") == 0) {
        baseline_learner_set_learning(false);
        s_learning = false;
        ESP_LOGI(TAG, "baseline learning stopped");
    } else if (strcmp(command, "RESET_BASELINE") == 0) {
        baseline_learner_reset();
        s_learning = false;
        ESP_LOGI(TAG, "baseline reset requested");
    } else if (strcmp(command, "BASELINE_STATUS") == 0) {
        commissioning_status_t status;
        commissioning_get_status(&status);
        ESP_LOGI(TAG, "baseline learning=%d valid=%d cycles=%u",
                 status.learning, status.baseline_valid, status.cycles);
    } else {
        ESP_LOGI(TAG, "commands: START_BASELINE STOP_BASELINE RESET_BASELINE BASELINE_STATUS HELP");
    }
    return ESP_OK;
}

void commissioning_get_status(commissioning_status_t *status)
{
    if (!status) return;
    status->learning = s_learning;
    status->baseline_valid = baseline_learner_is_valid();
    status->cycles = baseline_learner_cycles();
}

static void command_task(void *arg)
{
    (void)arg;
    char line[COMMISSIONING_COMMAND_MAX];
    size_t length = 0;
    for (;;) {
        int c = getchar();
        if (c == EOF) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
        if (c == '\r' || c == '\n') {
            line[length] = '\0';
            if (length > 0) {
                if (commissioning_dispatch_command(line) != ESP_OK) {
                    ESP_LOGW(TAG, "unknown command: %s", line);
                }
            }
            length = 0;
        } else if (length + 1 < sizeof(line)) {
            line[length++] = (char)c;
        } else {
            length = 0;
            ESP_LOGW(TAG, "command too long");
        }
    }
}

void commissioning_start(void)
{
    s_commands = xQueueCreate(4, sizeof(uint32_t));
    if (s_commands != NULL) {
        xTaskCreate(command_task, "commission", 3072, NULL, 3, NULL);
        ESP_LOGI(TAG, "commands ready");
    }
}
