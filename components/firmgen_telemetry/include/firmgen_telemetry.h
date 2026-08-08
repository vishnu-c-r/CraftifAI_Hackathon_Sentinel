/**
 * firmgen_telemetry.h — Firmgen live RTOS telemetry component
 *
 * Emits a JSON line over UART stdout every 500 ms that the Firmgen
 * backend can parse and forward to the Live Intelligence Panel.
 *
 * Minimum usage (call once from app_main, after scheduler starts):
 *   #include "firmgen_telemetry.h"
 *   firmgen_telemetry_start();
 *
 * CPU usage (optional):
 *   Enable CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y in menuconfig
 *   (Component config → FreeRTOS → Enable FreeRTOS trace facility +
 *    Enable FreeRTOS to collect run time stats).
 *   When enabled, the "cpu" field reports real busy-%. Without it, "cpu"
 *   is always 0 — all other metrics are always live.
 *
 * Task count:
 *   Requires CONFIG_FREERTOS_USE_TRACE_FACILITY=y (auto-set when runtime
 *   stats are enabled). Without it, "tasks" reports 0.
 *
 * Works on: ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6, ESP32-H2,
 *            ESP32-P4 — any target supported by ESP-IDF v5.x.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the background telemetry task.
 * Idempotent — safe to call more than once; only the first call has effect.
 * Must be called after the FreeRTOS scheduler is running (i.e. from app_main
 * or any task, NOT from a constructor or early-init hook).
 */
void firmgen_telemetry_start(void);

#ifdef __cplusplus
}
#endif
