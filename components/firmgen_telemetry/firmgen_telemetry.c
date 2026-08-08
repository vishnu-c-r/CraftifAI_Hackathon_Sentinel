/*
 * firmgen_telemetry.c — single-file Firmgen live RTOS telemetry component.
 *
 * Spawns one FreeRTOS task that samples system metrics every 500 ms and
 * emits a single JSON v1 line to stdout (UART0):
 *
 *   {"t":"rtos","v":1,"cpu":<n>,"tasks":<n>,
 *    "heap":{"free":<b>,"min":<b>,"total":<b>},
 *    "stack_peak":<n>,"uptime":<s>
 *    [,"psram":{"present":<bool>,"free":<b>,"total":<b>}]
 *    [,"health":{"temp_c":<f>}]
 *   }
 *
 * PSRAM section appears only on chips with SPIRAM enabled.
 * Health section appears only on chips with a temperature sensor.
 * Compatible with ESP32, S2, S3, C3, C6, H2, P4 on IDF v5.x.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "firmgen_telemetry.h"

/* ── Tunables ─────────────────────────────────────────────────────────────── */

#define FG_VERSION      1u
/* Must cover the deep printf→newlib-buffered-flush→VFS→UART path. 3072 was marginal (~67% peak)
 * and the sibling proto-monitor emitter crashed at 3072 on larger batches — bumped for headroom. */
#define FG_STACK_BYTES  6144u
#define FG_STACK_WORDS  (FG_STACK_BYTES / sizeof(StackType_t))
#define FG_INTERVAL_MS  500u
#define FG_PRIORITY     1u

/* ── Internal metric types ────────────────────────────────────────────────── */

typedef struct {
    uint32_t cpu_pct;
    uint32_t tasks;
    size_t   free_heap;
    size_t   min_heap;
    size_t   total_heap;
    uint32_t stack_pct;
    uint64_t uptime_s;
} fg_core_t;

typedef struct {
    bool   present;
    size_t free;
    size_t total;
} fg_psram_t;

typedef struct {
    bool  valid;
    float temp_c;
} fg_health_t;

/* ── CPU delta sampling ───────────────────────────────────────────────────── */

#if CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS

static uint32_t s_prev_idle  = 0;
static uint32_t s_prev_total = 0;

static uint32_t sample_cpu_pct(void)
{
    uint32_t n   = uxTaskGetNumberOfTasks();
    TaskStatus_t *buf = pvPortMalloc(n * sizeof(TaskStatus_t));
    if (!buf) return 0u;

    uint32_t total_ticks = 0u;
    uint32_t found = uxTaskGetSystemState(buf, n, &total_ticks);

    /* Sum idle task counters (IDF names: "IDLE", "IDLE0", "IDLE1"). */
    uint32_t idle_ticks = 0u;
    for (uint32_t i = 0u; i < found; i++) {
        const char *nm = buf[i].pcTaskName;
        if (nm[0]=='I' && nm[1]=='D' && nm[2]=='L' && nm[3]=='E') {
            idle_ticks += buf[i].ulRunTimeCounter;
        }
    }
    vPortFree(buf);

    uint32_t d_total = total_ticks - s_prev_total;
    uint32_t d_idle  = idle_ticks  - s_prev_idle;
    s_prev_total     = total_ticks;
    s_prev_idle      = idle_ticks;

    if (d_total == 0u) return 0u;
    uint32_t idle_pct = (d_idle * 100u) / d_total;
    return (idle_pct > 100u) ? 0u : (100u - idle_pct);
}

#else
static uint32_t sample_cpu_pct(void) { return 0u; }
#endif

/* ── Core metrics ─────────────────────────────────────────────────────────── */

static void core_sample(fg_core_t *out, UBaseType_t hwm)
{
    out->total_heap = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    out->free_heap  = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    out->min_heap   = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    out->cpu_pct    = sample_cpu_pct();
    out->uptime_s   = (uint64_t)(esp_timer_get_time() / 1000000ULL);

    uint32_t used  = (FG_STACK_WORDS > (uint32_t)hwm) ? (FG_STACK_WORDS - (uint32_t)hwm) : 0u;
    out->stack_pct = (FG_STACK_WORDS > 0u) ? ((used * 100u) / FG_STACK_WORDS) : 0u;
    if (out->stack_pct > 100u) out->stack_pct = 100u;

#if configUSE_TRACE_FACILITY
    out->tasks = (uint32_t)uxTaskGetNumberOfTasks();
#else
    out->tasks = 0u;
#endif
}

/* ── PSRAM metrics ────────────────────────────────────────────────────────── */

static void psram_sample(fg_psram_t *out)
{
#if CONFIG_SPIRAM
    out->present = true;
    out->free    = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    out->total   = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
#else
    out->present = false;
    out->free    = 0;
    out->total   = 0;
#endif
}

/* ── Temperature sensor ───────────────────────────────────────────────────── */

/* Opt-in (Kconfig, default off). The __has_include guard means that even if the
 * option is enabled without esp_driver_tsens on the include path, this compiles out
 * gracefully instead of failing the build with "temperature_sensor.h: No such file". */
#if CONFIG_SOC_TEMP_SENSOR_SUPPORTED && CONFIG_FG_TELEMETRY_TEMP_SENSOR && __has_include("driver/temperature_sensor.h")
#include "driver/temperature_sensor.h"
static temperature_sensor_handle_t s_temp     = NULL;
static bool                        s_temp_init = false;

static bool read_temp(float *out_c)
{
    if (!s_temp_init) {
        s_temp_init = true;
        temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
        if (temperature_sensor_install(&cfg, &s_temp) != ESP_OK) { s_temp = NULL; }
        else { temperature_sensor_enable(s_temp); }
    }
    if (!s_temp) return false;
    return temperature_sensor_get_celsius(s_temp, out_c) == ESP_OK;
}
#else
static bool read_temp(float *out_c) { (void)out_c; return false; }
#endif

static void health_sample(fg_health_t *out)
{
    out->valid  = read_temp(&out->temp_c);
    if (!out->valid) out->temp_c = 0.0f;
}

/* ── JSON emitter ─────────────────────────────────────────────────────────── */

static void emit_json(const fg_core_t *core, const fg_psram_t *psram, const fg_health_t *health)
{
    /* Assemble the whole line in a local buffer and write it with a SINGLE fwrite() so a
     * concurrent ESP_LOG can't splice into the middle of the JSON (each stdio call is
     * atomic under its own FILE lock). We deliberately do NOT hold flockfile across
     * multiple printf calls — that deadlocked the device at Wi-Fi connect (this task and
     * the proto-monitor task would hold the stdout FILE lock while the Wi-Fi driver
     * blocked on it, wedging the whole system with no panic — the task WDT is disabled).
     * See firmgen_proto_monitor.c for the full rationale. */
    char   line[320];
    size_t off = 0;
    int n = snprintf(line, sizeof(line),
           "{\"t\":\"rtos\",\"v\":%u,\"cpu\":%lu,\"tasks\":%lu,"
           "\"heap\":{\"free\":%lu,\"min\":%lu,\"total\":%lu},"
           "\"stack_peak\":%lu,\"uptime\":%llu",
           (unsigned)FG_VERSION,
           (unsigned long)core->cpu_pct,
           (unsigned long)core->tasks,
           (unsigned long)core->free_heap,
           (unsigned long)core->min_heap,
           (unsigned long)core->total_heap,
           (unsigned long)core->stack_pct,
           (unsigned long long)core->uptime_s);
    if (n < 0) return;
    off = ((size_t)n < sizeof(line)) ? (size_t)n : sizeof(line) - 1u;

    if (psram) {
        int m = snprintf(line + off, sizeof(line) - off,
               ",\"psram\":{\"present\":%s,\"free\":%lu,\"total\":%lu}",
               psram->present ? "true" : "false",
               (unsigned long)psram->free,
               (unsigned long)psram->total);
        if (m > 0) off += ((size_t)m < sizeof(line) - off) ? (size_t)m : sizeof(line) - off - 1u;
    }

    if (health && health->valid) {
        int m = snprintf(line + off, sizeof(line) - off,
               ",\"health\":{\"temp_c\":%.1f}", (double)health->temp_c);
        if (m > 0) off += ((size_t)m < sizeof(line) - off) ? (size_t)m : sizeof(line) - off - 1u;
    }

    if (off < sizeof(line) - 2u) { line[off++] = '}'; line[off++] = '\n'; }
    fwrite(line, 1, off, stdout);
    fflush(stdout);
}

/* ── Telemetry task ───────────────────────────────────────────────────────── */

static TaskHandle_t s_handle = NULL;

static void telemetry_task(void *arg)
{
    (void)arg;
    sample_cpu_pct(); /* prime delta counters on first sample */

    for (;;) {
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);

        fg_core_t   core;   core_sample(&core, hwm);
        fg_psram_t  psram;  psram_sample(&psram);
        fg_health_t health; health_sample(&health);

        emit_json(&core, &psram, &health);

        vTaskDelay(pdMS_TO_TICKS(FG_INTERVAL_MS));
    }
}

/* ── Public API ───────────────────────────────────────────────────────────── */

void firmgen_telemetry_start(void)
{
    if (s_handle != NULL) return; /* idempotent */

    BaseType_t ret = xTaskCreate(
        telemetry_task,
        "fg_telemetry",
        FG_STACK_BYTES,
        NULL,
        FG_PRIORITY,
        &s_handle
    );

    if (ret != pdPASS) {
        fprintf(stderr, "firmgen_telemetry: xTaskCreate failed\n");
    }
}
