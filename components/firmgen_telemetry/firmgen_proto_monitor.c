/*
 * firmgen_proto_monitor.c  —  Firmgen Protocol Bus Monitor  (v3)
 *
 * HOW IT WORKS
 * ────────────
 * GNU linker --wrap intercepts ESP-IDF driver calls at link time.  Every call
 * the application makes to functions such as i2c_master_transmit(),
 * uart_write_bytes(), etc. is silently redirected
 * through a __wrap_* shim defined here.  The shim calls the real function,
 * then pushes a compact event record into a thread-safe ring buffer.
 *
 * A low-priority FreeRTOS task drains the ring buffer every 500 ms and emits
 * one JSON line on stdout (UART0):
 *
 *   {"t":"proto","v":2,"ts_ms":<ms>,"evts":[...]}
 *
 * The Rust backend reads that UART stream, parses lines tagged "proto", and
 * stores a rolling 200-event snapshot served via GET /v1/proto_stats.
 *
 * RTOS IMPACT — none
 * ──────────────────
 * • The emitter task runs at priority 1 (lowest user-task priority).
 * • The ring buffer is protected by a portMUX spinlock — no blocking mutexes,
 *   no scheduler suspension, never called from ISR context.
 * • Auto-sampled WiFi RSSI blocks only for the duration of a Wi-Fi driver call
 *   (~few µs) inside the emitter task.
 *
 * PROTOCOLS
 * ─────────
 *   Auto  : I2C, UART (ports 1+), MQTT publish, WiFi RSSI
 *   Manual: MQTT subscribe (call fg_proto_mqtt_sub)
 *
 * ONE CALL TO START:
 *   firmgen_proto_monitor_start();   // call from app_main after scheduler start
 */

#include "firmgen_proto_monitor.h"
#include "sdkconfig.h"        /* CONFIG_FG_PROTO_MONITOR */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

/* ── Optional driver headers (detected at compile time) ─────────────────── */

/* Wi-Fi telemetry must be gated on the SoC actually HAVING a Wi-Fi radio, not merely on the
 * header being reachable. On Wi-Fi-less targets (e.g. ESP32-H2, which is 802.15.4 / BLE only)
 * esp_wifi.h is STILL on the include path, so a header-only check turns the shim ON and the link
 * then fails with "undefined reference to esp_wifi_sta_get_ap_info". SOC_WIFI_SUPPORTED
 * (soc/soc_caps.h) is absent/0 on those targets, so the shim correctly compiles out. */
#if __has_include("soc/soc_caps.h")
#  include "soc/soc_caps.h"
#endif
#if defined(SOC_WIFI_SUPPORTED) && SOC_WIFI_SUPPORTED && __has_include("esp_wifi.h")
#  include "esp_wifi.h"
#  define FGP_HAVE_WIFI 1
#else
#  define FGP_HAVE_WIFI 0
#endif

#if __has_include("driver/i2c_master.h")
#  include "driver/i2c_master.h"
#  define FGP_HAVE_I2C_NEW 1
#else
#  define FGP_HAVE_I2C_NEW 0
#endif

#if __has_include("driver/i2c.h")
#  include "driver/i2c.h"
#  define FGP_HAVE_I2C_OLD 1
#else
#  define FGP_HAVE_I2C_OLD 0
#endif

#if __has_include("driver/uart.h")
#  include "driver/uart.h"
#  define FGP_HAVE_UART 1
#else
#  define FGP_HAVE_UART 0
#endif

/*
 * FGP_HAVE_MQTT priority:
 *   1. CMake sets it via target_compile_definitions(-DFGP_HAVE_MQTT=1)
 *      when CONFIG_MQTT_PROTOCOL_311/5 is detected.  This is always in sync
 *      with the --wrap linker flags so the two can never diverge.
 *   2. Fall back to __has_include when CMake didn't set it (e.g. manual build).
 *
 * CRITICAL: --wrap flags and FGP_HAVE_MQTT MUST match.
 *   --wrap set + FGP_HAVE_MQTT=0 → linker error (undefined __wrap_* symbols).
 *   --wrap set + FGP_HAVE_MQTT=1 → correct — __wrap_* functions compiled in.
 */
#ifndef FGP_HAVE_MQTT
#  if __has_include("mqtt_client.h")
#    define FGP_HAVE_MQTT 1
#  else
#    define FGP_HAVE_MQTT 0
#  endif
#endif
#if FGP_HAVE_MQTT
#  include "mqtt_client.h"
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 1 — TUNABLES
 * ══════════════════════════════════════════════════════════════════════════ */

#define FGP_RING_SIZE       64u        /* must be a power of 2             */
#define FGP_RING_MASK       (FGP_RING_SIZE - 1u)
#define FGP_STACK_BYTES     6144u      /* emitter task stack. Must cover the DEEP printf→newlib
                                        * buffered-flush→VFS→UART path (_fflush_r/__swbuf_r/__swrite/
                                        * console_write/esp_vfs_write/uart_write). 3072 overflowed on
                                        * MQTT-bearing batches (topic+host+payload hex) → the emitter
                                        * crashed in uart_tx_char and rebooted, so MQTT never showed. */
#define FGP_INTERVAL_MS     500u       /* flush period                     */
#define FGP_PRIORITY        1u         /* lowest user-task priority        */
#define FGP_DATA_MAX        16u        /* max payload bytes captured/event */

/*
 * UART port used by the monitor for its own printf output.
 * Wrap intercepts on this port are SKIPPED to prevent self-capture.
 * Override to UART_NUM_MAX if you want to monitor all ports.
 */
#define FGP_SKIP_UART_PORT  UART_NUM_0

#define FGP_MAX_I2C_BUSES     4
#define FGP_MAX_I2C_DEVS      8
#define FGP_MAX_UART          3
#define FGP_MAX_MQTT_CLIENTS  4
#define FGP_MQTT_HOST_MAX     20   /* max broker hostname chars stored     */
#define FGP_MQTT_TOPIC_MAX    32   /* max MQTT topic chars stored          */

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 2 — INTERNAL TYPES
 * ══════════════════════════════════════════════════════════════════════════ */

typedef enum {
    FGP_I2C = 0,
    FGP_UART,
    FGP_WIFI,
    FGP_MQTT,
} fgp_proto_t;

typedef enum {
    FGP_TX = 0,
    FGP_RX,
    FGP_RSSI,
    FGP_PUB,
    FGP_SUB,
} fgp_evt_t;

typedef struct {
    uint64_t    ts_us;
    fgp_proto_t proto;
    fgp_evt_t   evt;
    uint8_t     gpio[4];          /* protocol-specific GPIO pins          */
    uint8_t     data[FGP_DATA_MAX];
    uint8_t     data_len;
    bool        data_trunc;
    uint8_t     data_rx[FGP_DATA_MAX];
    uint8_t     data_rx_len;
    bool        data_rx_trunc;
    union {
        struct { uint8_t  addr;   uint16_t len; uint8_t ack;         } i2c;
        struct { uint8_t  port;   uint16_t len;                      } uart;
        struct { int8_t   rssi; uint8_t chan;                         } wifi;
        struct {
            uint8_t  qos;
            uint16_t blen;
            uint16_t port;
            char     host[FGP_MQTT_HOST_MAX];    /* broker hostname, null-terminated */
            char     topic[FGP_MQTT_TOPIC_MAX];  /* MQTT topic, null-terminated      */
        } mqtt;
    };
} fgp_entry_t;

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 3 — HANDLE TABLES
 * (Map driver handles back to the GPIO pins configured at init time)
 * ══════════════════════════════════════════════════════════════════════════ */

typedef struct { void *h;   uint8_t sda; uint8_t scl;  }                   fgp_i2c_bus_t;
typedef struct { void *dev; void *bus;   uint16_t addr; }                   fgp_i2c_dev_t;
typedef struct { uint8_t tx; uint8_t rx; uint8_t rts;  uint8_t cts;   }    fgp_uart_pins_t;

static fgp_i2c_bus_t   s_i2c_bus[FGP_MAX_I2C_BUSES];
static fgp_i2c_dev_t   s_i2c_dev[FGP_MAX_I2C_DEVS];
static fgp_uart_pins_t s_uart[FGP_MAX_UART] = {
    {FG_PIN_NONE, FG_PIN_NONE, FG_PIN_NONE, FG_PIN_NONE},
    {FG_PIN_NONE, FG_PIN_NONE, FG_PIN_NONE, FG_PIN_NONE},
    {FG_PIN_NONE, FG_PIN_NONE, FG_PIN_NONE, FG_PIN_NONE},
};

/* MQTT clients: handle -> {broker host, port}
 * Populated by __wrap_esp_mqtt_client_init so publish/subscribe events
 * can carry the broker address without the user doing anything extra. */
typedef struct {
    void    *h;
    char     host[FGP_MQTT_HOST_MAX];
    uint16_t port;
} fgp_mqtt_client_t;
static fgp_mqtt_client_t s_mqtt_clients[FGP_MAX_MQTT_CLIENTS];

/* ── WiFi SSID cache — updated once per flush by auto_sample_wifi ────────── */
#if FGP_HAVE_WIFI
static char s_wifi_ssid[33]; /* null-terminated; empty when not connected */

/* Set while a Wi-Fi scan is in flight. esp_wifi_sta_get_ap_info() MUST NOT run
 * concurrently with esp_wifi_scan_start() — doing so trips an assert inside the
 * Wi-Fi driver (the crash seen when an app scans while telemetry samples RSSI).
 * We track scan state by wrapping the scan APIs (see SECTION 10) and the RSSI
 * sampler skips whenever a scan is active. */
static volatile bool s_wifi_scanning = false;
#endif

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 4 — THREAD-SAFE RING BUFFER
 *
 * portMUX spinlock — interrupt-safe on single and dual-core ESP32 variants.
 * Critical sections are kept as short as possible (single struct copy).
 * ══════════════════════════════════════════════════════════════════════════ */

static portMUX_TYPE    s_ring_mux  = portMUX_INITIALIZER_UNLOCKED;
static fgp_entry_t     s_ring[FGP_RING_SIZE];
static volatile uint32_t s_head   = 0;   /* volatile for lockless peek in emitter */
static volatile uint32_t s_tail   = 0;

static void ring_push(const fgp_entry_t *e) {
    taskENTER_CRITICAL(&s_ring_mux);
    uint32_t next = (s_head + 1u) & FGP_RING_MASK;
    if (next == s_tail) {
        /* Buffer full — drop the oldest event to make room */
        s_tail = (s_tail + 1u) & FGP_RING_MASK;
    }
    s_ring[s_head] = *e;
    s_head = (s_head + 1u) & FGP_RING_MASK;
    taskEXIT_CRITICAL(&s_ring_mux);
}

static bool ring_pop(fgp_entry_t *out) {
    taskENTER_CRITICAL(&s_ring_mux);
    if (s_tail == s_head) {
        taskEXIT_CRITICAL(&s_ring_mux);
        return false;
    }
    *out   = s_ring[s_tail];
    s_tail = (s_tail + 1u) & FGP_RING_MASK;
    taskEXIT_CRITICAL(&s_ring_mux);
    return true;
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 5 — HELPERS
 * ══════════════════════════════════════════════════════════════════════════ */

static void capture(uint8_t *dst, uint8_t *dlen, bool *dtrunc,
                    const void *src, size_t slen) {
    if (!src || !slen) { *dlen = 0; *dtrunc = false; return; }
    uint8_t n = (slen > FGP_DATA_MAX) ? FGP_DATA_MAX : (uint8_t)slen;
    memcpy(dst, src, n);
    *dlen   = n;
    *dtrunc = (slen > FGP_DATA_MAX);
}

/* ── I2C handle lookups ─────────────────────────────────────────────────── */

static void i2c_dev_info(void *dev_h, uint8_t *sda, uint8_t *scl, uint8_t *addr) {
    *sda = *scl = FG_PIN_NONE; *addr = 0xFF;
    for (int i = 0; i < FGP_MAX_I2C_DEVS; i++) {
        if (s_i2c_dev[i].dev != dev_h) continue;
        *addr = (uint8_t)s_i2c_dev[i].addr;
        for (int j = 0; j < FGP_MAX_I2C_BUSES; j++) {
            if (s_i2c_bus[j].h == s_i2c_dev[i].bus) {
                *sda = s_i2c_bus[j].sda;
                *scl = s_i2c_bus[j].scl;
                return;
            }
        }
        return;
    }
}

/* ── MQTT client lookups ─────────────────────────────────────────────────── */

static void mqtt_client_info(void *h, char *host_out, uint16_t *port_out) {
    host_out[0] = '\0'; *port_out = 0;
    for (int i = 0; i < FGP_MAX_MQTT_CLIENTS; i++) {
        if (s_mqtt_clients[i].h == h) {
            /* strncpy + explicit null: safe on all IDF versions */
            strncpy(host_out, s_mqtt_clients[i].host, FGP_MQTT_HOST_MAX - 1);
            host_out[FGP_MQTT_HOST_MAX - 1] = '\0';
            *port_out = s_mqtt_clients[i].port;
            return;
        }
    }
}

static void copy_topic(char *dst, const char *src) {
    if (!src) { dst[0] = '\0'; return; }
    int i;
    for (i = 0; i < (int)(FGP_MQTT_TOPIC_MAX - 1) && src[i]; i++)
        dst[i] = src[i];
    dst[i] = '\0';
}

/* ── Printable-ASCII check ───────────────────────────────────────────────── */

static bool is_printable_ascii(const uint8_t *b, uint8_t n) {
    for (uint8_t i = 0; i < n; i++) {
        if (b[i] < 0x20u || b[i] > 0x7Eu) return false;
    }
    return n > 0;
}

/* ── Atomic line assembly ───────────────────────────────────────────────────
 * The whole batch is rendered into s_line and written with ONE fwrite() (see
 * monitor_task). A single stdio call is atomic under the FILE lock, so a
 * concurrent ESP_LOG lands cleanly BEFORE or AFTER the batch — never spliced
 * into the middle of the JSON. Emitting via many small printf/putchar calls let
 * logs interleave and corrupted every line, so the host parser showed nothing.
 * We must NOT instead hold flockfile across the small writes — that deadlocked
 * at Wi-Fi connect. Only the single emitter task uses this buffer, so one static
 * instance needs no locking. */
#define FGP_LINE_BUF 4096u
static char   s_line[FGP_LINE_BUF];
static size_t s_line_len;

static void ob_reset(void) { s_line_len = 0; }

static void ob_putc(char c) {
    if (s_line_len < FGP_LINE_BUF - 1u) s_line[s_line_len++] = c;
}

static void ob_printf(const char *fmt, ...) {
    if (s_line_len >= FGP_LINE_BUF - 1u) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_line + s_line_len, FGP_LINE_BUF - s_line_len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        s_line_len += (size_t)n;
        if (s_line_len > FGP_LINE_BUF - 1u) s_line_len = FGP_LINE_BUF - 1u; /* truncated */
    }
}

/* ── Safe JSON string printer ────────────────────────────────────────────── */

static void print_json_str(const char *s) {
    /* Escape only " and \ — MQTT topics/hostnames have no other special chars */
    for (; *s; s++) {
        if (*s == '"' || *s == '\\') ob_putc('\\');
        ob_putc(*s);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 6 — JSON EMISSION
 * ══════════════════════════════════════════════════════════════════════════ */

static const char *PROTO_STR[] = {"i2c","uart","wifi","mqtt"};
static const char *EVT_STR[]   = {"tx","rx","rssi","pub","sub"};

static void print_hex(const uint8_t *b, uint8_t n) {
    static const char H[] = "0123456789ABCDEF";
    for (uint8_t i = 0; i < n; i++) {
        ob_putc(H[b[i] >> 4]);
        ob_putc(H[b[i] & 0xF]);
    }
}

/*
 * Emit the optional "gpio" JSON object.
 * Pass NULL for key names that are not applicable to the protocol.
 * Pins equal to FG_PIN_NONE are omitted automatically.
 */
static void emit_gpio(const char *k0, const char *k1,
                      const char *k2, const char *k3,
                      const uint8_t p[4]) {
    const char *keys[4] = {k0, k1, k2, k3};
    bool any = false;
    for (int i = 0; i < 4; i++) {
        if (keys[i] && p[i] != FG_PIN_NONE) { any = true; break; }
    }
    if (!any) return;
    ob_printf(",\"gpio\":{");
    bool first = true;
    for (int i = 0; i < 4; i++) {
        if (!keys[i] || p[i] == FG_PIN_NONE) continue;
        ob_printf("%s\"%s\":%u", first ? "" : ",", keys[i], p[i]);
        first = false;
    }
    ob_putc('}');
}

static void emit_entry(const fgp_entry_t *e, uint64_t t0, bool first_in_batch) {
    if (!first_in_batch) ob_putc(',');
    uint64_t us = (e->ts_us >= t0) ? (e->ts_us - t0) : 0u;

    /* "t" = protocol type, "e" = event direction — matches outer schema key */
    ob_printf("{\"t\":\"%s\",\"e\":\"%s\"", PROTO_STR[e->proto], EVT_STR[e->evt]);

    switch (e->proto) {
    case FGP_I2C:
        ob_printf(",\"addr\":%u,\"len\":%u,\"ack\":%u",
               e->i2c.addr, (unsigned)e->i2c.len, e->i2c.ack);
        emit_gpio("sda", "scl", NULL, NULL, e->gpio);
        break;

    case FGP_UART:
        ob_printf(",\"port\":%u,\"len\":%u", e->uart.port, (unsigned)e->uart.len);
        emit_gpio("tx", "rx", "rts", "cts", e->gpio);
        break;

    case FGP_WIFI:
        ob_printf(",\"rssi\":%d", (int)e->wifi.rssi);
        /* channel — 0 means unknown (manually-logged events) */
        if (e->wifi.chan) ob_printf(",\"chan\":%u", e->wifi.chan);
        /* SSID from last successful esp_wifi_sta_get_ap_info() call.
         * WiFi password is NOT accessible via the IDF API for security reasons. */
#if FGP_HAVE_WIFI
        if (s_wifi_ssid[0]) {
            ob_printf(",\"ssid\":\"");
            print_json_str(s_wifi_ssid);
            ob_putc('"');
        }
#endif
        break;

    case FGP_MQTT:
        ob_printf(",\"qos\":%u", e->mqtt.qos);
        if (e->mqtt.blen) ob_printf(",\"blen\":%u", (unsigned)e->mqtt.blen);
        if (e->mqtt.topic[0]) {
            ob_printf(",\"topic\":\""); print_json_str(e->mqtt.topic); ob_putc('"');
        }
        if (e->mqtt.host[0]) {
            ob_printf(",\"host\":\""); print_json_str(e->mqtt.host); ob_putc('"');
        }
        if (e->mqtt.port) ob_printf(",\"port\":%u", (unsigned)e->mqtt.port);
        break;
    }

    /* TX payload — hex always present; "str" added when printable ASCII */
    if (e->data_len) {
        ob_printf(",\"data\":\"");
        print_hex(e->data, e->data_len);
        ob_putc('"');
        if (is_printable_ascii(e->data, e->data_len)) {
            ob_printf(",\"str\":\"");
            for (int si = 0; si < (int)e->data_len; si++) {
                if (e->data[si] == '"' || e->data[si] == '\\') ob_putc('\\');
                ob_putc((char)e->data[si]);
            }
            ob_putc('"');
        }
        if (e->data_trunc) ob_printf(",\"trunc\":1");
    }

    /* RX payload (I2C transmit_receive response) */
    if (e->data_rx_len) {
        ob_printf(",\"data_rx\":\"");
        print_hex(e->data_rx, e->data_rx_len);
        ob_putc('"');
        if (is_printable_ascii(e->data_rx, e->data_rx_len)) {
            ob_printf(",\"str_rx\":\"");
            for (int si = 0; si < (int)e->data_rx_len; si++) {
                if (e->data_rx[si] == '"' || e->data_rx[si] == '\\') ob_putc('\\');
                ob_putc((char)e->data_rx[si]);
            }
            ob_putc('"');
        }
        if (e->data_rx_trunc) ob_printf(",\"trunc_rx\":1");
    }

    ob_printf(",\"us\":%llu}", (unsigned long long)us);
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 7 — AUTO WiFi RSSI SAMPLING
 * ══════════════════════════════════════════════════════════════════════════ */

static void auto_sample_wifi(void) {
#if FGP_HAVE_WIFI
    /* Safety: never query the STA while a scan is running — esp_wifi_sta_get_ap_info()
     * concurrent with esp_wifi_scan_start() asserts inside the driver. When the STA is
     * simply not connected (no scan), get_ap_info returns an error, which we handle
     * cleanly below. This guard is what makes telemetry safe in a WiFi-scanning app. */
    if (s_wifi_scanning) {
        return;
    }
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) != ESP_OK) {
        s_wifi_ssid[0] = '\0';   /* clear stale SSID on disconnect */
        return;
    }
    /* Cache SSID for emit_entry() — ssid is uint8_t[33], already null-terminated */
    memcpy(s_wifi_ssid, ap.ssid, sizeof(ap.ssid));
    s_wifi_ssid[32] = '\0';      /* guarantee null-termination */

    fgp_entry_t e;
    memset(&e, 0, sizeof(e));
    e.ts_us     = (uint64_t)esp_timer_get_time();
    e.proto     = FGP_WIFI;
    e.evt       = FGP_RSSI;
    e.wifi.rssi = ap.rssi;
    e.wifi.chan = ap.primary;    /* 2.4 GHz channel (1-13) or 5 GHz channel */
    memset(e.gpio, FG_PIN_NONE, 4);
    ring_push(&e);
#endif
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 8 — BACKGROUND EMITTER TASK
 * ══════════════════════════════════════════════════════════════════════════ */

static TaskHandle_t s_task = NULL;

static void monitor_task(void *arg) {
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(FGP_INTERVAL_MS));
        auto_sample_wifi();

        /* Always emit a batch — even when evts:[] — so the serial monitor and
         * backend can see the component is alive and not just waiting for traffic.
         * The backend parser treats an empty evts array as a no-op snapshot update. */
        uint64_t t0    = (uint64_t)esp_timer_get_time();
        uint64_t ts_ms = t0 / 1000ULL;

        /* Render the whole batch into s_line, then write it with a SINGLE fwrite() so no
         * concurrent ESP_LOG can splice into the middle of the JSON (each stdio call is
         * atomic under the FILE lock — logs land cleanly before or after, never inside).
         * We do NOT hold flockfile across the batch: that deadlocked the device at Wi-Fi
         * connect (the prio-1 emitter held the stdout FILE lock while the Wi-Fi driver
         * blocked on it, wedging the system with no panic — the task WDT is disabled). */
        ob_reset();
        ob_printf("{\"p\":\"proto_monitor\",\"v\":3,\"ts_ms\":%llu,\"evts\":[",
                  (unsigned long long)ts_ms);

        fgp_entry_t e;
        bool first = true;
        while (ring_pop(&e)) {
            /* Bandwidth guard. A fast peripheral (e.g. a 100 Hz I2C sensor loop) can push
             * far more events per flush than the UART can carry; unchecked it fills the
             * line buffer with I2C and STARVES the rarer-but-important WiFi RSSI / MQTT /
             * UART events, so those tabs stay empty. When the buffer gets tight, shed
             * further I2C but keep emitting the other protocols so every tab populates.
             * ob_* clamp internally, but we stop before a partial event so the JSON stays
             * valid (a shorter valid batch beats a truncated, unparseable one). */
            if (s_line_len > FGP_LINE_BUF - 512u && e.proto == FGP_I2C) continue;
            if (s_line_len > FGP_LINE_BUF - 256u) continue;
            emit_entry(&e, t0, first);
            first = false;
        }

        ob_printf("]}\n");
        fwrite(s_line, 1, s_line_len, stdout);
        fflush(stdout);
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 9 — PUBLIC START
 * ══════════════════════════════════════════════════════════════════════════ */

void firmgen_proto_monitor_start(void) {
    if (s_task) return;   /* idempotent */

#if CONFIG_FG_PROTO_MONITOR
    /*
     * Boot-time diagnostic: confirms which protocols have active --wrap shims.
     * Check this line in the serial log after flashing.
     * If mqtt=0, the CMakeLists.txt CONFIG_MQTT_PROTOCOL_311 block didn't fire
     * or FGP_HAVE_MQTT was not set — run `idf.py fullclean build` to fix.
     */
    ESP_LOGI("fg_proto",
             "starting — i2c=%d uart=%d wifi=%d mqtt=%d "
             "(1=active 0=disabled)",
             (FGP_HAVE_I2C_NEW || FGP_HAVE_I2C_OLD),
             FGP_HAVE_UART, FGP_HAVE_WIFI, FGP_HAVE_MQTT);

    BaseType_t ret = xTaskCreate(monitor_task, "fg_proto",
                                  FGP_STACK_BYTES, NULL, FGP_PRIORITY, &s_task);
    if (ret != pdPASS) {
        s_task = NULL;
        fprintf(stderr, "firmgen_proto_monitor: xTaskCreate failed\n");
    }
#else
    /* Disabled at build time via CONFIG_FG_PROTO_MONITOR=n: the emitter task is never
     * created — no extra task, no serial emission, no WiFi sampling. The --wrap shims
     * stay linked (harmless: they only push into a never-drained ring) so the build
     * still succeeds. `(void)monitor_task` keeps the task fn (and the helpers it calls)
     * referenced so disabling the monitor does not trigger unused-function warnings. */
    ESP_LOGD("fg_proto", "protocol monitor disabled (CONFIG_FG_PROTO_MONITOR=n)");
    (void)monitor_task;
#endif
}

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 10 — LINKER WRAP FUNCTIONS
 *
 * Each shim calls the real driver function first, then records the event.
 * The application's return value is always preserved unchanged.
 * ══════════════════════════════════════════════════════════════════════════ */

/* ── I2C — new master API (IDF v5.x) ────────────────────────────────────── */
#if FGP_HAVE_I2C_NEW

esp_err_t __real_i2c_new_master_bus(const i2c_master_bus_config_t *, i2c_master_bus_handle_t *);
esp_err_t __wrap_i2c_new_master_bus(const i2c_master_bus_config_t *cfg,
                                     i2c_master_bus_handle_t *ret) {
    esp_err_t r = __real_i2c_new_master_bus(cfg, ret);
    if (r == ESP_OK && ret) {
        for (int i = 0; i < FGP_MAX_I2C_BUSES; i++) {
            if (!s_i2c_bus[i].h) {
                s_i2c_bus[i] = (fgp_i2c_bus_t){
                    *ret, (uint8_t)cfg->sda_io_num, (uint8_t)cfg->scl_io_num
                };
                break;
            }
        }
    }
    return r;
}

esp_err_t __real_i2c_master_bus_add_device(i2c_master_bus_handle_t,
                                            const i2c_device_config_t *,
                                            i2c_master_dev_handle_t *);
esp_err_t __wrap_i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
                                            const i2c_device_config_t *dev_cfg,
                                            i2c_master_dev_handle_t *ret) {
    esp_err_t r = __real_i2c_master_bus_add_device(bus, dev_cfg, ret);
    if (r == ESP_OK && ret) {
        for (int i = 0; i < FGP_MAX_I2C_DEVS; i++) {
            if (!s_i2c_dev[i].dev) {
                s_i2c_dev[i] = (fgp_i2c_dev_t){ *ret, bus, dev_cfg->device_address };
                break;
            }
        }
    }
    return r;
}

esp_err_t __real_i2c_master_transmit(i2c_master_dev_handle_t,
                                      const uint8_t *, size_t, int);
esp_err_t __wrap_i2c_master_transmit(i2c_master_dev_handle_t dev,
                                      const uint8_t *buf, size_t len, int tmo) {
    esp_err_t r = __real_i2c_master_transmit(dev, buf, len, tmo);
    uint8_t sda, scl, addr;
    i2c_dev_info(dev, &sda, &scl, &addr);
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_I2C; e.evt = FGP_TX;
    e.gpio[0] = sda; e.gpio[1] = scl; e.gpio[2] = e.gpio[3] = FG_PIN_NONE;
    e.i2c.addr = addr;
    e.i2c.len  = (uint16_t)(len > 0xFFFFu ? 0xFFFFu : len);
    e.i2c.ack  = (r == ESP_OK) ? 1 : 0;
    capture(e.data, &e.data_len, &e.data_trunc, buf, len);
    ring_push(&e);
    return r;
}

esp_err_t __real_i2c_master_receive(i2c_master_dev_handle_t,
                                     uint8_t *, size_t, int);
esp_err_t __wrap_i2c_master_receive(i2c_master_dev_handle_t dev,
                                     uint8_t *buf, size_t len, int tmo) {
    esp_err_t r = __real_i2c_master_receive(dev, buf, len, tmo);
    uint8_t sda, scl, addr;
    i2c_dev_info(dev, &sda, &scl, &addr);
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_I2C; e.evt = FGP_RX;
    e.gpio[0] = sda; e.gpio[1] = scl; e.gpio[2] = e.gpio[3] = FG_PIN_NONE;
    e.i2c.addr = addr;
    e.i2c.len  = (uint16_t)(len > 0xFFFFu ? 0xFFFFu : len);
    e.i2c.ack  = (r == ESP_OK) ? 1 : 0;
    capture(e.data, &e.data_len, &e.data_trunc, buf, len);
    ring_push(&e);
    return r;
}

esp_err_t __real_i2c_master_transmit_receive(i2c_master_dev_handle_t,
                                              const uint8_t *, size_t,
                                              uint8_t *, size_t, int);
esp_err_t __wrap_i2c_master_transmit_receive(i2c_master_dev_handle_t dev,
                                              const uint8_t *wbuf, size_t wlen,
                                              uint8_t *rbuf, size_t rlen, int tmo) {
    esp_err_t r = __real_i2c_master_transmit_receive(dev, wbuf, wlen, rbuf, rlen, tmo);
    uint8_t sda, scl, addr;
    i2c_dev_info(dev, &sda, &scl, &addr);
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_I2C; e.evt = FGP_TX;   /* TX = write phase, data_rx = read phase */
    e.gpio[0] = sda; e.gpio[1] = scl; e.gpio[2] = e.gpio[3] = FG_PIN_NONE;
    e.i2c.addr = addr;
    e.i2c.len  = (uint16_t)(wlen > 0xFFFFu ? 0xFFFFu : wlen);
    e.i2c.ack  = (r == ESP_OK) ? 1 : 0;
    capture(e.data,    &e.data_len,    &e.data_trunc,    wbuf, wlen);
    capture(e.data_rx, &e.data_rx_len, &e.data_rx_trunc, rbuf, rlen);
    ring_push(&e);
    return r;
}

#endif /* FGP_HAVE_I2C_NEW */

/*
 * I2C — legacy API (IDF v4.x / projects that don't use the new master driver).
 *
 * CRITICAL: Do NOT compile this block when the new master API is present.
 * On IDF v5.x both headers coexist, but linking __real_i2c_master_write_to_device
 * pulls in the old driver and triggers the "CONFLICT! driver_ng is not allowed
 * to be used with this old driver" abort even if user code never calls it.
 */
#if FGP_HAVE_I2C_OLD && !FGP_HAVE_I2C_NEW

esp_err_t __real_i2c_master_write_to_device(i2c_port_t, uint8_t,
                                              const uint8_t *, size_t, TickType_t);
esp_err_t __wrap_i2c_master_write_to_device(i2c_port_t port, uint8_t addr,
                                              const uint8_t *buf, size_t len,
                                              TickType_t ticks) {
    esp_err_t r = __real_i2c_master_write_to_device(port, addr, buf, len, ticks);
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_I2C; e.evt = FGP_TX;
    memset(e.gpio, FG_PIN_NONE, 4);
    e.i2c.addr = addr;
    e.i2c.len  = (uint16_t)(len > 0xFFFFu ? 0xFFFFu : len);
    e.i2c.ack  = (r == ESP_OK) ? 1 : 0;
    capture(e.data, &e.data_len, &e.data_trunc, buf, len);
    ring_push(&e);
    return r;
}

esp_err_t __real_i2c_master_read_from_device(i2c_port_t, uint8_t,
                                               uint8_t *, size_t, TickType_t);
esp_err_t __wrap_i2c_master_read_from_device(i2c_port_t port, uint8_t addr,
                                               uint8_t *buf, size_t len,
                                               TickType_t ticks) {
    esp_err_t r = __real_i2c_master_read_from_device(port, addr, buf, len, ticks);
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_I2C; e.evt = FGP_RX;
    memset(e.gpio, FG_PIN_NONE, 4);
    e.i2c.addr = addr;
    e.i2c.len  = (uint16_t)(len > 0xFFFFu ? 0xFFFFu : len);
    e.i2c.ack  = (r == ESP_OK) ? 1 : 0;
    capture(e.data, &e.data_len, &e.data_trunc, buf, len);
    ring_push(&e);
    return r;
}

#endif /* FGP_HAVE_I2C_OLD && !FGP_HAVE_I2C_NEW */

/* ── UART ────────────────────────────────────────────────────────────────── */
#if FGP_HAVE_UART

esp_err_t __real_uart_set_pin(uart_port_t, int, int, int, int);
esp_err_t __wrap_uart_set_pin(uart_port_t port, int tx, int rx, int rts, int cts) {
    if ((int)port < FGP_MAX_UART) {
        s_uart[port].tx  = (tx  >= 0) ? (uint8_t)tx  : FG_PIN_NONE;
        s_uart[port].rx  = (rx  >= 0) ? (uint8_t)rx  : FG_PIN_NONE;
        s_uart[port].rts = (rts >= 0) ? (uint8_t)rts : FG_PIN_NONE;
        s_uart[port].cts = (cts >= 0) ? (uint8_t)cts : FG_PIN_NONE;
    }
    return __real_uart_set_pin(port, tx, rx, rts, cts);
}

int __real_uart_write_bytes(uart_port_t, const void *, size_t);
int __wrap_uart_write_bytes(uart_port_t port, const void *src, size_t size) {
    int r = __real_uart_write_bytes(port, src, size);
    /*
     * Skip FGP_SKIP_UART_PORT (UART0 = monitor stdout) to prevent self-capture:
     * the JSON lines emitted by this component would otherwise appear as UART TX
     * events in the next batch.  Set FGP_SKIP_UART_PORT to UART_NUM_MAX in
     * firmgen_proto_monitor.h to monitor all ports including the log port.
     */
    if (r > 0 && (int)port < FGP_MAX_UART && port != FGP_SKIP_UART_PORT) {
        fgp_entry_t e; memset(&e, 0, sizeof(e));
        e.ts_us = (uint64_t)esp_timer_get_time();
        e.proto = FGP_UART; e.evt = FGP_TX;
        e.gpio[0] = s_uart[port].tx;  e.gpio[1] = s_uart[port].rx;
        e.gpio[2] = s_uart[port].rts; e.gpio[3] = s_uart[port].cts;
        e.uart.port = (uint8_t)port;
        e.uart.len  = (uint16_t)(size > 0xFFFFu ? 0xFFFFu : size);
        capture(e.data, &e.data_len, &e.data_trunc, src, size);
        ring_push(&e);
    }
    return r;
}

int __real_uart_read_bytes(uart_port_t, void *, uint32_t, TickType_t);
int __wrap_uart_read_bytes(uart_port_t port, void *buf, uint32_t len, TickType_t ticks) {
    int r = __real_uart_read_bytes(port, buf, len, ticks);
    if (r > 0 && (int)port < FGP_MAX_UART && port != FGP_SKIP_UART_PORT) {
        fgp_entry_t e; memset(&e, 0, sizeof(e));
        e.ts_us = (uint64_t)esp_timer_get_time();
        e.proto = FGP_UART; e.evt = FGP_RX;
        e.gpio[0] = s_uart[port].tx;  e.gpio[1] = s_uart[port].rx;
        e.gpio[2] = s_uart[port].rts; e.gpio[3] = s_uart[port].cts;
        e.uart.port = (uint8_t)port;
        e.uart.len  = (uint16_t)((uint32_t)r > 0xFFFFu ? 0xFFFFu : (uint32_t)r);
        capture(e.data, &e.data_len, &e.data_trunc, buf, (size_t)r);
        ring_push(&e);
    }
    return r;
}

#endif /* FGP_HAVE_UART */

/* ── WiFi scan-state tracking ──────────────────────────────────────────────
 * These shims do NOT record events; they exist solely to know when a scan is in
 * flight so auto_sample_wifi() can skip. Calling esp_wifi_sta_get_ap_info()
 * concurrently with esp_wifi_scan_start() asserts inside the Wi-Fi driver — this
 * is the root cause of the crash when an app scans while telemetry samples RSSI.
 * The real function's behavior and return value are always preserved. */
#if FGP_HAVE_WIFI

esp_err_t __real_esp_wifi_scan_start(const wifi_scan_config_t *, bool);
esp_err_t __wrap_esp_wifi_scan_start(const wifi_scan_config_t *config, bool block) {
    s_wifi_scanning = true;
    esp_err_t r = __real_esp_wifi_scan_start(config, block);
    /* A blocking scan has finished by the time it returns; an async scan stays flagged
     * until the app collects results (scan_get_ap_num) or stops the scan. On error,
     * no scan is running, so clear immediately. */
    if (block || r != ESP_OK) {
        s_wifi_scanning = false;
    }
    return r;
}

esp_err_t __real_esp_wifi_scan_stop(void);
esp_err_t __wrap_esp_wifi_scan_stop(void) {
    esp_err_t r = __real_esp_wifi_scan_stop();
    s_wifi_scanning = false;
    return r;
}

esp_err_t __real_esp_wifi_scan_get_ap_num(uint16_t *);
esp_err_t __wrap_esp_wifi_scan_get_ap_num(uint16_t *number) {
    /* The app retrieving the AP count means the (async) scan has completed. */
    s_wifi_scanning = false;
    return __real_esp_wifi_scan_get_ap_num(number);
}

#endif /* FGP_HAVE_WIFI */

/* ── MQTT ────────────────────────────────────────────────────────────────── */
#if FGP_HAVE_MQTT

/*
 * Extract broker hostname and port from esp_mqtt_client_config_t.
 * Handles both URI ("mqtt://host:port") and separate hostname+port styles.
 * IDF v5.x uses nested broker.address struct.
 */
static void extract_mqtt_broker(const esp_mqtt_client_config_t *cfg,
                                 char *host_out, uint16_t *port_out) {
    host_out[0] = '\0'; *port_out = 0;
    if (!cfg) return;

    const char *h = cfg->broker.address.hostname;

    /* Fall back to URI parsing when hostname isn't set separately */
    if (!h || !h[0]) {
        const char *uri = cfg->broker.address.uri;
        if (uri) {
            const char *p = strstr(uri, "://");
            h = p ? (p + 3) : uri;
            /* Default port by scheme */
            if (!cfg->broker.address.port) {
                *port_out = (strncmp(uri, "mqtts://", 8) == 0) ? 8883u : 1883u;
            }
        }
    }

    if (h && h[0]) {
        int j = 0;
        while (j < (int)(FGP_MQTT_HOST_MAX - 1) &&
               h[j] && h[j] != ':' && h[j] != '/' && h[j] != '?') {
            host_out[j] = h[j]; j++;
        }
        host_out[j] = '\0';
    }

    if (cfg->broker.address.port && !*port_out)
        *port_out = (uint16_t)cfg->broker.address.port;
}

/* ── Internal MQTT event listener — auto-captures connect/sub/pub/data ───── */

static void fg_mqtt_copy_event_topic(char *dst, esp_mqtt_event_handle_t ev) {
    if (!ev || !ev->topic || ev->topic_len <= 0) {
        dst[0] = '\0';
        return;
    }
    int n = ev->topic_len < (int)(FGP_MQTT_TOPIC_MAX - 1)
                ? ev->topic_len : (int)(FGP_MQTT_TOPIC_MAX - 1);
    memcpy(dst, ev->topic, (size_t)n);
    dst[n] = '\0';
}

static void fg_mqtt_event_cb(void *arg, esp_event_base_t base,
                              int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;
    if (!ev) return;

    fgp_entry_t e;
    memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_MQTT;
    memset(e.gpio, FG_PIN_NONE, 4);
    mqtt_client_info(ev->client, e.mqtt.host, &e.mqtt.port);

    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        e.evt = FGP_PUB;
        e.mqtt.qos = 0;
        e.mqtt.blen = 0;
        ring_push(&e);
        return;

    case MQTT_EVENT_SUBSCRIBED:
        e.evt = FGP_SUB;
        e.mqtt.qos = 0;
        e.mqtt.blen = 0;
        fg_mqtt_copy_event_topic(e.mqtt.topic, ev);
        ring_push(&e);
        return;

    case MQTT_EVENT_PUBLISHED:
        e.evt = FGP_PUB;
        e.mqtt.qos = 0;
        e.mqtt.blen = 0;
        ring_push(&e);
        return;

    case MQTT_EVENT_DATA:
        if (!ev->data || ev->data_len <= 0) return;
        if (ev->current_data_offset > 0) return;
        e.evt = FGP_RX;
        e.mqtt.qos  = (uint8_t)ev->qos;
        {
            int total = ev->total_data_len > 0 ? ev->total_data_len : ev->data_len;
            e.mqtt.blen = (uint16_t)(total > 0xFFFF ? 0xFFFF : (unsigned)total);
        }
        fg_mqtt_copy_event_topic(e.mqtt.topic, ev);
        capture(e.data, &e.data_len, &e.data_trunc,
                (const uint8_t *)ev->data, (size_t)ev->data_len);
        ring_push(&e);
        return;

    default:
        return;
    }
}

/* ── esp_mqtt_client_init — capture broker host/port on client creation ── */

esp_mqtt_client_handle_t __real_esp_mqtt_client_init(const esp_mqtt_client_config_t *);
esp_mqtt_client_handle_t __wrap_esp_mqtt_client_init(const esp_mqtt_client_config_t *cfg) {
    esp_mqtt_client_handle_t h = __real_esp_mqtt_client_init(cfg);
    if (h && cfg) {
        char host[FGP_MQTT_HOST_MAX]; uint16_t port;
        extract_mqtt_broker(cfg, host, &port);
        for (int i = 0; i < FGP_MAX_MQTT_CLIENTS; i++) {
            if (!s_mqtt_clients[i].h) {
                s_mqtt_clients[i].h    = h;
                s_mqtt_clients[i].port = port;
                strncpy(s_mqtt_clients[i].host, host, FGP_MQTT_HOST_MAX - 1);
                s_mqtt_clients[i].host[FGP_MQTT_HOST_MAX - 1] = '\0';
                break;
            }
        }
    }
    if (h) {
        /* Register our internal listener so MQTT_EVENT_DATA is captured
         * automatically — no user-code changes required.  The callback is
         * fast: one spinlock + memcpy per received message. */
        esp_mqtt_client_register_event(h, ESP_EVENT_ANY_ID,
                                        fg_mqtt_event_cb, NULL);
    }
    return h;
}

/* ── esp_mqtt_client_publish — capture publish with topic + broker info ── */

int __real_esp_mqtt_client_publish(esp_mqtt_client_handle_t, const char *,
                                    const char *, int, int, int);
int __wrap_esp_mqtt_client_publish(esp_mqtt_client_handle_t client,
                                    const char *topic, const char *data,
                                    int len, int qos, int retain) {
    int r = __real_esp_mqtt_client_publish(client, topic, data, len, qos, retain);
    if (r >= 0) {
        int actual_len = (len == 0 && data) ? (int)strlen(data) : len;
        fgp_entry_t e; memset(&e, 0, sizeof(e));
        e.ts_us = (uint64_t)esp_timer_get_time();
        e.proto = FGP_MQTT; e.evt = FGP_PUB;
        memset(e.gpio, FG_PIN_NONE, 4);
        e.mqtt.qos  = (uint8_t)qos;
        e.mqtt.blen = (uint16_t)(actual_len > 0xFFFF ? 0xFFFF : actual_len);
        copy_topic(e.mqtt.topic, topic);
        mqtt_client_info(client, e.mqtt.host, &e.mqtt.port);
        capture(e.data, &e.data_len, &e.data_trunc, data, (size_t)actual_len);
        ring_push(&e);
    }
    return r;
}

/* ── esp_mqtt_client_enqueue — non-blocking publish (IDF v5.x preferred) ── */

int __real_esp_mqtt_client_enqueue(esp_mqtt_client_handle_t, const char *,
                                    const char *, int, int, int, bool);
int __wrap_esp_mqtt_client_enqueue(esp_mqtt_client_handle_t client,
                                    const char *topic, const char *data,
                                    int len, int qos, int retain, bool store) {
    int r = __real_esp_mqtt_client_enqueue(client, topic, data, len, qos, retain, store);
    if (r >= 0) {
        int actual_len = (len == 0 && data) ? (int)strlen(data) : len;
        fgp_entry_t e; memset(&e, 0, sizeof(e));
        e.ts_us = (uint64_t)esp_timer_get_time();
        e.proto = FGP_MQTT; e.evt = FGP_PUB;
        memset(e.gpio, FG_PIN_NONE, 4);
        e.mqtt.qos  = (uint8_t)qos;
        e.mqtt.blen = (uint16_t)(actual_len > 0xFFFF ? 0xFFFF : actual_len);
        copy_topic(e.mqtt.topic, topic);
        mqtt_client_info(client, e.mqtt.host, &e.mqtt.port);
        capture(e.data, &e.data_len, &e.data_trunc, data, (size_t)actual_len);
        ring_push(&e);
    }
    return r;
}

/* ── esp_mqtt_client_subscribe — capture subscribe topic (legacy API) ───── */

int __real_esp_mqtt_client_subscribe(esp_mqtt_client_handle_t, const char *, int);
int __wrap_esp_mqtt_client_subscribe(esp_mqtt_client_handle_t client,
                                      const char *topic, int qos) {
    int r = __real_esp_mqtt_client_subscribe(client, topic, qos);
    if (r >= 0) {
        fgp_entry_t e; memset(&e, 0, sizeof(e));
        e.ts_us = (uint64_t)esp_timer_get_time();
        e.proto = FGP_MQTT; e.evt = FGP_SUB;
        memset(e.gpio, FG_PIN_NONE, 4);
        e.mqtt.qos  = (uint8_t)qos;
        e.mqtt.blen = 0;
        copy_topic(e.mqtt.topic, topic);
        mqtt_client_info(client, e.mqtt.host, &e.mqtt.port);
        ring_push(&e);
    }
    return r;
}

/* ── esp_mqtt_client_subscribe_single — capture subscribe topic ─────────── */

int __real_esp_mqtt_client_subscribe_single(esp_mqtt_client_handle_t,
                                             const char *, int);
int __wrap_esp_mqtt_client_subscribe_single(esp_mqtt_client_handle_t client,
                                             const char *topic, int qos) {
    int r = __real_esp_mqtt_client_subscribe_single(client, topic, qos);
    if (r >= 0) {
        fgp_entry_t e; memset(&e, 0, sizeof(e));
        e.ts_us = (uint64_t)esp_timer_get_time();
        e.proto = FGP_MQTT; e.evt = FGP_SUB;
        memset(e.gpio, FG_PIN_NONE, 4);
        e.mqtt.qos  = (uint8_t)qos;
        e.mqtt.blen = 0;
        copy_topic(e.mqtt.topic, topic);
        mqtt_client_info(client, e.mqtt.host, &e.mqtt.port);
        ring_push(&e);
    }
    return r;
}

#endif /* FGP_HAVE_MQTT */

/* ══════════════════════════════════════════════════════════════════════════
 * SECTION 11 — MANUAL API
 *
 * Use these when auto-interception is unavailable (supplemental UART
 * logging on the monitor port, MQTT received messages, etc.) or when you
 * want to add GPIO information to protocols that the legacy I2C API doesn't
 * expose.
 * ══════════════════════════════════════════════════════════════════════════ */

/* ── I2C ─────────────────────────────────────────────────────────────────── */

void fg_proto_i2c(uint8_t addr, bool read, uint8_t len, bool ack) {
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_I2C; e.evt = read ? FGP_RX : FGP_TX;
    memset(e.gpio, FG_PIN_NONE, 4);
    e.i2c.addr = addr; e.i2c.len = len; e.i2c.ack = ack ? 1u : 0u;
    ring_push(&e);
}

void fg_proto_i2c_ex(uint8_t sda, uint8_t scl,
                     uint8_t addr, bool read,
                     const uint8_t *data, uint8_t len, bool ack) {
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_I2C; e.evt = read ? FGP_RX : FGP_TX;
    e.gpio[0] = sda; e.gpio[1] = scl; e.gpio[2] = e.gpio[3] = FG_PIN_NONE;
    e.i2c.addr = addr; e.i2c.len = len; e.i2c.ack = ack ? 1u : 0u;
    capture(e.data, &e.data_len, &e.data_trunc, data, len);
    ring_push(&e);
}

/* ── UART ────────────────────────────────────────────────────────────────── */

void fg_proto_uart(uint8_t port, bool is_rx, uint8_t len) {
    if (port >= FGP_MAX_UART) return;
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_UART; e.evt = is_rx ? FGP_RX : FGP_TX;
    e.gpio[0] = s_uart[port].tx;  e.gpio[1] = s_uart[port].rx;
    e.gpio[2] = s_uart[port].rts; e.gpio[3] = s_uart[port].cts;
    e.uart.port = port; e.uart.len = len;
    ring_push(&e);
}

void fg_proto_uart_ex(uint8_t tx_pin, uint8_t rx_pin,
                      uint8_t port, bool is_rx,
                      const uint8_t *data, uint8_t len) {
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_UART; e.evt = is_rx ? FGP_RX : FGP_TX;
    e.gpio[0] = tx_pin; e.gpio[1] = rx_pin;
    e.gpio[2] = e.gpio[3] = FG_PIN_NONE;
    e.uart.port = port; e.uart.len = len;
    capture(e.data, &e.data_len, &e.data_trunc, data, len);
    ring_push(&e);
}

/* ── WiFi ────────────────────────────────────────────────────────────────── */

void fg_proto_wifi_rssi(int8_t rssi) {
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_WIFI; e.evt = FGP_RSSI;
    memset(e.gpio, FG_PIN_NONE, 4);
    e.wifi.rssi = rssi;
    ring_push(&e);
}

/* ── MQTT ────────────────────────────────────────────────────────────────── */

void fg_proto_mqtt_pub(uint8_t qos, uint16_t payload_len) {
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_MQTT; e.evt = FGP_PUB;
    memset(e.gpio, FG_PIN_NONE, 4);
    e.mqtt.qos = qos; e.mqtt.blen = payload_len;
    ring_push(&e);
}

void fg_proto_mqtt_sub(uint8_t qos) {
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_MQTT; e.evt = FGP_SUB;
    memset(e.gpio, FG_PIN_NONE, 4);
    e.mqtt.qos = qos; e.mqtt.blen = 0;
    ring_push(&e);
}

/*
 * fg_proto_mqtt_recv — call this from your MQTT_EVENT_DATA handler to capture
 * incoming messages.  Linker-wrap interception of callbacks is not possible,
 * so this is the only way to capture received MQTT payloads.
 *
 * Example usage:
 *
 *   static void mqtt_event_handler(void *arg, esp_event_base_t base,
 *                                  int32_t id, void *event_data) {
 *       esp_mqtt_event_handle_t ev = event_data;
 *       if (id == MQTT_EVENT_DATA) {
 *           fg_proto_mqtt_recv(ev->topic, ev->topic_len,
 *                              (const uint8_t *)ev->data, ev->data_len, 0);
 *       }
 *   }
 *
 * @param topic      topic string (NOT necessarily null-terminated from IDF)
 * @param topic_len  length of topic string
 * @param payload    received payload bytes (NULL = length-only log)
 * @param payload_len number of payload bytes
 * @param qos        QoS level (0, 1, or 2)
 */
void fg_proto_mqtt_recv(const char *topic, int topic_len,
                         const uint8_t *payload, int payload_len, uint8_t qos) {
    fgp_entry_t e; memset(&e, 0, sizeof(e));
    e.ts_us = (uint64_t)esp_timer_get_time();
    e.proto = FGP_MQTT; e.evt = FGP_RX;
    memset(e.gpio, FG_PIN_NONE, 4);
    e.mqtt.qos  = qos;
    e.mqtt.blen = (uint16_t)(payload_len > 0xFFFF ? 0xFFFF : payload_len);

    /* Copy topic — IDF gives us (char*, len), not a null-terminated string */
    if (topic && topic_len > 0) {
        int n = topic_len < (int)(FGP_MQTT_TOPIC_MAX - 1)
                    ? topic_len : (int)(FGP_MQTT_TOPIC_MAX - 1);
        memcpy(e.mqtt.topic, topic, (size_t)n);
        e.mqtt.topic[n] = '\0';
    }

    capture(e.data, &e.data_len, &e.data_trunc, payload, (size_t)payload_len);
    ring_push(&e);
}

