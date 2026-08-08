/**
 * firmgen_proto_monitor.h — Firmgen unified protocol bus monitor
 *
 * Captures I2C, UART, WiFi, and MQTT traffic and emits it as
 * newline-delimited JSON on the UART stdout channel for the backend parser.
 *
 * Wire format — one JSON line per 500 ms flush (schema v3):
 *
 * Outer envelope key: "p":"proto_monitor"  (identifies the source)
 * Inner event key:    "t":<proto>          ("uart","i2c","wifi","mqtt")
 *
 *   {"p":"proto_monitor","v":3,"ts_ms":<ms>,"evts":[
 *     {"t":"i2c", "e":"tx","addr":60,"len":3,"ack":1,
 *      "gpio":{"sda":21,"scl":22},"data":"0A1B2C","us":200},
 *
 *     {"t":"uart","e":"rx","port":1,"len":5,
 *      "gpio":{"tx":17,"rx":16},"data":"48656C6C6F","us":400},
 *
 *     {"t":"wifi","e":"rssi","rssi":-67,"us":800},
 *
 *     {"t":"mqtt","e":"pub","qos":1,"blen":12,"data":"48656C6C6F","us":900}
 *   ]}
 *
 * AUTO-INTERCEPTED (no user calls needed — GNU --wrap at link time):
 *   I2C  → i2c_master_transmit / receive / transmit_receive
 *   UART → uart_write_bytes / uart_read_bytes  (UART0 excluded — monitor port)
 *   MQTT → esp_mqtt_client_publish
 *   WiFi → RSSI sampled automatically each flush interval
 *
 * MANUAL — call from your event handlers / driver wrappers:
 *   I2C  → fg_proto_i2c_ex()  or  fg_proto_i2c()
 *   UART → fg_proto_uart_ex() or  fg_proto_uart()
 *   MQTT → fg_proto_mqtt_pub() / fg_proto_mqtt_sub()
 *
 * RTOS impact: none.  The emitter task runs at priority 1 (lowest) and uses
 * a portMUX spinlock — no mutexes, no scheduler suspension, no ISR wakeup.
 *
 * Compatible with ESP32, S2, S3, C3, C6, H2, P4 on IDF v5.x.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Sentinel value — GPIO pin not connected or not known. */
#define FG_PIN_NONE  0xFFu

/**
 * Start the background emitter task.
 * Idempotent — safe to call more than once.
 * Must be called after the FreeRTOS scheduler is running (e.g. from app_main).
 */
void firmgen_proto_monitor_start(void);

/* ═══════════════════════════════════════════════════════════════════════════
 * EXTENDED API — GPIO pins + data payload
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Log an I2C transaction with GPIO info and payload bytes.
 * @param sda   SDA GPIO number (FG_PIN_NONE if unknown)
 * @param scl   SCL GPIO number (FG_PIN_NONE if unknown)
 * @param addr  7-bit device address
 * @param read  true = I2C read (RX), false = write (TX)
 * @param data  byte buffer (NULL = count-only log)
 * @param len   byte count
 * @param ack   true = ACK received, false = NAK
 */
void fg_proto_i2c_ex(uint8_t sda, uint8_t scl,
                     uint8_t addr, bool read,
                     const uint8_t *data, uint8_t len, bool ack);

/**
 * Log a UART burst with GPIO info and payload.
 * @param tx_pin  TX GPIO (FG_PIN_NONE if unknown)
 * @param rx_pin  RX GPIO (FG_PIN_NONE if unknown)
 * @param port    UART port number (0–2)
 * @param is_rx   true = received, false = transmitted
 * @param data    byte buffer (NULL = count-only log)
 * @param len     byte count
 */
void fg_proto_uart_ex(uint8_t tx_pin, uint8_t rx_pin,
                      uint8_t port, bool is_rx,
                      const uint8_t *data, uint8_t len);

/* ═══════════════════════════════════════════════════════════════════════════
 * SHORT-FORM API — no GPIO info, no data capture
 * ═══════════════════════════════════════════════════════════════════════════ */

void fg_proto_i2c     (uint8_t addr, bool read, uint8_t len, bool ack);
void fg_proto_uart    (uint8_t port, bool is_rx, uint8_t len);
void fg_proto_wifi_rssi(int8_t rssi);
void fg_proto_mqtt_pub(uint8_t qos, uint16_t payload_len);
void fg_proto_mqtt_sub(uint8_t qos);

/**
 * Log an incoming MQTT message (received payload).
 *
 * MQTT receive events arrive via the MQTT event loop callback, which cannot
 * be intercepted with linker --wrap.  Call this from your MQTT_EVENT_DATA
 * handler to capture received messages:
 *
 *   case MQTT_EVENT_DATA:
 *       fg_proto_mqtt_recv(event->topic,    event->topic_len,
 *                          (uint8_t*)event->data, event->data_len, 0);
 *       break;
 *
 * @param topic       topic string (IDF passes non-null-terminated char*)
 * @param topic_len   length of topic string
 * @param payload     payload bytes (NULL = count-only log)
 * @param payload_len payload byte count
 * @param qos         QoS level (0, 1, or 2)
 */
void fg_proto_mqtt_recv(const char *topic, int topic_len,
                         const uint8_t *payload, int payload_len, uint8_t qos);

#ifdef __cplusplus
}
#endif
