# ══════════════════════════════════════════════════════════════════════════════
# MQTT compile-time support
#
# idf_component_optional_requires(PRIVATE mqtt) was supposed to make
# mqtt_client.h visible, but in practice it does NOT reliably add the
# component's include path when called after idf_component_register.
#
# Fix: build the PRIV_REQUIRES list before the register call so that when
# CONFIG_MQTT_PROTOCOL_311/5 is set, "mqtt" is a real PRIV_REQUIRES entry
# and its include directory is guaranteed to be on the compiler search path.
#
# CRITICAL: FGP_HAVE_MQTT=1 and --wrap=esp_mqtt_client_* MUST be in sync:
#   FGP_HAVE_MQTT=1  → __wrap_* functions compiled into the component
#   --wrap flags     → application calls redirected to those functions
# If either side is missing the build fails to compile or link.
# ══════════════════════════════════════════════════════════════════════════════

# Only the always-needed core components are hard-required here. Peripheral drivers
# (I2C/UART/Wi-Fi/MQTT/temp) are added conditionally AFTER register via
# idf_component_optional_requires — see the hooks below. That keeps simple projects
# lean while still making each driver's header visible to the protocol-monitor shims
# when the project actually uses that peripheral.
set(_fgp_req esp_timer heap)
set(_fgp_priv_req "")

idf_component_register(
    SRCS        "firmgen_telemetry.c"
                "firmgen_proto_monitor.c"
    INCLUDE_DIRS "include"
    REQUIRES    ${_fgp_req}
    PRIV_REQUIRES ${_fgp_priv_req}
)

# ── Protocol-monitor driver auto-detection ──────────────────────────────────────
# The I2C / UART / Wi-Fi capture shims in firmgen_proto_monitor.c are gated by
# __has_include("driver/i2c_master.h") etc. __has_include only sees a header if THIS
# component requires that driver — it does NOT inherit the app's includes. So without
# these, every shim compiles out and the I2C/UART/Wi-Fi tabs stay blank even when the
# app uses those buses (this was the bug). idf_component_optional_requires adds each
# dependency ONLY when the project already pulls that component into the build:
#   • app uses the bus  → component in build → header visible → shim ON  → live data
#   • app doesn't        → no-op            → shim compiled OUT → no dep, no bloat
# Granular esp_driver_* (ESP-IDF v5.2+) only — intentionally NOT the "driver" umbrella,
# which pulls the whole esp_driver_* suite + esp_lcd and balloons the build (see
# standard_scaffold::migrate_main_requires, which strips it from project main/).
idf_component_optional_requires(PRIVATE esp_wifi)         # esp_wifi.h          → Wi-Fi RSSI
idf_component_optional_requires(PRIVATE esp_driver_i2c)   # driver/i2c_master.h → I2C
idf_component_optional_requires(PRIVATE esp_driver_uart)  # driver/uart.h       → UART events

if(CONFIG_MQTT_PROTOCOL_311 OR CONFIG_MQTT_PROTOCOL_5 OR CONFIG_MQTT_TRANSPORT_SSL OR CONFIG_MQTT_TRANSPORT_WEBSOCKET)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE FGP_HAVE_MQTT=1)
    get_filename_component(_fgp_project_dir "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
    if(EXISTS "${_fgp_project_dir}/managed_components/espressif__mqtt/CMakeLists.txt")
        idf_component_optional_requires(PRIVATE espressif__mqtt)
    else()
        idf_component_optional_requires(PRIVATE mqtt)
    endif()
endif()

# Temperature sensor (RTOS panel "CPU Temp" / health.temp_c) is opt-in via Kconfig
# and OFF by default, so simple projects never drag in the tsens driver. When the
# user enables CONFIG_FG_TELEMETRY_TEMP_SENSOR, pull in esp_driver_tsens so that
# driver/temperature_sensor.h resolves. optional_requires skips silently on older
# IDF where the granular component is absent, and the source additionally gates the
# include on __has_include, so it degrades cleanly instead of failing the build.
if(CONFIG_FG_TELEMETRY_TEMP_SENSOR)
    idf_component_optional_requires(PRIVATE esp_driver_tsens)
endif()

# ══════════════════════════════════════════════════════════════════════════════
# GNU linker --wrap flags — single unconditional INTERFACE block
#
# INTERFACE propagates these flags to the final link step via every target that
# depends on firmgen_telemetry (typically the project's main component).
#
# All wraps are unconditional to avoid the "conditional INTERFACE" bug where
# CMake silently drops options from inside if() blocks in some IDF v5.x versions.
#
# Safety rule: --wrap=foo is harmless when no code calls foo.
#   Projects without MQTT never call esp_mqtt_client_publish → no linker error.
#   Projects with MQTT have FGP_HAVE_MQTT=1 (above) → __wrap_* compiled → OK.
#
# NOTE: GPIO wraps were removed with the GPIO State Monitor — their __wrap_gpio_*
# implementations lived in firmgen_gpio_monitor.c, which is not part of this build.
# ══════════════════════════════════════════════════════════════════════════════
target_link_options(${COMPONENT_LIB} INTERFACE
    # ── I2C new master API (IDF v5.x) ──────────────────────────────────────
    -Wl,--wrap=i2c_new_master_bus
    -Wl,--wrap=i2c_master_bus_add_device
    -Wl,--wrap=i2c_master_transmit
    -Wl,--wrap=i2c_master_receive
    -Wl,--wrap=i2c_master_transmit_receive

    # ── UART ────────────────────────────────────────────────────────────────
    -Wl,--wrap=uart_set_pin
    -Wl,--wrap=uart_write_bytes
    -Wl,--wrap=uart_read_bytes

    # ── WiFi (scan-state tracking only — so RSSI sampling never races a scan) ──
    -Wl,--wrap=esp_wifi_scan_start
    -Wl,--wrap=esp_wifi_scan_stop
    -Wl,--wrap=esp_wifi_scan_get_ap_num

    # ── MQTT ────────────────────────────────────────────────────────────────
    -Wl,--wrap=esp_mqtt_client_init
    -Wl,--wrap=esp_mqtt_client_publish
    -Wl,--wrap=esp_mqtt_client_enqueue
    -Wl,--wrap=esp_mqtt_client_subscribe
    -Wl,--wrap=esp_mqtt_client_subscribe_single
)
