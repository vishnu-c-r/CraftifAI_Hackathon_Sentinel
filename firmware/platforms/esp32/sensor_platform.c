#include "sensor_platform.h"
#include "vibration_source.h"
#include "operating_state.h"
#include "baseline_learner.h"
#include "led_platform.h"
#include "anomaly_detector.h"
#include "telemetry.h"
#include "app_config.h"
#include "logger.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sensors";
static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_mpu;
static i2c_master_dev_handle_t s_pressure;
static bool s_mpu_present;
static bool s_pressure_present;
static bool s_address_present[0x80];
static float s_gravity_x;
static float s_gravity_y;
static float s_gravity_z;
static float s_filtered_magnitude;
static bool s_filter_initialized;
static TaskHandle_t s_sensor_task_handle;
static esp_timer_handle_t s_sensor_timer;

static void sensor_timer_callback(void *arg)
{
    (void)arg;
    if (s_sensor_task_handle != NULL) {
        xTaskNotifyGive(s_sensor_task_handle);
    }
}
#if APP_SENSOR_SIMULATION
static uint64_t s_simulation_start_ms;
#endif
static float s_temperature_c;
static bool s_temperature_valid;
static int16_t s_ac1, s_ac2, s_ac3, s_b1, s_b2, s_mb, s_mc, s_md;
static uint16_t s_ac4, s_ac5, s_ac6;

static esp_err_t add_device(uint8_t address, i2c_master_dev_handle_t *device)
{
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = APP_I2C_FREQUENCY_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &config, device);
}

static esp_err_t read_register(i2c_master_dev_handle_t device, uint8_t reg,
                               uint8_t *data, size_t length)
{
    return i2c_master_transmit_receive(device, &reg, 1, data, length,
                                        APP_I2C_TIMEOUT_MS);
}

static esp_err_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(s_mpu, data, sizeof(data), APP_I2C_TIMEOUT_MS);
}

static void print_scan(void)
{
    ESP_LOGI(TAG, "I2C scan on SDA=%d SCL=%d", APP_I2C_SDA_GPIO, APP_I2C_SCL_GPIO);
    for (uint8_t address = 1; address < 0x7f; ++address) {
        if (i2c_master_probe(s_bus, address, APP_I2C_TIMEOUT_MS) == ESP_OK) {
            s_address_present[address] = true;
            ESP_LOGI(TAG, "I2C device found at 0x%02X", address);
        }
    }
}

static void detect_motion(void)
{
    const uint8_t addresses[] = {0x68, 0x69};
    for (size_t i = 0; i < 2; ++i) {
        if (!s_address_present[addresses[i]]) continue;
        i2c_master_dev_handle_t device = NULL;
        if (add_device(addresses[i], &device) != ESP_OK) continue;
        uint8_t id = 0;
        if (read_register(device, 0x75, &id, 1) == ESP_OK) {
            ESP_LOGI(TAG, "motion 0x%02X WHO_AM_I=0x%02X (%s)", addresses[i], id,
                     id == 0x68 ? "MPU-6050" : "unknown motion chip");
            if (id == 0x68 && !s_mpu_present) {
                s_mpu = device;
                s_mpu_present = true;
                if (write_register(0x6B, 0x00) != ESP_OK ||
                    write_register(0x1C, 0x00) != ESP_OK ||
                    write_register(0x1A, APP_MPU_DLPF_CFG) != ESP_OK ||
                    write_register(0x19, APP_MPU_SAMPLE_RATE_DIV) != ESP_OK) {
                    ESP_LOGE(TAG, "MPU-6050 configuration failed");
                }
            } else {
                i2c_master_bus_rm_device(device);
            }
        } else {
            i2c_master_bus_rm_device(device);
        }
    }
    ESP_LOGI(TAG, "MPU-6050 detected: %s", s_mpu_present ? "yes" : "no");
}

static void detect_pressure(void)
{
    const uint8_t addresses[] = {0x76, 0x77};
    for (size_t i = 0; i < 2; ++i) {
        if (!s_address_present[addresses[i]]) continue;
        i2c_master_dev_handle_t device = NULL;
        if (add_device(addresses[i], &device) != ESP_OK) continue;
        uint8_t id = 0;
        if (read_register(device, 0xD0, &id, 1) == ESP_OK) {
            const char *name = id == 0x55 ? "BMP180" : id == 0x58 ? "BMP280" :
                               id == 0x60 ? "BME280" : "unknown pressure chip";
            ESP_LOGI(TAG, "pressure 0x%02X chip ID=0x%02X (%s)", addresses[i], id, name);
            if (id == 0x55 && !s_pressure_present) {
                s_pressure = device;
                s_pressure_present = true;
            } else {
                i2c_master_bus_rm_device(device);
            }
        } else {
            i2c_master_bus_rm_device(device);
        }
    }
    ESP_LOGI(TAG, "BMP180 detected: %s", s_pressure_present ? "yes" : "no");
}

static void read_bmp180_calibration(void)
{
    uint8_t raw[22];
    if (!s_pressure_present || read_register(s_pressure, 0xAA, raw, sizeof(raw)) != ESP_OK) return;
    s_ac1 = (int16_t)((raw[0] << 8) | raw[1]); s_ac2 = (int16_t)((raw[2] << 8) | raw[3]);
    s_ac3 = (int16_t)((raw[4] << 8) | raw[5]); s_ac4 = (uint16_t)((raw[6] << 8) | raw[7]);
    s_ac5 = (uint16_t)((raw[8] << 8) | raw[9]); s_ac6 = (uint16_t)((raw[10] << 8) | raw[11]);
    s_b1 = (int16_t)((raw[12] << 8) | raw[13]); s_b2 = (int16_t)((raw[14] << 8) | raw[15]);
    s_mb = (int16_t)((raw[16] << 8) | raw[17]); s_mc = (int16_t)((raw[18] << 8) | raw[19]);
    s_md = (int16_t)((raw[20] << 8) | raw[21]);
}

static void update_temperature(void)
{
    if (!s_pressure_present) return;
    uint8_t command[] = {0xF4, 0x2E};
    if (i2c_master_transmit(s_pressure, command, sizeof(command), APP_I2C_TIMEOUT_MS) != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(5));
    uint8_t raw[2];
    if (read_register(s_pressure, 0xF6, raw, sizeof(raw)) != ESP_OK) return;
    const int32_t ut = (raw[0] << 8) | raw[1];
    const int32_t x1 = ((ut - s_ac6) * s_ac5) >> 15;
    const int32_t x2 = (s_mc << 11) / (x1 + s_md);
    const int32_t b5 = x1 + x2;
    s_temperature_c = ((float)(b5 + 8) / 16.0f) / 10.0f;
    s_temperature_valid = isfinite(s_temperature_c);
}

esp_err_t sensor_platform_init(void)
{
    const i2c_master_bus_config_t config = {
        .i2c_port = I2C_NUM_0, .sda_io_num = APP_I2C_SDA_GPIO, .scl_io_num = APP_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT, .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&config, &s_bus);
    if (err != ESP_OK) return err;
#if !APP_SENSOR_SIMULATION
    print_scan(); detect_motion(); detect_pressure(); read_bmp180_calibration();
#else
    ESP_LOGI(TAG, "simulation mode enabled; physical sensor reads bypassed");
#endif
    return ESP_OK;
}

esp_err_t vibration_source_init(void) { return sensor_platform_init(); }
bool vibration_source_is_present(void) { return s_mpu_present; }

esp_err_t vibration_source_read(vibration_sample_t *sample)
{
    if (!sample) return ESP_ERR_INVALID_ARG;
#if APP_SENSOR_SIMULATION
    const uint64_t now_ms = (uint64_t)(esp_timer_get_time() / 1000ULL);
    if (s_simulation_start_ms == 0) s_simulation_start_ms = now_ms;
    const uint32_t cycle_ms = APP_SENSOR_SIM_OFF_MS + APP_SENSOR_SIM_STARTING_MS +
                               APP_SENSOR_SIM_RUNNING_MS + APP_SENSOR_SIM_STOPPING_MS;
    const uint32_t phase_ms = (uint32_t)((now_ms - s_simulation_start_ms) % cycle_ms);
    const uint32_t start = APP_SENSOR_SIM_OFF_MS;
    const uint32_t run = start + APP_SENSOR_SIM_STARTING_MS;
    const uint32_t stop = run + APP_SENSOR_SIM_RUNNING_MS;
    float value = 0.01f;
    if (phase_ms >= start && phase_ms < run) {
        value = 0.01f + 0.79f * (float)(phase_ms - start) / (float)APP_SENSOR_SIM_STARTING_MS;
    } else if (phase_ms >= run && phase_ms < stop) {
        value = 0.05f;
    } else if (phase_ms >= stop) {
        value = 0.40f * (1.0f - (float)(phase_ms - stop) / (float)APP_SENSOR_SIM_STOPPING_MS);
    }
    sample->x_g = value; sample->y_g = 0.0f; sample->z_g = 0.0f; sample->magnitude_g = value;
    return ESP_OK;
#else
    if (!s_mpu_present) return ESP_ERR_INVALID_STATE;
    uint8_t raw[6];
    esp_err_t err = read_register(s_mpu, 0x3B, raw, sizeof(raw));
    if (err != ESP_OK) return err;
    const int16_t x = (int16_t)((raw[0] << 8) | raw[1]);
    const int16_t y = (int16_t)((raw[2] << 8) | raw[3]);
    const int16_t z = (int16_t)((raw[4] << 8) | raw[5]);
    const float ax = (float)x * APP_MPU_ACCEL_SCALE_G / 32768.0f;
    const float ay = (float)y * APP_MPU_ACCEL_SCALE_G / 32768.0f;
    const float az = (float)z * APP_MPU_ACCEL_SCALE_G / 32768.0f;
    const float gravity_alpha = 0.01f;
    s_gravity_x += gravity_alpha * (ax - s_gravity_x);
    s_gravity_y += gravity_alpha * (ay - s_gravity_y);
    s_gravity_z += gravity_alpha * (az - s_gravity_z);
    sample->x_g = ax - s_gravity_x; sample->y_g = ay - s_gravity_y; sample->z_g = az - s_gravity_z;
    sample->magnitude_g = sqrtf(sample->x_g * sample->x_g + sample->y_g * sample->y_g + sample->z_g * sample->z_g);
    return ESP_OK;
#endif
}

static void sensor_task(void *arg)
{
    (void)arg;
    float sum_sq = 0.0f, peak = 0.0f;
    uint32_t count = 0, errors = 0, last_temp_ms = 0;
    operating_state_init();
    anomaly_detector_init();
#if APP_SENSOR_LEARNING_ENABLE
    baseline_learner_init(APP_SENSOR_BASELINE_REQUIRED_CYCLES);
#endif
    for (;;) {
        const uint64_t now_us = esp_timer_get_time();
        vibration_sample_t sample;
        if (vibration_source_read(&sample) == ESP_OK && isfinite(sample.magnitude_g)) {
            if (!s_filter_initialized) { s_filtered_magnitude = sample.magnitude_g; s_filter_initialized = true; }
            else s_filtered_magnitude = APP_SENSOR_EMA_ALPHA * sample.magnitude_g + (1.0f - APP_SENSOR_EMA_ALPHA) * s_filtered_magnitude;
            sum_sq += s_filtered_magnitude * s_filtered_magnitude;
            if (s_filtered_magnitude > peak) peak = s_filtered_magnitude;
            ++count;
        } else ++errors;
        const uint32_t now_ms = (uint32_t)(now_us / 1000ULL);
        if (now_ms - last_temp_ms >= APP_SENSOR_TEMP_PERIOD_MS) { update_temperature(); last_temp_ms = now_ms; }
        if (now_ms % APP_SENSOR_WINDOW_MS < APP_SENSOR_RAW_PERIOD_MS && count > 0) {
            const float rms = sqrtf(sum_sq / (float)count);
            const sentinel_state_t state = operating_state_update(rms, now_ms);
            const anomaly_level_t anomaly = state == SENTINEL_STATE_RUNNING ?
                anomaly_detector_update(rms, now_ms, baseline_learner_is_valid()) : ANOMALY_NONE;
            if (anomaly == ANOMALY_CRITICAL || state == SENTINEL_STATE_FAULT) led_platform_set_status(LED_STATUS_CRITICAL);
            else if (anomaly == ANOMALY_WARNING) led_platform_set_status(LED_STATUS_WARNING);
            else if (state == SENTINEL_STATE_FAULT) led_platform_set_status(LED_STATUS_CRITICAL);
            else if (baseline_learner_is_learning()) led_platform_set_status(LED_STATUS_BASELINE);
            else if (state == SENTINEL_STATE_RUNNING) led_platform_set_status(LED_STATUS_RUNNING);
            else if (state == SENTINEL_STATE_OFF) led_platform_set_status(LED_STATUS_OFF);
            else led_platform_set_status(LED_STATUS_BOOT);
#if APP_SENSOR_LEARNING_ENABLE
            baseline_learner_observe(state, rms);
#endif
            telemetry_snapshot_t snapshot = {
                .uptime_ms = now_ms,
                .vibration_rms_g = rms,
                .vibration_peak_g = peak,
                .temperature_c = s_temperature_c,
                .temperature_valid = s_temperature_valid,
                .samples = count,
                .errors = errors,
                .phase = state,
                .anomaly = anomaly
            };
            telemetry_publish(&snapshot);
            ESP_LOGI(TAG, "uptime=%lu phase=%s rms=%.3f g peak=%.3f g temp=%s%.2f C samples=%lu errors=%lu anomaly=%s",
                     (unsigned long)now_ms, operating_state_name(state), rms, peak,
                     s_temperature_valid ? "" : "invalid ", s_temperature_c,
                     (unsigned long)count, (unsigned long)errors,
                     anomaly_detector_name(anomaly));
            sum_sq = 0.0f; peak = 0.0f; count = 0; errors = 0;
        }
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
}

void sensor_platform_start(void)
{
#if APP_SENSOR_SIMULATION
    if (xTaskCreate(sensor_task, "sensor_200hz", 4096, NULL, 5, &s_sensor_task_handle) == pdPASS) {
        const esp_timer_create_args_t timer_args = {
            .callback = sensor_timer_callback,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "sensor_200hz",
        };
        if (esp_timer_create(&timer_args, &s_sensor_timer) == ESP_OK) {
            esp_timer_start_periodic(s_sensor_timer, APP_SENSOR_RAW_PERIOD_MS * 1000ULL);
        }
    }
#elif !APP_SENSOR_SIMULATION
    if (s_mpu_present) {
        if (xTaskCreate(sensor_task, "sensor_200hz", 4096, NULL, 5, &s_sensor_task_handle) == pdPASS) {
            const esp_timer_create_args_t timer_args = {
                .callback = sensor_timer_callback,
                .arg = NULL,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "sensor_200hz",
            };
            if (esp_timer_create(&timer_args, &s_sensor_timer) == ESP_OK) {
                esp_timer_start_periodic(s_sensor_timer, APP_SENSOR_RAW_PERIOD_MS * 1000ULL);
            }
        }
    } else {
        ESP_LOGW(TAG, "no supported motion sensor; sensor task not started");
    }
#endif
}
