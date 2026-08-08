# CraftiAI Sentinel

ESP32-C3-DevKitM-1 condition-monitoring firmware for vibration, temperature, local status indication, and real-machine baseline commissioning.

## 1. System overview

Sentinel monitors an HW-290/GY-91 sensor board over I²C:

- **MPU-6050** — accelerometer and vibration source
- **BMP180** — temperature and pressure sensor; this project currently uses temperature
- **WS2812 RGB LED** — local operating/anomaly indicator
- **ESP32-C3 SoftAP + HTTP server** — device-hosted commissioning webpage

Humidity is not available from the BMP180. Add a separate humidity sensor in a future hardware revision if humidity monitoring is required.

## 2. Board and pinout

Target board: **ESP32-C3-DevKitM-1**

| Function | GPIO | Direction / notes |
|---|---:|---|
| HW-290 SDA | **4** | I²C data; module has onboard pull-ups |
| HW-290 SCL | **5** | I²C clock; module has onboard pull-ups |
| Onboard WS2812 data | **8** | Addressable RGB LED; reserved |
| Restricted GPIOs | **11–17** | Connected to integrated flash; do not use |

### HW-290 wiring

```text
ESP32-C3-DevKitM-1       HW-290 / GY-91
------------------       -------------
3V3                      VCC
GND                      GND
GPIO4                   SDA
GPIO5                   SCL
```

Use 3.3 V logic. Do not connect the sensor board to 5 V logic unless the module’s level compatibility has been independently verified.

## 3. I²C device detection

At startup, Sentinel scans the 7-bit I²C address space and reports every responding address.

Expected addresses for the tested board:

| Device | Address | Identification register | Expected value |
|---|---:|---:|---:|
| MPU-6050 | `0x68` or `0x69` | `WHO_AM_I`, `0x75` | `0x68` |
| BMP180 | `0x76` or `0x77` | Chip ID, `0xD0` | `0x55` |

The firmware does not silently configure unknown chip IDs as known devices.

## 4. Vibration acquisition and filtering

The MPU-6050 is configured for:

- Full-scale accelerometer range: **±2 g**
- Raw acquisition target: **200 Hz**
- Sample interval: **5 ms**
- MPU digital low-pass filter: `APP_MPU_DLPF_CFG=2`, approximately 94 Hz bandwidth
- Sample-rate divider: `APP_MPU_SAMPLE_RATE_DIV=4`
- Telemetry aggregation window: **100 ms**
- Serial/dashboard summary rate: approximately **10 Hz**

Static gravity is estimated independently on X, Y, and Z and removed before calculating dynamic vibration magnitude. The magnitude is smoothed with an exponential moving average:

```text
filtered = alpha * new_value + (1 - alpha) * filtered
```

Default:

```c
#define APP_SENSOR_EMA_ALPHA 0.25f
```

Each summary reports vibration RMS, peak, sample count, I²C/read errors, temperature, operating phase, and anomaly state.

## 5. Operating states

Sentinel classifies machine behavior as:

```text
OFF → STARTING → RUNNING → STOPPING → OFF
```

A `FAULT` state is used for persistent abnormal behavior. Startup and shutdown transients are handled separately so they are not immediately treated as running-condition faults.

Current commissioning defaults:

| Setting | Default |
|---|---:|
| Start threshold | `0.08 g RMS` |
| Stop threshold | `0.05 g RMS` |
| Fault threshold | `0.30 g RMS` |
| Recovery threshold | `0.20 g RMS` |
| Startup settling | `10 s` |
| Stop confirmation | `2 s` |
| Fault persistence | `2 s` |
| Fault clear persistence | `5 s` |

These are provisional values. Real machine thresholds must be established through baseline commissioning.

## 6. Baseline commissioning

Simulation is disabled in the production configuration:

```c
#define APP_SENSOR_SIMULATION 0
```

A valid baseline requires **10 complete real-machine cycles** in this order:

```text
OFF → STARTING → RUNNING → STOPPING → OFF
```

The baseline learner records per-phase vibration statistics and derives running thresholds:

```text
warning  = max(0.08 g, mean + 3 × standard deviation)
critical = max(0.30 g, mean + 5 × standard deviation)
```

A valid baseline is saved to NVS only after completion. An incomplete or invalid trial must not replace an existing valid baseline.

### Commissioning commands

Commands are line-based and terminated with Enter. They are available through the serial console and the device-hosted dashboard command path:

```text
START_BASELINE
STOP_BASELINE
RESET_BASELINE
BASELINE_STATUS
HELP
```

Recommended procedure:

1. Install Sentinel securely on the machine.
2. Power the machine off and allow vibration to settle.
3. Connect to the Sentinel dashboard or serial console.
4. Issue `START_BASELINE`.
5. Operate the machine normally through at least 10 complete cycles.
6. Issue `BASELINE_STATUS` to check progress.
7. Stop only after the required cycles are complete.
8. Confirm the baseline is valid before enabling production alerts.

Do not collect a baseline while the machine has a known fault or abnormal load.

## 7. Device-hosted dashboard

Sentinel creates a local WPA2 Wi-Fi access point:

```text
SSID:      Sentinel-Setup
Password:  Sentinel1234
Address:   http://192.168.4.1/
```

Usage:

1. Connect a phone or computer to `Sentinel-Setup`.
2. Temporarily disable mobile data, VPN, or another active network if the browser chooses the wrong route.
3. Open `http://192.168.4.1/` manually.
4. Hard-refresh the page if an older cached page appears.

The webpage provides two controls:

- **Start baseline**
- **Stop**

It also displays formatted live values for:

- Machine phase
- Vibration RMS
- Vibration peak
- Temperature
- Anomaly status
- Baseline validity and progress
- Connection/update status

The machine-readable endpoint remains available at:

```text
GET http://192.168.4.1/api/status
```

The dashboard is implemented with the native ESP-IDF HTTP server. FastAPI/Python is not run on the ESP32-C3.

## 8. WS2812 LED status

The onboard addressable LED is on GPIO8. Status priority is:

| LED indication | Meaning |
|---|---|
| Off | Machine OFF |
| Blue | STARTING or STOPPING |
| Green | Normal RUNNING |
| Yellow | Baseline learning |
| Amber | Warning anomaly |
| Red | Critical anomaly or fault |
| Purple | Sensor-health fault |

Warning, critical, and sensor-fault indications may pulse.

## 9. Build and flash

Project directory:

```text
C:\Users\vishn\Documents\projects\Hackathon\my_firmware\craftiai_sentinel
```

Use the ESP-IDF tools from the project directory:

```text
idf.py set-target esp32c3
idf.py build
idf.py -p COM5 flash monitor
```

The Firmgen build tools can also be used through the IDE workflow. The current verified device port is `COM5`; detect the device again if the board is reconnected or the port changes.

Serial monitor speed:

```text
115200 baud
```

Expected startup messages include:

```text
I (...) sensors: I2C device found at 0x68
I (...) sensors: I2C device found at 0x77
I (...) sensors: motion 0x68 WHO_AM_I=0x68 (MPU-6050)
I (...) sensors: pressure 0x77 chip ID=0x55 (BMP180)
I (...) dashboard: dashboard ready at http://192.168.4.1
```

## 10. Configuration

Application settings are in:

```text
firmware/configs/app_config.h
```

Important settings include:

```c
#define APP_I2C_SDA_GPIO 4
#define APP_I2C_SCL_GPIO 5
#define APP_LED_GPIO 8
#define APP_SENSOR_SIMULATION 0
#define APP_SENSOR_BASELINE_REQUIRED_CYCLES 10
#define APP_SENSOR_RAW_PERIOD_MS 5
#define APP_SENSOR_WINDOW_MS 100
#define APP_SENSOR_EMA_ALPHA 0.25f
```

GPIO assignments belong in `app_config.h`, not `sdkconfig` or `sdkconfig.defaults`.

## 11. Source layout

- `firmware/app/` — application startup orchestration
- `firmware/configs/` — application pins and tuning values
- `firmware/interfaces/` — replaceable sensor contracts
- `firmware/platforms/esp32/` — ESP-IDF I²C, sensor, LED, Wi-Fi, and hardware adapters
- `firmware/services/operating_state.c` — machine phase classification
- `firmware/services/baseline_learner.c` — commissioning statistics and NVS baseline storage
- `firmware/services/anomaly_detector.c` — warning/critical persistence and recovery hysteresis
- `firmware/services/commissioning.c` — serial/shared command handling
- `firmware/services/dashboard.c` — SoftAP, HTTP server, embedded webpage, and REST endpoints
- `firmware/services/telemetry.c` — bounded recent telemetry history
- `firmware/utils/` — logging and helper utilities

## 12. Current limitations and next work

- BMP180 provides temperature and pressure, not humidity.
- UTC/NTP timestamps are not yet enabled; current timestamps are monotonic uptime milliseconds.
- Telegram notifications and cloud/web-hosted dashboards are not yet enabled.
- Baseline thresholds are machine-specific and must not be inferred from the synthetic simulator.
- HTTP dashboard authentication is not yet suitable for an exposed production network; use the local SoftAP only during commissioning.
- The temperature reading should be checked against an independent reference before environmental limits are configured.

Next planned work:

1. Complete and validate real-machine baseline commissioning.
2. Test anomaly thresholds using controlled machine conditions.
3. Add UTC/NTP time synchronization.
4. Add secure MQTT telemetry and event publishing.
5. Add Telegram alerts and a remote dashboard.
6. Add a dedicated humidity sensor if required.

## 13. Documentation

Generated API documentation:

```text
firmware/docs/api/html/index.html
```

The public ESP-IDF RESTful HTTP-server example was used only as an API-mechanics reference. The application design, sensor logic, dashboard content, and pin assignments are specific to CraftiAI Sentinel.
