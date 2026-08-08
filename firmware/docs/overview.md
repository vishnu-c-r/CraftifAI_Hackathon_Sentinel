# CraftiAI Sentinel — Device-hosted dashboard

Sentinel runs on an ESP32-C3-DevKitM-1 with an MPU-6050 and BMP180 on I2C, while retaining the onboard WS2812 LED on GPIO8.

## Hardware and dashboard

- I2C SDA: GPIO4
- I2C SCL: GPIO5
- MPU-6050: detected at 0x68
- BMP180: detected at 0x77
- WS2812 LED: GPIO8
- Dashboard SoftAP: `Sentinel-Setup`
- Dashboard address: `http://192.168.4.1`

The embedded dashboard is based on the public ESP-IDF RESTful HTTP-server example mechanics and uses the native `esp_http_server` component. No Python or FastAPI runtime is placed on the ESP32-C3.

## Controls and telemetry

Serial and dashboard commands use the same command names: `START_BASELINE`, `STOP_BASELINE`, `RESET_BASELINE`, `BASELINE_STATUS`, and `HELP`. The dashboard status endpoint reports baseline learning state, validity, and completed cycles. The sensor task continues 200 Hz acquisition and emits 100 ms telemetry summaries containing phase, vibration RMS/peak, temperature, sample/error counts, baseline state, and anomaly level.

Baseline commissioning requires ten complete ordered real-machine cycles. A valid baseline is stored in NVS only after validation. Synthetic data is disabled and cannot become a production baseline.

## LED status

The GPIO8 LED reports local status: off for machine OFF, blue for startup/stopping, green for normal running, yellow for baseline learning, amber for warning, red for critical anomaly, and purple for sensor fault.

## Validation

The current firmware built, flashed to COM5, detected the MPU-6050 and BMP180, started the SoftAP and HTTP server, and ran without panic or reboot loop. Connect a client to the SoftAP and open `http://192.168.4.1` to exercise the dashboard.
