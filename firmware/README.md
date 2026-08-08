# CraftiAI Hackathon Sentinel — Phase 1

ESP32-C3-DevKitM-1 vibration-monitoring firmware for the HW-290/GY-91 board.

## Hardware

- MPU-6050 and BMP180 on I2C
- SDA: GPIO4
- SCL: GPIO5
- I2C speed: 400 kHz
- Onboard WS2812 LED: GPIO8 (existing LED behavior preserved)
- No humidity sensor in this phase; BMP180 does not measure humidity.

## Acquisition and filtering

The MPU-6050 is configured for 200 Hz raw acquisition using sample-rate divider 4 and DLPF_CFG=2 (approximately 94 Hz accelerometer bandwidth). Samples are read every 5 ms and aggregated into 100 ms windows.

Static gravity is estimated independently on X/Y/Z and removed before calculating dynamic vibration magnitude. The magnitude is smoothed with the existing EMA:

```text
filtered = alpha * new_value + (1 - alpha) * filtered
```

The default alpha is `0.25f`. Each 100 ms summary includes RMS vibration, peak vibration, valid sample count, error count, latest BMP180 temperature, and monotonic uptime in milliseconds. Output is approximately 10 Hz rather than raw 200 Hz logging.

## Operating states

The provisional state machine contains `OFF`, `STARTING`, `RUNNING`, `STOPPING`, and `FAULT`. Startup and shutdown persistence timers prevent normal transients from immediately becoming faults. Defaults are commissioning values and must be replaced or tuned through the later baseline trial:

- Start: 0.08 g RMS
- Stop: 0.05 g RMS
- Fault: 0.30 g RMS
- Recovery: 0.20 g RMS
- Startup settling: 10 s
- Stop confirmation: 2 s
- Fault persistence: 2 s
- Fault clear persistence: 5 s

The later phases will add baseline learning, UTC/NTP timestamps, LED anomaly colors, dashboard connectivity, Telegram alerts, and humidity hardware.

## Source boundaries

`interfaces/vibration_source.h` defines the replaceable accelerometer sample contract. `platforms/esp32/sensor_platform.c` owns I2C and sensor-specific acquisition. `services/operating_state.c` owns phase classification. The LED platform remains independent.

All pins and commissioning values are in `configs/app_config.h`.
