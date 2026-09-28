# LTE LCD Project

Standalone ESP-IDF application for the ESP32-S3 DataMeter board.

It starts the Cavli C16qs 4G modem, connects to MQTT, and displays only LTE
and MQTT diagnostics on the 128x64 ST7920 LCD.

## LCD pages

The display cycles through two pages:

1. Network, SIM, RSSI, band, IP, and registration state.
2. MQTT connection state, reason code, sent/failed counts, and recovery state.

## Hardware used

- Modem UART2: TX GPIO 41, RX GPIO 40, 115200 baud
- ST7920 LCD: CS GPIO 1, data GPIO 11, clock GPIO 12, reset GPIO 2

## Configure before building

Edit `main/app_config.h` and set the MQTT broker, port, client ID, user,
password, and topic. The reused modem manager currently configures the modem
with the existing firmware's `jionet` APN.

## Build

From this directory:

```text
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

The project references the original repository components through
`EXTRA_COMPONENT_DIRS`, so keep this folder beside the repository's
`components` directory.

## Free HTTPS OTA test

Enable GitHub Pages for the `main` branch and repository root. The firmware
URL will be:

```text
https://YOUR_USERNAME.github.io/lte-ota-test/ota/lte_lcd_project-3.1.5.bin
```
