# Cloud-Controlled Embedded IR System (STM32 + ESP32)

## Overview
This project implements a full-stack embedded IoT system that:
- Monitors environment data (temperature, humidity, pressure)
- Sends data to cloud via MQTT (HiveMQ)
- Allows real remote control of appliances via IR
- Works from anywhere (not same WiFi)

## Architecture
BME280 → STM32 (real-time) → UART → ESP32 (MQTT/TLS) → HiveMQ → Web/Mobile
