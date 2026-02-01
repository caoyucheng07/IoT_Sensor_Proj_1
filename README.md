# Cloud-Controlled Embedded IR System (STM32 + ESP32)

## Overview
This project implements a full-stack embedded IoT system that:
- Monitors environment data (temperature, humidity, pressure)
- Sends data to cloud via MQTT (HiveMQ)
- Allows real remote control of appliances via IR
- Works from anywhere (not same WiFi)

## System Architecture:
- Sensor Layer: BME280 (I2C) → STM32F103 (real-time acquisition)
- Control Layer: STM32 IR decoding & replay (36 kHz, timer-based)
- Communication Layer: STM32 ↔ ESP32 (UART)
- Network Layer: ESP32 → MQTT over TLS → HiveMQ Cloud
- Application Layer: Web / Mobile client for monitoring & control
