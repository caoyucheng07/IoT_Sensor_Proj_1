# Cloud-Controlled Embedded IR System (STM32 + ESP32)

## Overview
This project implements a full-stack embedded IoT system that:
- Monitors environment data (temperature, humidity, pressure)
- Sends data to cloud via MQTT (HiveMQ)
- Allows real remote control of appliances via IR
- Works from anywhere (not same WiFi)

## Architecture
[BME280 Sensor]
      │ I2C
      ▼
[STM32 (Real-time Control)]
      │ UART
      ▼
[ESP32 (Network + MQTT/TLS)]
      │ Internet
      ▼
[HiveMQ Broker]
      │
      ▼
[Web / Mobile UI]
