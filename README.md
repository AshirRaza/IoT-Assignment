# Adaptive IoT Sampling with FFT, MQTT, and LoRaWAN

**Version:** 1.0  
**Updated:** April 4, 2025  
**Board:** Heltec ESP32 V3

---

## Project Summary

This project is an ESP32-based IoT application that:
- Dynamically samples a sensor (or synthetic signal) using an FFT to determine the optimal sampling frequency.
- Aggregates sensor data over a specified time window.
- Transmits aggregated metrics via MQTT (over WiFi) and LoRaWAN (for long-range, low-power communication).
- Evaluates performance metrics including end-to-end latency and data volume.
- Optionally implements deep sleep to reduce power consumption.

This README serves as a detailed walkthrough of the system, describing code organization, system architecture, and how to set up and run the solution.

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Architecture & Key Components](#architecture--key-components)
   - [Tasks](#tasks)
   - [Shared Resources and Synchronization](#shared-resources-and-synchronization)
   - [RTC Memory for State Persistence](#rtc-memory-for-state-persistence)
3. [Code Breakdown and Walkthrough](#code-breakdown-and-walkthrough)
   - [WiFi and MQTT Connectivity](#wifi-and-mqtt-connectivity)
   - [Adaptive Sampling and FFT Analysis](#adaptive-sampling-and-fft-analysis)
   - [Data Aggregation and Evaluation](#data-aggregation-and-evaluation)
   - [LoRaWAN Transmission](#lorawan-transmission)
   - [Power Management and Deep Sleep (Optional)](#power-management-and-deep-sleep-optional)
4. [Setup Instructions](#setup-instructions)
   - [Hardware and Software Prerequisites](#hardware-and-software-prerequisites)
   - [Step-by-Step Setup](#step-by-step-setup)
5. [Performance Evaluation](#performance-evaluation)
6. [Troubleshooting and Additional Notes](#troubleshooting-and-additional-notes)
7. [License](#license)
8. [Acknowledgments](#acknowledgments)

---

## System Overview

The application runs on an ESP32 (Heltec ESP32 V3) and uses FreeRTOS to manage concurrent tasks across the dual cores. The main operational phases of the system are:

1. **Initialization:**  
   The board boots up, initializes WiFi, MQTT, LoRaWAN, and FreeRTOS primitives, and performs a maximum-sampling test to measure the device’s capability.

2. **Processing:**  
   - **Sampling:** The system collects sensor (or generated) data into a buffer.  
   - **FFT Analysis:** Once the buffer fills, an FFT task computes the dominant frequency and adapts the sampling frequency using the Nyquist criterion.  
   - **Aggregation:** Over a fixed window (e.g., 5 seconds), the system computes statistical metrics (mean, median, MSE) of the sensor data.
   - **Communication:** The aggregated data are transmitted via MQTT and also prepared for LoRaWAN.

3. **Evaluation:**  
   The system logs performance metrics such as end-to-end latency (from sampling to MQTT publish) and calculates the total bytes transmitted over MQTT.

4. **(Optional) Power Save Mode:**  
   The device may enter deep sleep between cycles to significantly reduce energy consumption, preserving critical state in RTC memory.

---

## Architecture & Key Components

### Tasks

- **Sampling Task:**  
  Collects samples based on the current sampling rate, updates the aggregate sum, and triggers FFT when the buffer is full.

- **FFT Task:**  
  Processes the sample buffer using FFT to determine the dominant frequency and then adjusts the sampling frequency accordingly.

- **Averaging Task:**  
  Computes the average (and other statistics) over a 5-second window, calculates latency, and publishes data over MQTT and prepares LoRaWAN payload.

- **LoRaWAN State Machine (in loop):**  
  Manages joining, sending, cycling, and sleeping states as per the Heltec LoRaWAN API.

### Shared Resources and Synchronization

- **Buffers & Global Variables:**  
  The sample buffer, aggregate sums, and timing variables are shared among tasks.  
- **Mutexes & Semaphores:**  
  - `dataMutex` protects the sample buffer and sample index.  
  - `aggregateMutex` protects the aggregate values.  
  - `timingMutex` synchronizes modifications to timing parameters (dt and samplingFrequency).

### RTC Memory for State Persistence

Certain variables critical to system operation (e.g., `cycle_count`, `mqtt_bytes_sent_total`, `total_latency_us`, and `latency_count`) are declared with `RTC_DATA_ATTR` so that their values persist through deep sleep cycles.

---

## Code Breakdown and Walkthrough

### WiFi and MQTT Connectivity

The system connects to the configured WiFi network and an MQTT broker. A static IP configuration is used to ensure DNS resolution, which is particularly useful when retrieving NTP time (if integrated).

```cpp
void ensureWiFiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Reconnecting...");
        WiFi.disconnect();
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("\n[WiFi] Reconnected");
    }
}

void ensureMQTTConnection() {
    if (!client.connected()) {
        Serial.print("Connecting to MQTT...");
        while (!client.connected()) {
            String clientId = "ESP32Client-" + String(random(0xffff), HEX);
            if (client.connect(clientId.c_str())) {
                Serial.println(" connected");
            } else {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" trying again in 5s...");
                delay(5000);
            }
        }
    }
}

