# Adaptive IoT Sampling with FFT, MQTT, and LoRaWAN

**Version:** 1.0  
**Updated:** April 15, 2025  
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

## Code Review and Walkthrough

This section explains the functionality and design choices of the main components of the code without reproducing the full source. The following is a breakdown of each module and its role in the system:

### 1. WiFi and MQTT Connectivity
- **Purpose:**  
  Establish and maintain the network connection.
- **Key Functions:**  
  - `ensureWiFiConnection()`: Checks whether the ESP32 is connected to WiFi; if not, it disconnects any stale connection and reconnects.
  - `ensureMQTTConnection()`: Attempts to connect to the MQTT broker repeatedly until successful.
- **Design Rationale:**  
  Using blocking loops and delays ensures that the device reliably connects to your network and MQTT broker before proceeding with sensor tasks.

### 2. Adaptive Sampling and FFT Analysis
- **Sampling Task:**  
  - **Role:** Continuously generates (or collects) sensor data using a simulated signal (or actual sensor input) and stores the data in a buffer.
  - **Mechanism:**  
    - Increments a logical time variable `t` using a dynamically determined time step `dt`.
    - Updates a running sum and count for later aggregation.
    - Notifies the FFT task when the sample buffer is full.
- **FFT Task:**  
  - **Role:** Processes the filled sample buffer to compute the FFT.
  - **Mechanism:**  
    - Applies a Hamming window to the data before computing the FFT.
    - Determines the dominant frequency by searching for the highest magnitude in the FFT result.
    - Updates the sampling frequency (using the Nyquist criterion: approximately 2× the dominant frequency) and adjusts the time increment `dt` accordingly.
    - Locks the adaptive frequency when successive FFT cycles show a stable dominant frequency.
- **Design Rationale:**  
  This adaptive approach minimizes oversampling and helps in conserving computational resources and energy by aligning the sampling rate with the signal characteristics.

### 3. Data Aggregation and Evaluation
- **Averaging Task:**  
  - **Role:** Calculates statistics (average, and potentially median, MSE, etc.) over a fixed time window.
  - **Mechanism:**  
    - Aggregates the sum of the samples and counts the total number collected over the window.
    - Computes the average value and prints this value along with the number of samples used.
- **Performance Metrics:**  
  - **Latency Measurement:**  
    - Timestamps are captured at the beginning of the sampling cycle and just before publishing via MQTT.
    - The latency (in microseconds) is computed as the difference between these timestamps and then averaged over multiple cycles.
  - **Data Volume Calculation:**  
    - The code estimates the data volume transmitted by summing the lengths of the MQTT topic and payload for each publication.
- **Design Rationale:**  
  Aggregating the data reduces the overall network communication overhead. Meanwhile, tracking performance metrics allows for later evaluation of system improvements (such as energy savings and reduced network usage).

### 4. Communication – MQTT and LoRaWAN
- **MQTT Publication:**  
  - **Role:** Publishes the aggregated sensor value to a specified MQTT topic.
  - **Mechanism:**  
    - The averaged result is converted to a string and sent using the PubSubClient library.
    - The byte count of each transmitted message (topic plus payload) is added to a global counter.
- **LoRaWAN Transmission:**  
  - **Role:** Prepares and sends the aggregated result over LoRaWAN.
  - **Mechanism:**  
    - The helper function formats the result into a byte array (payload) and then triggers transmission using Heltec's LoRaWAN API.
- **Design Rationale:**  
  Dual transmission ensures that data reaches both a local edge server (via MQTT) and a remote cloud solution (via LoRaWAN), leveraging the strengths of each communication protocol.

### 5. Power Management and Deep Sleep (Optional)
- **Deep Sleep Integration:**  
  - **Purpose:**  
    To minimize power consumption by putting the ESP32 into a low-power state between active data collection/transmission cycles.
  - **Mechanism:**  
    - A dedicated function (e.g., `goToDeepSleep()`) disconnects WiFi, shuts down unnecessary peripherals, and invokes `esp_deep_sleep_start()` for a preset duration.
    - Critical variables (such as cycle count and performance metrics) are stored in RTC memory using the `RTC_DATA_ATTR` qualifier so that their values persist across deep sleep cycles.
- **Design Rationale:**  
  Deep sleep dramatically reduces power usage in battery-powered deployments, at the cost of brief processing interruptions—which is acceptable in many IoT applications.

### 6. Integration and Flow Control
- **Main Loop and State Machine:**  
  - **Role:**  
    Executes the LoRaWAN state machine, handling device initialization, joining, sending, cycling, and sleep.
  - **Mechanism:**  
    - The main loop reads the `deviceState` variable and calls LoRaWAN functions accordingly.
    - It calls `ensureWiFiConnection()` and `client.loop()` to maintain network connectivity.
- **Overall Flow:**  
  1. The system begins with a maximum sampling test.
  2. Then, it enters the sampling phase, where data is collected and aggregated.
  3. FFT analysis dynamically adjusts the sampling rate.
  4. The aggregated result is transmitted via MQTT and prepared for LoRaWAN transmission.
  5. Performance metrics (latency and data volume) are logged.
  6. Optionally, the device enters deep sleep to save power before the next cycle.

---



