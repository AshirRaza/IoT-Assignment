# Adaptive IoT Sampling with Dual Transmission on Heltec ESP32 V3

**Version:** 1.0  
**Last Updated:** April 4, 2025  
**Board:** Heltec ESP32 V3

---

## Overview

This project demonstrates an IoT system that dynamically adapts the sampling frequency based on signal characteristics using FFT, aggregates the sensor data, and then transmits the aggregated values via MQTT (over WiFi) and LoRaWAN. The primary objectives of this system are:

- **Energy Efficiency:** Lower power consumption by reducing the sampling rate to only what is needed.
- **Reduced Communication Overhead:** By aggregating data, the volume of transmitted data is minimized.
- **Performance Evaluation:** The system calculates performance metrics such as end-to-end latency and data volume transmitted.

This README provides a detailed walkthrough of every component of the code, explains the system architecture, and offers setup instructions to help you run the system on your own Heltec ESP32 V3.

---

## System Architecture

The system is built using FreeRTOS on the ESP32 and is divided into several concurrent tasks. Each task is designed to handle a specific function, ensuring efficient use of resources and parallel processing. Here is a summary:

1. **Max Sampling Test Task:**  
   Measures the maximum achievable sampling frequency using the analog input.

2. **Sampling Task:**  
   Continuously generates samples (or reads sensor data) based on a defined signal function. It updates an aggregate (sum and count) and fills a sample buffer for FFT processing.

3. **FFT Task:**  
   When the sample buffer fills up, this task is notified and performs FFT processing. It determines the dominant frequency in the signal and adjusts the sampling frequency accordingly (using the Nyquist criterion). When the dominant frequency stabilizes over two consecutive cycles, it “locks” the adaptive sampling rate.

4. **Averaging Task:**  
   Aggregates the collected samples over a 5-second window, computes the average sensor reading, and then publishes this value via MQTT. It also prepares a LoRaWAN payload with the same aggregated value. In addition, this task calculates performance metrics:
   - **Latency:** The time from the first sample (recorded by `sample_timestamp`) until the MQTT publish time (`publish_timestamp`).
   - **Data Volume:** An estimate is computed by summing the lengths (in bytes) of the MQTT topic and payload.
  
5. **LoRaWAN State Machine (in loop):**  
   Implements the state machine (init, join, send, cycle, sleep) from the Heltec LoRaWan_APP library to handle the LoRaWAN communication process.

6. **(Optional) Deep Sleep:**  
   Although not directly called in this code, the structure is designed so that you can integrate deep sleep cycles between active processing periods. Critical state variables are stored in RTC memory (via `RTC_DATA_ATTR`) to preserve progress between sleep cycles.

---

## Detailed Code Walkthrough

Below is the complete code with inline explanations.

### Header Includes and Global Definitions

```cpp
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <arduinoFFT.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <heltec.h>
#include "LoRaWan_APP.h"
#include <esp_timer.h>

// --- WiFi and MQTT Configuration ---
const char* ssid = "Ashir";
const char* password = "Ashir1234";
const char* mqtt_server = "test.mosquitto.org";
const char* mqtt_topic = "esp32/average";

WiFiClient espClient;
PubSubClient client(espClient);

// --- Task Handles ---
TaskHandle_t fftTaskHandle = NULL;

// --- Semaphores ---
// dataMutex protects the sample buffers and sample index,
// aggregateMutex protects the running sum and sample count for averaging,
// timingMutex is used to safely update timing-related variables (sampling frequency, dt).
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t aggregateMutex;
SemaphoreHandle_t timingMutex;

// --- Constants ---
#define SAMPLE_COUNT 512  // Number of samples per FFT block
#define MAX_SAMPLING_TEST_COUNT 1000  // Number of iterations for maximum sampling test
#define ANALOG_PIN 1  // Analog pin used for maximum sampling test (or for reference)
#define AVG_WINDOW_MS 5000  // Aggregation window duration in ms
