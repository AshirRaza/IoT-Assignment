# IoT-Assignment
This project implements an IoT system on the Heltec WiFi LoRa 32 V3 board that evaluates the performance of adaptive signal sampling using local FFT analysis, compared to traditional fixed oversampling. The system periodically transmits the computed average of the signal via MQTT over WiFi.

# Adaptive vs Fixed Sampling IoT System (FFT + MQTT)

This project implements an IoT system on the **Heltec WiFi LoRa 32 V3** board that evaluates the performance of **adaptive signal sampling** using **local FFT analysis**, compared to traditional fixed oversampling. The system periodically transmits the computed average of the signal via **MQTT** over WiFi.

## Objective

The aim is to compare **adaptive** and **fixed** (oversampling) signal acquisition strategies based on:

- Energy efficiency (qualitative analysis)
- Network data volume
- End-to-end latency

These evaluations are performed using different types of simulated input signals.

---

## System Features

- **Signal Sampling**: Samples analog input on a fixed or adaptively adjusted rate.
- **FFT Analysis**: Performs FFT on collected samples to estimate dominant frequency.
- **Adaptive Sampling**: Adjusts sampling frequency to twice the dominant frequency.
- **Fixed Sampling**: Assumes a maximum (pre-computed) sampling rate.
- **Averaging & Transmission**: Aggregates and transmits average value every 5 seconds.
- **Performance Evaluation**: Logs fixed vs adaptive sample count, data volume reduction, and latency.

---

## Project Structure

```
main.ino
├── setup()                        → Initializes WiFi, MQTT, and creates tasks
├── loop()                         → Maintains MQTT connection
├── samplingTask()                → Samples the signal at current dt interval
├── fftTask()                     → Computes FFT and updates sampling rate
├── averagingTask()               → Calculates 5s average, evaluates performance
├── maxSamplingTestTask()        → Measures max analog sampling rate
├── generateSignal()             → Simulates signal (selectable)
├── ensureWiFiConnection()       → Maintains WiFi connection
├── ensureMQTTConnection()       → Maintains MQTT connection
```

---

## Signal Types

You can manually choose one of the three signals by editing this line before uploading:

```cpp
int signalType = 0; // Options: 0, 1, or 2
```

| Value | Description                        |
|-------|------------------------------------|
| 0     | Composite Signal (3Hz + 5Hz)       |
| 1     | High Frequency Signal (10Hz)       |
| 2     | Low-Frequency Mixed (0.5Hz + 2Hz)  |

---

## Setup Instructions

### 1. Hardware
- **Board**: Heltec WiFi LoRa 32 V3
- **Pin Used**: Analog input on GPIO 1 (change `ANALOG_PIN` if needed)

### 2. Libraries to Install
Install the following from Arduino Library Manager:

- `PubSubClient` (for MQTT)
- `arduinoFFT`

Already included:

- `WiFi.h`
- `FreeRTOS`

### 3. Configuration

In `main.ino`, set your WiFi credentials and MQTT broker:

```cpp
const char* ssid = "YourSSID";
const char* password = "YourPassword";
const char* mqtt_server = "broker.hivemq.com"; // or test.mosquitto.org
```

### 4. Upload & Monitor

- Open Arduino IDE
- Select the board: **Heltec WiFi LoRa 32 V3**
- Connect the board via USB
- Upload the code
- Open Serial Monitor (115200 baud) to observe output

---

## Performance Metrics (Logged Every 5 Seconds)

- **Fixed samples**: Based on max sampling frequency
- **Adaptive samples**: Based on FFT-determined sampling frequency
- **Data reduction**: Percentage decrease in samples transmitted
- **Latency**: Time from aggregation start to publish

Example output:

```
[MAX TEST] Max Frequency: 15934.78 Hz
[FFT] Dominant: 5.01 Hz | Sampling Freq: 10.02 Hz (dt = 0.0998 s)
[AVERAGE] Over last 5 sec: 0.0855 (Samples: 55)
[EVAL] Fixed samples: 79671, Adaptive samples: 55
[EVAL] Data volume reduction: 99.93%
[EVAL] End-to-end latency: 5014 ms
[MQTT] Published to esp32/average: 0.0855
```

---

## Energy Consumption (Qualitative Notes)

While no deep sleep is implemented, the adaptive system:
- Takes fewer samples per window
- Reduces CPU and ADC usage
- Lowers MQTT transmission payload size

These reduce overall energy consumption per interval.

---

## Summary

This system highlights the tradeoffs between high-frequency sampling and adaptive signal-aware sampling in an embedded IoT environment. It provides real-time insight into data transmission volume and latency — essential metrics in energy- and bandwidth-constrained systems like remote sensing or edge analytics.

