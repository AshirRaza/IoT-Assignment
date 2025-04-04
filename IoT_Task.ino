// Updated on 2025-04-04 at 07:10:43
// Heltec V3 - Adaptive vs Fixed Sampling Evaluation (FFT + MQTT)

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <arduinoFFT.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------- WiFi & MQTT ----------
const char* ssid = "Ashir";
const char* password = "Ashir1234";
const char* mqtt_server = "test.mosquitto.org";
const char* mqtt_topic = "esp32/average";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------- Constants ----------
#define SAMPLE_COUNT 512
#define MAX_SAMPLING_TEST_COUNT 1000
#define ANALOG_PIN 1
#define AVG_WINDOW_MS 5000

// ---------- Signal & FFT ----------
double vReal[SAMPLE_COUNT];
double vImag[SAMPLE_COUNT];
float t = 0.0;
double dt;
double samplingFrequency;
double maxSamplingFrequency = 0.0;
int sampleIndex = 0;
bool maxSamplingMeasured = false;

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLE_COUNT, 1000.0);

// ---------- Aggregation ----------
double sumAggregate = 0.0;
int aggregateSampleCount = 0;
unsigned long lastAggregateTime = 0;
unsigned long avgStartTime = 0;
unsigned long avgEndTime = 0;

// ---------- Evaluation ----------
double totalSamplesFixed = 0;
double totalSamplesAdaptive = 0;
int signalType = 2; // <-- CHANGE THIS TO 0, 1, or 2 MANUALLY BEFORE UPLOADING

// ---------- Function Prototypes ----------
void ensureWiFiConnection();
void ensureMQTTConnection();
float generateSignal(float t, int type);

// ---------- Signal Generator ----------
float generateSignal(float t, int type) {
  switch (type) {
    case 0: return 2.0 * sin(2 * PI * 3 * t) + 4.0 * sin(2 * PI * 5 * t);  // Composite
    case 1: return sin(2 * PI * 10 * t);                                   // High frequency
    case 2: return sin(2 * PI * 2 * t) + 0.5 * sin(2 * PI * 0.5 * t);      // Mixed low-frequency
    default: return 0;
  }
}

// ---------- Connection Management ----------
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
        Serial.println("connected");
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" trying again in 5s...");
        delay(5000);
      }
    }
  }
}

// ---------- Tasks ----------
void maxSamplingTestTask(void *parameter) {
  int dummy;
  unsigned long start = micros();
  for (int i = 0; i < MAX_SAMPLING_TEST_COUNT; i++) {
    dummy = analogRead(ANALOG_PIN);
  }
  unsigned long end = micros();
  float elapsed = (end - start) / 1000.0;
  samplingFrequency = MAX_SAMPLING_TEST_COUNT / (elapsed / 1000.0);
  maxSamplingFrequency = samplingFrequency;
  dt = 1.0 / samplingFrequency;
  FFT = ArduinoFFT<double>(vReal, vImag, SAMPLE_COUNT, samplingFrequency);
  Serial.printf("[MAX TEST] Max Frequency: %.2f Hz\n", samplingFrequency);
  maxSamplingMeasured = true;
  vTaskDelete(NULL);
}

void samplingTask(void *parameter) {
  while (!maxSamplingMeasured) vTaskDelay(100 / portTICK_PERIOD_MS);
  while (1) {
    if (sampleIndex < SAMPLE_COUNT) {
      float sample = generateSignal(t, signalType);
      vReal[sampleIndex] = sample;
      vImag[sampleIndex] = 0.0;
      sampleIndex++;
      t += dt;
      sumAggregate += sample;
      aggregateSampleCount++;
    }
    vTaskDelay(pdMS_TO_TICKS(dt * 1000));
  }
}

void fftTask(void *parameter) {
  while (!maxSamplingMeasured) vTaskDelay(100 / portTICK_PERIOD_MS);
  while (1) {
    if (sampleIndex >= SAMPLE_COUNT) {
      FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
      FFT.compute(FFTDirection::Forward);
      FFT.complexToMagnitude();
      double peak = FFT.majorPeak();
      if (peak < 1.0) peak = 1.0;
      samplingFrequency = max(2.0 * peak, 2.0);
      dt = 1.0 / samplingFrequency;
      FFT = ArduinoFFT<double>(vReal, vImag, SAMPLE_COUNT, samplingFrequency);
      totalSamplesAdaptive += SAMPLE_COUNT;
      Serial.printf("[FFT] Dominant: %.2f Hz | Sampling Freq: %.2f Hz (dt = %.4f s)\n", peak, samplingFrequency, dt);
      sampleIndex = 0;
      t = 0.0;
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void averagingTask(void *parameter) {
  while (!maxSamplingMeasured) vTaskDelay(100 / portTICK_PERIOD_MS);
  lastAggregateTime = millis();
  while (1) {
    unsigned long now = millis();
    if (now - lastAggregateTime >= AVG_WINDOW_MS) {
      avgEndTime = millis();
      if (aggregateSampleCount > 0) {
        double average = sumAggregate / aggregateSampleCount;
        Serial.printf("[AVERAGE] Over last 5 sec: %.4f (Samples: %d)\n", average, aggregateSampleCount);

        double expectedFixedSamples = maxSamplingFrequency * (AVG_WINDOW_MS / 1000.0);
        totalSamplesFixed += expectedFixedSamples;
        double reductionPercent = 100.0 * (1.0 - (aggregateSampleCount / expectedFixedSamples));
        unsigned long latency = avgEndTime - lastAggregateTime;

        Serial.printf("[EVAL] Fixed samples: %.0f, Adaptive samples: %d\n", expectedFixedSamples, aggregateSampleCount);
        Serial.printf("[EVAL] Data volume reduction: %.2f%%\n", reductionPercent);
        Serial.printf("[EVAL] End-to-end latency: %lu ms\n", latency);

        ensureMQTTConnection();
        char msg[50];
        snprintf(msg, sizeof(msg), "%.4f", average);
        if (client.publish(mqtt_topic, msg)) {
          Serial.printf("[MQTT] Published to %s: %s\n", mqtt_topic, msg);
        } else {
          Serial.println("[MQTT] Publish failed - will retry next cycle");
        }
      } else {
        Serial.println("[AVERAGE] No samples to average.");
      }
      sumAggregate = 0.0;
      aggregateSampleCount = 0;
      lastAggregateTime = now;
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Adaptive Sampling + Evaluation + MQTT");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  client.setServer(mqtt_server, 1883);
  client.setKeepAlive(60);
  ensureMQTTConnection();

  xTaskCreatePinnedToCore(maxSamplingTestTask, "MaxSamplingTest", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(samplingTask, "SamplingTask", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(fftTask, "FFTTask", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(averagingTask, "AveragingTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
  ensureWiFiConnection();
  client.loop();
}