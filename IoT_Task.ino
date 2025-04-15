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

// ---------- WiFi & MQTT ----------
const char* ssid = "Ashir";
const char* password = "Ashir1234";
const char* mqtt_server = "test.mosquitto.org";
const char* mqtt_topic = "esp32/average";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------- Task Handles ----------
TaskHandle_t fftTaskHandle = NULL;

// ---------- Semaphores ----------
SemaphoreHandle_t dataMutex;      // Protects vReal, vImag, sampleIndex
SemaphoreHandle_t aggregateMutex; // Protects sumAggregate, aggregateSampleCount
SemaphoreHandle_t timingMutex;    // Protects samplingFrequency, dt

// ---------- Constants ----------
#define SAMPLE_COUNT 512
#define MAX_SAMPLING_TEST_COUNT 1000
#define ANALOG_PIN 1
#define AVG_WINDOW_MS 5000

// ---------- Signal & FFT ----------
double vReal[SAMPLE_COUNT];
double vImag[SAMPLE_COUNT];
float t = 0.0;               // Logical time for generating signal
double dt;                   // Will be set based on FFT peak frequency
double samplingFrequency;    // Will be set to 2 * peak frequency (Nyquist)
double maxSamplingFrequency = 0.0;
volatile int sampleIndex = 0;
bool maxSamplingMeasured = false;
unsigned long lastFFTTime = 0;
bool frequencyLocked = false;  // New flag to indicate if we've locked onto a stable frequency
volatile bool fftTriggerSent = false;  // New flag to control FFT triggering
int signalType = 1;  // Add signal type variable (0 = composite, 1 = high frequency, 2 = mixed low-frequency)

// Initialize FFT with temporary frequency
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLE_COUNT, 1000.0);

// ---------- Aggregation ----------
double sumAggregate = 0.0;
int aggregateSampleCount = 0;
unsigned long lastAggregateTime = 0;
unsigned long avgStartTime = 0;
unsigned long avgEndTime = 0;

// ---------- LoRaWAN Parameters ----------
/* OTAA parameters */
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0xFA, 0xCE };
uint8_t appEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x03, 0xD0, 0x00 };
uint8_t appKey[] = { 0x5D, 0x81, 0x15, 0x42, 0xC7, 0x32, 0x82, 0xC6, 0x3E, 0x36, 0x8C, 0x65, 0x1C, 0x9E, 0x37, 0x22 };

/* ABP parameters - needed even when using OTAA */
uint8_t nwkSKey[] = { 0x15, 0xB1, 0xD0, 0xEF, 0xA4, 0x63, 0xDF, 0xBE, 0x3D, 0x11, 0x18, 0x1E, 0x1E, 0xC7, 0xDA, 0x85 };
uint8_t appSKey[] = { 0xD7, 0x2C, 0x78, 0x75, 0x8C, 0xDC, 0xCA, 0xBF, 0x55, 0xEE, 0x4A, 0x77, 0x8D, 0x16, 0xEF, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

/* LoRaWAN region and class */
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_EU868;
DeviceClass_t loraWanClass = CLASS_A;

/* Application port and duty cycle */
uint8_t appPort = 2;
uint32_t appTxDutyCycle = 15000;

/* OTAA/ADR settings */
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t confirmedNbTrials = 4;

/* Channels mask */
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

// Add RTC variables for metrics
RTC_DATA_ATTR int cycle_count = 0;
RTC_DATA_ATTR int mqtt_bytes_sent_total = 0;
RTC_DATA_ATTR uint64_t total_latency_us = 0;
RTC_DATA_ATTR int latency_count = 0;

// Add timestamp variables
uint64_t sample_timestamp = 0;
uint64_t publish_timestamp = 0;

// ---------- Function Prototypes ----------
void ensureWiFiConnection();
void ensureMQTTConnection();
float generateSignal(float t, int type);
void prepareTxFrame(uint8_t port, float aggValue);

// ---------- Signal Generator ----------
float generateSignal(float t, int type) {
    switch (type) {
        case 0: return 2.0 * sin(2 * PI * 3 * t) + 4.0 * sin(2 * PI * 5 * t);  // Composite
        case 1: return sin(2 * PI * 100 * t);                                   // High frequency
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
    
    if (xSemaphoreTake(timingMutex, portMAX_DELAY) == pdTRUE) {
        samplingFrequency = MAX_SAMPLING_TEST_COUNT / (elapsed / 1000.0);
        maxSamplingFrequency = samplingFrequency;
        dt = 1.0 / samplingFrequency;
        FFT = ArduinoFFT<double>(vReal, vImag, SAMPLE_COUNT, samplingFrequency);
        xSemaphoreGive(timingMutex);
    }
    
    Serial.printf("[MAX TEST] Max Frequency: %.2f Hz\n", samplingFrequency);
    maxSamplingMeasured = true;
    vTaskDelete(NULL);
}

void samplingTask(void *parameter) {
    while (!maxSamplingMeasured) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    float currentDt;
    while (1) {
        // Get current dt value safely
        if (xSemaphoreTake(timingMutex, portMAX_DELAY) == pdTRUE) {
            currentDt = dt;
            xSemaphoreGive(timingMutex);
        }
        
        // Try to take both mutexes (prevent deadlock by always taking in same order)
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(aggregateMutex, portMAX_DELAY) == pdTRUE) {
                if (sampleIndex < SAMPLE_COUNT) {
                    float sample = generateSignal(t, signalType);
                    vReal[sampleIndex] = sample;
                    vImag[sampleIndex] = 0.0;
                    sampleIndex++;
                    t += currentDt;
                    sumAggregate += sample;
                    aggregateSampleCount++;
                    
                    // Only trigger FFT when buffer is full and we're not locked
                    if (sampleIndex == SAMPLE_COUNT && !fftTriggerSent && !frequencyLocked) {
                        fftTriggerSent = true;
                        xTaskNotify(fftTaskHandle, 0, eNoAction);  // Notify FFT task
                    } else if (sampleIndex == SAMPLE_COUNT) {
                        // Reset buffer without triggering FFT if we're locked
                        sampleIndex = 0;
                    }
                }
                xSemaphoreGive(aggregateMutex);
            }
            xSemaphoreGive(dataMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(currentDt * 1000));
    }
}

void fftTask(void *parameter) {
    while (!maxSamplingMeasured) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    double lastPeak = 0.0;
    int stableCount = 0;
    
    while (!frequencyLocked) {  // Only run until frequency is locked
        uint32_t notification;
        
        // Wait for notification from sampling task
        if (xTaskNotifyWait(0, 0, &notification, portMAX_DELAY) == pdTRUE) {
            lastFFTTime = millis();
            
            if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
                if (sampleIndex == SAMPLE_COUNT) {
                    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
                    FFT.compute(FFTDirection::Forward);
                    FFT.complexToMagnitude();
                    double currentPeak = FFT.majorPeak();
                    
                    // Ensure a minimum peak frequency to avoid division by zero
                    if (currentPeak < 1.0) {
                        currentPeak = 1.0;
                    }
                    
                    // Check if the current peak is within 1% of the last peak
                    if (lastPeak != 0.0 && fabs(currentPeak - lastPeak) < (0.01 * lastPeak)) {
                        stableCount++;
                        Serial.printf("[FFT] Stable cycle count: %d (Peak ~ %.2f Hz)\n", 
                                    stableCount, currentPeak);
                        
                        // After we've seen the same dominant frequency twice, lock it in
                        if (stableCount >= 2) {
                            if (xSemaphoreTake(timingMutex, portMAX_DELAY) == pdTRUE) {
                                samplingFrequency = 2.0 * currentPeak;  // Nyquist frequency
                                dt = 1.0 / samplingFrequency;
                                xSemaphoreGive(timingMutex);
                                
                                Serial.printf("[FFT] Dominant freq stable at ~%.2f Hz; locking in Fs = %.2f Hz (dt = %.4f s)\n",
                                            currentPeak, samplingFrequency, dt);
                                frequencyLocked = true;  // Lock in the frequency and stop FFT processing
                            }
                        }
                    } else {
                        stableCount = 0;
                        // If frequency is not stable, adjust sampling frequency
                        if (xSemaphoreTake(timingMutex, portMAX_DELAY) == pdTRUE) {
                            samplingFrequency = 2.0 * currentPeak;  // Nyquist frequency
                            dt = 1.0 / samplingFrequency;
                            xSemaphoreGive(timingMutex);
                            
                            Serial.printf("[FFT] (Adaptive) Peak = %.2f Hz => New Fs = %.2f Hz (dt = %.4f s)\n",
                                        currentPeak, samplingFrequency, dt);
                        }
                    }
                    
                    lastPeak = currentPeak;
                    
                    // Reinitialize FFT with the updated sampling frequency
                    FFT = ArduinoFFT<double>(vReal, vImag, SAMPLE_COUNT, samplingFrequency);
                    
                    // Reset buffer
                    sampleIndex = 0;
                    fftTriggerSent = false;
                }
                xSemaphoreGive(dataMutex);
            }
            
            unsigned long fftDuration = millis() - lastFFTTime;
            Serial.printf("[FFT] Processing time: %lu ms\n", fftDuration);
        }
    }
    
    // Once we've locked the frequency, print a message and delete this task
    Serial.println("[FFT] Frequency locked. FFT task complete.");
    vTaskDelete(NULL);
}

void prepareTxFrame(uint8_t port, float aggValue) {
    appDataSize = 4;
    memcpy(appData, &aggValue, sizeof(aggValue));
    Serial.printf("[LoRaWAN] Payload prepared on port %d: ", port);
    for (int i = 0; i < appDataSize; i++) {
        Serial.printf("%02X ", appData[i]);
    }
    Serial.println();
}

void averagingTask(void *parameter) {
    while (!maxSamplingMeasured) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    lastAggregateTime = millis();
    sample_timestamp = esp_timer_get_time(); // Record start time
    
    while (1) {
        unsigned long now = millis();
        
        if (now - lastAggregateTime >= AVG_WINDOW_MS) {
            double localSum = 0;
            int localCount = 0;
            
            if (xSemaphoreTake(aggregateMutex, portMAX_DELAY) == pdTRUE) {
                localSum = sumAggregate;
                localCount = aggregateSampleCount;
                sumAggregate = 0.0;
                aggregateSampleCount = 0;
                xSemaphoreGive(aggregateMutex);
            }
            
            if (localCount > 0) {
                double average = localSum / localCount;
                Serial.printf("[AVERAGE] Over last 5 sec: %.4f (Samples: %d)\n", average, localCount);
                
                // MQTT Publishing with metrics tracking
                ensureMQTTConnection();
                char msg[50];
                snprintf(msg, sizeof(msg), "%.4f", average);
                
                if (client.publish(mqtt_topic, msg)) {
                    mqtt_bytes_sent_total += strlen(mqtt_topic) + strlen(msg); // Track bytes sent
                    publish_timestamp = esp_timer_get_time(); // Record publish time
                    
                    // Calculate and track latency
                    uint64_t latency_us = publish_timestamp - sample_timestamp;
                    total_latency_us += latency_us;
                    latency_count++;
                    
                    float avg_latency_ms = (float)(total_latency_us / latency_count) / 1000.0;
                    Serial.printf("[METRICS] Latency: %.2f ms, Total MQTT Bytes: %d\n", 
                                avg_latency_ms, mqtt_bytes_sent_total);
                    
                    Serial.printf("[MQTT] Published to %s: %s\n", mqtt_topic, msg);
                } else {
                    Serial.println("[MQTT] Publish failed - will retry next cycle");
                }
                
                // LoRaWAN Transmission
                prepareTxFrame(appPort, (float)average);
                deviceState = DEVICE_STATE_SEND;
            } else {
                Serial.println("[AVERAGE] No samples to average.");
            }
            
            sample_timestamp = esp_timer_get_time(); // Reset timestamp for next window
            lastAggregateTime = millis();
        }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Starting Adaptive Sampling + Evaluation + MQTT + LoRaWAN");
    
    // Print cycle information
    Serial.printf("Cycle Count: %d\n", cycle_count);
    if (latency_count > 0) {
        float avg_latency_ms = (float)(total_latency_us / latency_count) / 1000.0;
        Serial.printf("Average Latency: %.2f ms\n", avg_latency_ms);
    }
    Serial.printf("Total MQTT Bytes Sent: %d\n", mqtt_bytes_sent_total);
    
    // Initialize LoRaWAN
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
    deviceState = DEVICE_STATE_INIT;
    
    // Create semaphores
    dataMutex = xSemaphoreCreateMutex();
    aggregateMutex = xSemaphoreCreateMutex();
    timingMutex = xSemaphoreCreateMutex();
    
    if (!dataMutex || !aggregateMutex || !timingMutex) {
        Serial.println("Error creating semaphores!");
        while(1);  // Halt if semaphores couldn't be created
    }
    
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
    xTaskCreatePinnedToCore(fftTask, "FFTTask", 8192, NULL, 1, &fftTaskHandle, 0);
    xTaskCreatePinnedToCore(averagingTask, "AveragingTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
    switch (deviceState) {
        case DEVICE_STATE_INIT:
        {
#if (LORAWAN_DEVEUI_AUTO)
            LoRaWAN.generateDeveuiByChipID();
#endif
            LoRaWAN.init(loraWanClass, loraWanRegion);
            LoRaWAN.setDefaultDR(3);
            deviceState = DEVICE_STATE_JOIN;
            break;
        }
        case DEVICE_STATE_JOIN:
        {
            LoRaWAN.join();
            break;
        }
        case DEVICE_STATE_SEND:
        {
            LoRaWAN.send();
            deviceState = DEVICE_STATE_CYCLE;
            break;
        }
        case DEVICE_STATE_CYCLE:
        {
            txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
            LoRaWAN.cycle(txDutyCycleTime);
            deviceState = DEVICE_STATE_SLEEP;
            break;
        }
        case DEVICE_STATE_SLEEP:
        {
            cycle_count++; // Increment cycle count before sleep
            LoRaWAN.sleep(loraWanClass);
            break;
        }
        default:
        {
            deviceState = DEVICE_STATE_INIT;
            break;
        }
    }
    
    ensureWiFiConnection();
    client.loop();
}
