#include "config.h"
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

#define BIND_PIN 6

// === ПИНЫ ДЛЯ КАНАЛОВ 1-4 ===
#define CH1_PIN 1
#define CH2_PIN 2
#define CH3_PIN 3
#define CH4_PIN 4

#define LED_PIN 8
#define LED_ON LOW
#define LED_OFF HIGH

// === НАСТРОЙКИ RC КАНАЛОВ ===
#define RC_CHANNELS 8
#define PULSE_MIN 1000
#define PULSE_MAX 2000
#define PULSE_CENTER 1500

// === МАКРОСЫ ДЛЯ ОТЛАДКИ ===
#ifdef DEBUG
  #define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTF(...)
  #define DEBUG_PRINTLN(...)
#endif

typedef struct {
    uint16_t channels[RC_CHANNELS];
    uint8_t connected;
    uint32_t seq;
} struct_message;

struct_message rxData;
bool isBindMode = false;
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

uint16_t currentCh1 = FAILSAFE_CH1;
uint16_t currentCh2 = FAILSAFE_CH2;
uint16_t currentCh3 = FAILSAFE_CH3;
uint16_t currentCh4 = FAILSAFE_CH4;

uint32_t packetCount = 0;
unsigned long lastPacketTime = 0;
unsigned long minDelta = 9999;
unsigned long maxDelta = 0;
unsigned long totalDelta = 0;
int deltaCount = 0;

bool failsafeActive = false;
unsigned long failsafeLedTime = 0;
bool failsafeLedState = false;

void writeServoPulse(Servo &servo, uint16_t pulse) {
    if (pulse < PULSE_MIN) pulse = PULSE_MIN;
    if (pulse > PULSE_MAX) pulse = PULSE_MAX;
    servo.writeMicroseconds(pulse);
}

#if ESP_IDF_VERSION_MAJOR >= 5
  void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
#else
  void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
    if (isBindMode) return;
    
    unsigned long now = millis();
    
    if (len != sizeof(struct_message)) {
        DEBUG_PRINTF("Wrong packet size: %d (expected %d)\n", len, sizeof(struct_message));
        return;
    }

    memcpy(&rxData, incomingData, sizeof(rxData));
    packetCount++;
    
    #ifdef DEBUG
      if (lastPacketTime > 0) {
          unsigned long delta = now - lastPacketTime;
          if (delta < minDelta) minDelta = delta;
          if (delta > maxDelta) maxDelta = delta;
          totalDelta += delta;
          deltaCount++;
          
          DEBUG_PRINTF("[%3lums] SEQ=%lu CH1=%4d CH2=%4d CH3=%4d CH4=%4d CH5=%4d CH6=%4d CH7=%4d CH8=%4d\n",
                        delta,
                        (unsigned long)rxData.seq,
                        rxData.channels[0], rxData.channels[1],
                        rxData.channels[2], rxData.channels[3],
                        rxData.channels[4], rxData.channels[5],
                        rxData.channels[6], rxData.channels[7]);
      } else {
          DEBUG_PRINTF("[FIRST] SEQ=%lu CH1=%4d CH2=%4d CH3=%4d CH4=%4d CH5=%4d CH6=%4d CH7=%4d CH8=%4d\n",
                        (unsigned long)rxData.seq,
                        rxData.channels[0], rxData.channels[1],
                        rxData.channels[2], rxData.channels[3],
                        rxData.channels[4], rxData.channels[5],
                        rxData.channels[6], rxData.channels[7]);
      }
    #endif
    
    lastPacketTime = now;

    if (failsafeActive) {
        failsafeActive = false;
        digitalWrite(LED_PIN, LED_ON);
        DEBUG_PRINTLN("Failsafe: connection restored");
    }

    if (rxData.connected) {
        if (rxData.channels[0] != currentCh1) {
            writeServoPulse(servo1, rxData.channels[0]);
            currentCh1 = rxData.channels[0];
        }
        
        if (rxData.channels[1] != currentCh2) {
            writeServoPulse(servo2, rxData.channels[1]);
            currentCh2 = rxData.channels[1];
        }
        
        if (rxData.channels[2] != currentCh3) {
            writeServoPulse(servo3, rxData.channels[2]);
            currentCh3 = rxData.channels[2];
        }
        
        if (rxData.channels[3] != currentCh4) {
            writeServoPulse(servo4, rxData.channels[3]);
            currentCh4 = rxData.channels[3];
        }
        
        digitalWrite(LED_PIN, LED_ON);
    } else {
        writeServoPulse(servo1, FAILSAFE_CH1);
        writeServoPulse(servo2, FAILSAFE_CH2);
        writeServoPulse(servo3, FAILSAFE_CH3);
        writeServoPulse(servo4, FAILSAFE_CH4);
        currentCh1 = FAILSAFE_CH1;
        currentCh2 = FAILSAFE_CH2;
        currentCh3 = FAILSAFE_CH3;
        currentCh4 = FAILSAFE_CH4;
        digitalWrite(LED_PIN, LED_OFF);
    }

    #ifdef DEBUG
      if (packetCount % 100 == 0 && deltaCount > 0) {
          float avgDelta = (float)totalDelta / deltaCount;
          DEBUG_PRINTF("=== STATS: packets=%lu, min=%lums, max=%lums, avg=%.1fms ===\n",
                        (unsigned long)packetCount, minDelta, maxDelta, avgDelta);
          minDelta = 9999;
          maxDelta = 0;
          totalDelta = 0;
          deltaCount = 0;
      }
    #endif
}

void setup() {
    Serial.begin(115200);
    delay(500);

    #ifdef DEBUG
      Serial.println("=== RECEIVER (4 OUTPUTS) ===");
      Serial.printf("Channels: %d, Pulse: %d-%d us\n", RC_CHANNELS, PULSE_MIN, PULSE_MAX);
      Serial.printf("Failsafe timeout: %d ms\n", FAILSAFE_TIMEOUT);
      Serial.printf("CH1: Pin %d\n", CH1_PIN);
      Serial.printf("CH2: Pin %d\n", CH2_PIN);
      Serial.printf("CH3: Pin %d\n", CH3_PIN);
      Serial.printf("CH4: Pin %d\n", CH4_PIN);
    #endif

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    pinMode(BIND_PIN, INPUT_PULLUP);
    delay(50);
    if (digitalRead(BIND_PIN) == LOW) {
        isBindMode = true;
        digitalWrite(LED_PIN, LED_ON);
        DEBUG_PRINTLN("BIND BEACON MODE ON");
    }

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo3.setPeriodHertz(50);
    servo4.setPeriodHertz(50);
    
    servo1.attach(CH1_PIN, PULSE_MIN, PULSE_MAX);
    servo2.attach(CH2_PIN, PULSE_MIN, PULSE_MAX);
    servo3.attach(CH3_PIN, PULSE_MIN, PULSE_MAX);
    servo4.attach(CH4_PIN, PULSE_MIN, PULSE_MAX);
    
    writeServoPulse(servo1, FAILSAFE_CH1);
    writeServoPulse(servo2, FAILSAFE_CH2);
    writeServoPulse(servo3, FAILSAFE_CH3);
    writeServoPulse(servo4, FAILSAFE_CH4);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    #ifdef DEBUG
      Serial.print("RX MAC: ");
      Serial.println(WiFi.macAddress());
    #endif

    if (esp_now_init() != ESP_OK) {
        DEBUG_PRINTLN("ESP-NOW INIT FAILED");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
    
    if (isBindMode) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, broadcastMac, 6);
        peerInfo.channel = 1;
        peerInfo.encrypt = false;
        esp_now_add_peer(&peerInfo);
    }

    #ifdef DEBUG
      Serial.println("ESP-NOW READY");
      Serial.println("========================================");
    #endif
}

void loop() {
    if (isBindMode) {
        static unsigned long lastBeacon = 0;
        if (millis() - lastBeacon > 300) {
            lastBeacon = millis();
            struct_message beaconMsg = {};
            beaconMsg.channels[0] = PULSE_CENTER;
            beaconMsg.channels[1] = PULSE_CENTER;
            beaconMsg.connected = 3;
            beaconMsg.seq = 0;
            esp_now_send(broadcastMac, (uint8_t *)&beaconMsg, sizeof(beaconMsg));
        }
        delay(10);
        return;
    }

    unsigned long now = millis();
    if (!failsafeActive && (now - lastPacketTime > FAILSAFE_TIMEOUT)) {
        failsafeActive = true;
        writeServoPulse(servo1, FAILSAFE_CH1);
        writeServoPulse(servo2, FAILSAFE_CH2);
        writeServoPulse(servo3, FAILSAFE_CH3);
        writeServoPulse(servo4, FAILSAFE_CH4);
        currentCh1 = FAILSAFE_CH1;
        currentCh2 = FAILSAFE_CH2;
        currentCh3 = FAILSAFE_CH3;
        currentCh4 = FAILSAFE_CH4;
        
        DEBUG_PRINTLN("!!! FAILSAFE ACTIVATED !!!");
        DEBUG_PRINTF("CH1=%d CH2=%d CH3=%d CH4=%d\n", 
                      FAILSAFE_CH1, FAILSAFE_CH2, FAILSAFE_CH3, FAILSAFE_CH4);
    }

    if (failsafeActive) {
        if (millis() - failsafeLedTime > 250) {
            failsafeLedTime = millis();
            failsafeLedState = !failsafeLedState;
            digitalWrite(LED_PIN, failsafeLedState ? LED_ON : LED_OFF);
        }
    }
    
    delay(10);
}