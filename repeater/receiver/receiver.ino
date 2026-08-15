#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

#define BIND_PIN 6
#define SERVO1_PIN 4
#define SERVO2_PIN 3
#define LED_PIN 8
#define LED_ON LOW
#define LED_OFF HIGH

#define FAILSAFE_TIMEOUT 500
#define FAILSAFE_ANGLE1 90
#define FAILSAFE_ANGLE2 90

typedef struct {
    uint8_t angle1;
    uint8_t angle2;
    uint8_t buttons;
    uint8_t connected;
    uint32_t seq;
} struct_message;

struct_message rxData;
bool isBindMode = false;
uint8_t broadcastMac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

Servo servo1;
Servo servo2;

int currentAngle1 = 90;
int currentAngle2 = 90;

uint32_t packetCount = 0;
unsigned long lastPacketTime = 0;
unsigned long minDelta = 9999;
unsigned long maxDelta = 0;
unsigned long totalDelta = 0;
int deltaCount = 0;

bool failsafeActive = false;
unsigned long failsafeLedTime = 0;
bool failsafeLedState = false;

#if ESP_IDF_VERSION_MAJOR >= 5
  void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
#else
  void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
    if (isBindMode) return;
    
    unsigned long now = millis();
    
    if (len != sizeof(struct_message)) {
        Serial.printf("Wrong packet size: %d\n", len);
        return;
    }

    memcpy(&rxData, incomingData, sizeof(rxData));
    packetCount++;
    
    // === ВЫЧИСЛЯЕМ ДЕЛЬТУ ДО ОБНОВЛЕНИЯ lastPacketTime ===
    if (lastPacketTime > 0) {
        unsigned long delta = now - lastPacketTime;
        if (delta < minDelta) minDelta = delta;
        if (delta > maxDelta) maxDelta = delta;
        totalDelta += delta;
        deltaCount++;
        
        Serial.printf("[%3lums] SEQ=%lu A1=%u A2=%u\n",
                      delta,
                      (unsigned long)rxData.seq,
                      rxData.angle1, rxData.angle2);
    } else {
        Serial.printf("[FIRST] SEQ=%lu A1=%u A2=%u\n",
                      (unsigned long)rxData.seq,
                      rxData.angle1, rxData.angle2);
    }
    
    // === ОБНОВЛЯЕМ lastPacketTime ПОСЛЕ ВЫЧИСЛЕНИЯ ДЕЛЬТЫ ===
    lastPacketTime = now;

    // Сбрасываем Failsafe
    if (failsafeActive) {
        failsafeActive = false;
        digitalWrite(LED_PIN, LED_ON);
        Serial.println("Failsafe: connection restored");
    }

    // Обновляем сервы
    if (rxData.connected) {
        if (rxData.angle1 != currentAngle1) {
            servo1.write(rxData.angle1);
            currentAngle1 = rxData.angle1;
        }
        
        if (rxData.angle2 != currentAngle2) {
            servo2.write(rxData.angle2);
            currentAngle2 = rxData.angle2;
        }
        
        digitalWrite(LED_PIN, LED_ON);
    } else {
        servo1.write(90);
        servo2.write(90);
        currentAngle1 = 90;
        currentAngle2 = 90;
        digitalWrite(LED_PIN, LED_OFF);
    }

    if (packetCount % 100 == 0 && deltaCount > 0) {
        float avgDelta = (float)totalDelta / deltaCount;
        Serial.printf("=== STATS: packets=%lu, min=%lums, max=%lums, avg=%.1fms ===\n",
                      (unsigned long)packetCount, minDelta, maxDelta, avgDelta);
        minDelta = 9999;
        maxDelta = 0;
        totalDelta = 0;
        deltaCount = 0;
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("=== RECEIVER ===");
    Serial.printf("Failsafe timeout: %d ms\n", FAILSAFE_TIMEOUT);
    Serial.printf("Failsafe angles: S1=%d, S2=%d\n", FAILSAFE_ANGLE1, FAILSAFE_ANGLE2);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    pinMode(BIND_PIN, INPUT_PULLUP);
    delay(50);
    if (digitalRead(BIND_PIN) == LOW) {
        isBindMode = true;
        digitalWrite(LED_PIN, LED_ON);
        Serial.println("BIND BEACON MODE ON");
    }

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo1.attach(SERVO1_PIN, 1000, 2000);
    servo2.attach(SERVO2_PIN, 1000, 2000);
    servo1.write(90);
    servo2.write(90);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.print("RX MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW INIT FAILED");
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

    Serial.println("ESP-NOW READY");
    Serial.println("========================================");
}

void loop() {
    if (isBindMode) {
        static unsigned long lastBeacon = 0;
        if (millis() - lastBeacon > 300) {
            lastBeacon = millis();
            struct_message beaconMsg = {90, 90, 0, 3, 0};
            esp_now_send(broadcastMac, (uint8_t *)&beaconMsg, sizeof(beaconMsg));
        }
        delay(10);
        return;
    }

    // Проверка Failsafe
    unsigned long now = millis();
    if (!failsafeActive && (now - lastPacketTime > FAILSAFE_TIMEOUT)) {
        failsafeActive = true;
        servo1.write(FAILSAFE_ANGLE1);
        servo2.write(FAILSAFE_ANGLE2);
        currentAngle1 = FAILSAFE_ANGLE1;
        currentAngle2 = FAILSAFE_ANGLE2;
        
        Serial.println("!!! FAILSAFE ACTIVATED !!!");
        Serial.printf("Servos set to: S1=%d, S2=%d\n", FAILSAFE_ANGLE1, FAILSAFE_ANGLE2);
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