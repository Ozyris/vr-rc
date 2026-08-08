#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

#define SERVO1_PIN 4
#define SERVO2_PIN 3
#define LED_PIN 8
#define LED_ON LOW
#define LED_OFF HIGH

typedef struct {
    uint8_t angle1;
    uint8_t angle2;
    uint8_t buttons;
    uint8_t connected;
    uint32_t seq;
} struct_message;

struct_message rxData;

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

// Универсальный callback для обеих версий ESP-IDF
#if ESP_IDF_VERSION_MAJOR >= 5
  void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
#else
  void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
    unsigned long now = millis();
    
    if (len != sizeof(struct_message)) {
        Serial.printf("Wrong packet size: %d\n", len);
        return;
    }

    memcpy(&rxData, incomingData, sizeof(rxData));
    packetCount++;

    // === ДИАГНОСТИКА: интервал между пакетами ===
    if (lastPacketTime > 0) {
        unsigned long delta = now - lastPacketTime;
        
        // Обновляем статистику
        if (delta < minDelta) minDelta = delta;
        if (delta > maxDelta) maxDelta = delta;
        totalDelta += delta;
        deltaCount++;
        
        // Выводим каждый пакет с интервалом
        Serial.printf("[%3lums] SEQ=%lu A1=%u A2=%u\n",
                      delta,
                      (unsigned long)rxData.seq,
                      rxData.angle1, rxData.angle2);
    } else {
        // Первый пакет
        Serial.printf("[FIRST] SEQ=%lu A1=%u A2=%u\n",
                      (unsigned long)rxData.seq,
                      rxData.angle1, rxData.angle2);
    }
    lastPacketTime = now;

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

    // Каждые 100 пакетов - статистика
    if (packetCount % 100 == 0 && deltaCount > 0) {
        float avgDelta = (float)totalDelta / deltaCount;
        Serial.printf("=== STATS: packets=%lu, min=%lums, max=%lums, avg=%.1fms ===\n",
                      (unsigned long)packetCount, minDelta, maxDelta, avgDelta);
        
        // Сбрасываем статистику для следующего периода
        minDelta = 9999;
        maxDelta = 0;
        totalDelta = 0;
        deltaCount = 0;
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("=== RECEIVER WITH INTERVAL DIAGNOSTICS ===");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

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
    Serial.println("ESP-NOW READY");
    Serial.println("========================================");
}

void loop() {
    delay(10);
}