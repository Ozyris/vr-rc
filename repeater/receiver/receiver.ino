#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#define SERVO1_PIN 4
#define SERVO2_PIN 3

#define LED_PIN 8  // Встроенный LED на ESP32C3
#define LED_ON  LOW
#define LED_OFF HIGH

// === СТРУКТУРА (4 БАЙТА) ===
typedef struct struct_message {
    uint8_t angle1;    // 0-180
    uint8_t angle2;    // 0-180
    uint8_t buttons;   // 8 кнопок (битовая маска)
    uint8_t connected; // 0 или 1
} struct_message;

struct_message receivedData;

// Создаем объекты серв
Servo servo1;
Servo servo2;

// Текущие углы серв
int currentAngle1 = 90;
int currentAngle2 = 90;

// MAC-адрес ретранслятора (Lolin32 Lite)
uint8_t transmitterMac[] = {0x7C, 0x9E, 0xBD, 0xED, 0x7B, 0xD0};

unsigned long lastReceiveTime = 0;
const unsigned long TIMEOUT = 500; // Таймаут 500мс

// === УНИВЕРСАЛЬНЫЙ CALLBACK ДЛЯ ОБЕИХ ВЕРСИЙ ===
#if ESP_IDF_VERSION_MAJOR >= 5
  // Для ESP-IDF 5.x (новая сигнатура)
  void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
      const uint8_t *mac = info->src_addr;
#else
  // Для ESP-IDF 4.x (старая сигнатура)
  void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
      // Проверяем, что данные пришли от нашего ретранслятора
      if (memcmp(mac, transmitterMac, 6) != 0) {
          return; // Игнорируем данные от других устройств
      }
      
      memcpy(&receivedData, incomingData, sizeof(receivedData));
      lastReceiveTime = millis();
      
      // Обновляем сервы только если данные получены
      if (receivedData.connected) {
          if (receivedData.angle1 != currentAngle1) {
              servo1.write(receivedData.angle1);
              currentAngle1 = receivedData.angle1;
          }
          
          if (receivedData.angle2 != currentAngle2) {
              servo2.write(receivedData.angle2);
              currentAngle2 = receivedData.angle2;
          }
          
          // Индикация приема данных
          digitalWrite(LED_PIN, LED_ON);
          
          // Отладка
          Serial.printf("Recv: A1=%d, A2=%d, B=0x%02X\n", 
                        receivedData.angle1, receivedData.angle2, 
                        receivedData.buttons);
      } else {
          // Если контроллер отключен - центрируем сервы
          servo1.write(90);
          servo2.write(90);
          currentAngle1 = 90;
          currentAngle2 = 90;
          digitalWrite(LED_PIN, LED_OFF);
          Serial.println("Controller disconnected - centering servos");
      }
  }

void setup() {
    Serial.begin(115200);
    
    esp_log_level_set("*", ESP_LOG_NONE);
    
    Serial.println("ESP32C3 Receiver started");
    Serial.printf("My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  WiFi.macAddress()[0], WiFi.macAddress()[1], WiFi.macAddress()[2],
                  WiFi.macAddress()[3], WiFi.macAddress()[4], WiFi.macAddress()[5]);

    // Настройка пина LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    // Подключение сервомашинок
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    servo1.setPeriodHertz(50);
    servo2.setPeriodHertz(50);
    servo1.attach(SERVO1_PIN, 1000, 2000);
    servo2.attach(SERVO2_PIN, 1000, 2000);
    
    // Начальное положение - центр
    servo1.write(90);
    servo2.write(90);

    // Инициализация WiFi и ESP-NOW
    WiFi.mode(WIFI_STA);
    // WiFi.setSleep(false); // Убрал, так как вызывает перезагрузку
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // Регистрируем callback (работает с обеими версиями)
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    // Проверка таймаута
    if (millis() - lastReceiveTime > TIMEOUT) {
        // Если данные не приходят - выключаем LED и центрируем сервы
        digitalWrite(LED_PIN, LED_OFF);
        // Не центрируем постоянно, чтобы не дергать сервы
        static bool wasTimeout = false;
        if (!wasTimeout) {
            servo1.write(90);
            servo2.write(90);
            currentAngle1 = 90;
            currentAngle2 = 90;
            wasTimeout = true;
            Serial.println("Timeout - centering servos");
        }
    } else {
        // Если данные есть - сбрасываем флаг таймаута
        static bool wasTimeout = false;
        if (wasTimeout) {
            wasTimeout = false;
        }
    }
    
    delay(20);
}