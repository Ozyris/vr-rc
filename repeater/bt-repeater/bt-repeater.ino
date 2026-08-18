#include "config.h"
#include <Bluepad32.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h> 

#define BIND_PIN 13
#define LED_PIN 22
#define LED_ON LOW
#define LED_OFF HIGH

// === НАСТРОЙКИ RC КАНАЛОВ ===
#define RC_CHANNELS 8

// === НАСТРОЙКИ ТАЙМАУТА КОНТРОЛЛЕРА ===
#define CONTROLLER_TIMEOUT 2000  // 2 секунды без данных от контроллера

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

uint8_t receiverMac[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool isBindMode = false;     
bool bindSuccess = false;   
Preferences preferences;   

typedef struct {
    uint16_t channels[RC_CHANNELS];
    uint8_t connected;
    uint32_t seq;
} struct_message;

struct_message txData;
esp_now_peer_info_t peerInfo;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

volatile bool sendFinished = true;
uint32_t sequence = 0;

const int DEAD_ZONE = 10;

// === ПЕРЕМЕННЫЕ ДЛЯ КОНТРОЛЛЯ ПОДКЛЮЧЕНИЯ ===
bool controllerReallyConnected = false;
unsigned long lastValidDataTime = 0;

// === ПЕРЕМЕННЫЕ ДЛЯ ГАЗА (VRBOX) ===
#ifdef VRBOX
  uint16_t currentThrottle = THROTTLE_DEFAULT;
  unsigned long lastThrottleChange = 0;
  const unsigned long THROTTLE_DEBOUNCE = 100;
#endif

uint16_t mapAxisToPulse(int axisValue, int center, int minVal, int maxVal) {
    if (abs(axisValue - center) < DEAD_ZONE) {
        return PULSE_CENTER;
    }
    
    int pulse;
    if (axisValue < center) {
        pulse = map(axisValue, minVal, center, PULSE_MIN, PULSE_CENTER);
    } else {
        pulse = map(axisValue, center, maxVal, PULSE_CENTER, PULSE_MAX);
    }
    
    if (pulse < PULSE_MIN) pulse = PULSE_MIN;
    if (pulse > PULSE_MAX) pulse = PULSE_MAX;
    
    return (uint16_t)pulse;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    sendFinished = true;
}

#if ESP_IDF_VERSION_MAJOR >= 5
  void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
#else
  void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
#endif
    if (!isBindMode || bindSuccess) return;
    if (len != sizeof(struct_message)) return;
    
    struct_message beacon;
    memcpy(&beacon, incomingData, sizeof(beacon));

    if (beacon.connected == 3) {
        #if ESP_IDF_VERSION_MAJOR >= 5
          memcpy(receiverMac, info->src_addr, 6);
        #else
          memcpy(receiverMac, mac, 6);
        #endif

        DEBUG_PRINT("BIND SUCCESS! RX MAC saved: ");
        for(int i=0; i<6; i++) DEBUG_PRINTF("%02X%s", receiverMac[i], (i<5)?":":"");
        DEBUG_PRINTLN();

        preferences.begin("rx_conf", false);
        preferences.putBytes("mac", receiverMac, 6);
        preferences.end();

        bindSuccess = true;
    }
}

void onConnectedController(ControllerPtr ctl) {
    if (isBindMode) return;
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            DEBUG_PRINTF("Controller connected, index=%d\n", i);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            controllerReallyConnected = true;
            lastValidDataTime = millis();
            digitalWrite(LED_PIN, LED_ON);
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (isBindMode) return;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            bool anyConnected = false;
            for (int j = 0; j < BP32_MAX_GAMEPADS; j++) {
                if (myControllers[j] != nullptr) anyConnected = true;
            }
            if (!anyConnected) {
                controllerReallyConnected = false;
                digitalWrite(LED_PIN, LED_OFF);
                txData.connected = 0;
                esp_now_send(receiverMac, (uint8_t *)&txData, sizeof(txData));
            }
            break;
        }
    }
}

bool isValidData(ControllerPtr ctl) {
    int axisX = ctl->axisX();
    int axisY = ctl->axisY();
    int axisRX = ctl->axisRX();
    int axisRY = ctl->axisRY();
    
    // Проверяем диапазон осей
    if (axisX < -512 || axisX > 512) return false;
    if (axisY < -512 || axisY > 512) return false;
    if (axisRX < -512 || axisRX > 512) return false;
    if (axisRY < -512 || axisRY > 512) return false;
    
    return true;
}

void processGamepad(ControllerPtr ctl) {
    if (!sendFinished) return;
    
    // === ПРОВЕРКА: контроллер реально подключен ===
    if (!controllerReallyConnected) {
        DEBUG_PRINTLN("Controller not connected - skipping send");
        return;
    }
    
    // === ПРОВЕРКА: валидность данных ===
    if (!isValidData(ctl)) {
        DEBUG_PRINTLN("Invalid data from controller - skipping");
        return;
    }
    
    // Обновляем время последних валидных данных
    lastValidDataTime = millis();

    int axisX = ctl->axisX();
    int axisY = ctl->axisY();
    int axisRX = ctl->axisRX();
    int axisRY = ctl->axisRY();
    uint16_t buttons = ctl->buttons();
    
    // === ЗАПОЛНЯЕМ ИСХОДНЫЕ ДАННЫЕ ===
    txData.channels[0] = mapAxisToPulse(axisX, CAL_CENTER_X, CAL_MIN_X, CAL_MAX_X);
    txData.channels[1] = mapAxisToPulse(axisY, CAL_CENTER_Y, CAL_MIN_Y, CAL_MAX_Y);
    txData.channels[3] = mapAxisToPulse(axisRX, 0, -512, 512);
    
    #ifdef VRBOX
      unsigned long now = millis();
      
      if ((buttons & 0x0020) && (now - lastThrottleChange > THROTTLE_DEBOUNCE)) {
          lastThrottleChange = now;
          currentThrottle += THROTTLE_STEP;
          if (currentThrottle > THROTTLE_MAX) currentThrottle = THROTTLE_MAX;
          DEBUG_PRINTF("Throttle UP: %d\n", currentThrottle);
      }
      
      if ((buttons & 0x0010) && (now - lastThrottleChange > THROTTLE_DEBOUNCE)) {
          lastThrottleChange = now;
          currentThrottle -= THROTTLE_STEP;
          if (currentThrottle < THROTTLE_MIN) currentThrottle = THROTTLE_MIN;
          DEBUG_PRINTF("Throttle DOWN: %d\n", currentThrottle);
      }
      
      txData.channels[2] = currentThrottle;
      
    #else
      txData.channels[2] = mapAxisToPulse(axisRY, 0, -512, 512);
    #endif
    
    txData.channels[4] = (buttons & 0x0001) ? PULSE_MAX : PULSE_MIN;
    txData.channels[5] = (buttons & 0x0002) ? PULSE_MAX : PULSE_MIN;
    txData.channels[6] = (buttons & 0x0004) ? PULSE_MAX : PULSE_MIN;
    txData.channels[7] = (buttons & 0x0008) ? PULSE_MAX : PULSE_MIN;
    
    // === ПРИМЕНЯЕМ МИКШЕР ===
    MixerData input;
    MixerData output;
    
    for (int i = 0; i < RC_CHANNELS; i++) {
        input.channels[i] = txData.channels[i];
    }
    
    applyMixer(&input, &output);
    
    for (int i = 0; i < RC_CHANNELS; i++) {
        txData.channels[i] = output.channels[i];
    }
    
    #ifdef DEBUG
      printMixerInfo(&output);
    #endif
    
    txData.connected = 1;
    txData.seq = sequence++;

    sendFinished = false;
    esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&txData, sizeof(txData));
    if (result != ESP_OK) {
        sendFinished = true;
    }

    // #ifdef DEBUG
    //   if (sequence % 10 == 0) {
    //     DEBUG_PRINTF("Sent: CH1=%4d CH2=%4d CH3=%4d CH4=%4d CH5=%4d CH6=%4d CH7=%4d CH8=%4d\n",
    //                   txData.channels[0], txData.channels[1], 
    //                   txData.channels[2], txData.channels[3],
    //                   txData.channels[4], txData.channels[5],
    //                   txData.channels[6], txData.channels[7]);
    //   }
    // #endif
}

void processControllers() {
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            if (ctl->isGamepad()) {
                processGamepad(ctl);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);

    #ifdef DEBUG
      Serial.println("=== TRANSMITTER (8 CHANNELS) ===");
      #ifdef VRBOX
        Serial.println("Mode: VRBOX (Throttle with buttons)");
      #else
        Serial.println("Mode: GAMEPAD (Throttle on right stick)");
      #endif
      Serial.printf("Channels: %d, Pulse: %d-%d us\n", RC_CHANNELS, PULSE_MIN, PULSE_MAX);
      Serial.printf("Controller timeout: %d ms\n", CONTROLLER_TIMEOUT);
    #endif

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    pinMode(BIND_PIN, INPUT_PULLUP);
    delay(50);
    if (digitalRead(BIND_PIN) == LOW) {
        isBindMode = true;
        digitalWrite(LED_PIN, LED_ON);
        DEBUG_PRINTLN("BIND MODE ON (Listening)");
    } else {
        preferences.begin("rx_conf", true);
        if (preferences.isKey("mac")) {
            preferences.getBytes("mac", receiverMac, 6);
            #ifdef DEBUG
              DEBUG_PRINT("Loaded MAC from NVS: ");
              for(int i=0; i<6; i++) DEBUG_PRINTF("%02X%s", receiverMac[i], (i<5)?":":"");
              DEBUG_PRINTLN();
            #endif
        } else {
          #ifdef DEBUG
            DEBUG_PRINTLN("No MAC in NVS, waiting for bind");
          #endif
        }
        preferences.end();
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    #ifdef DEBUG
      Serial.print("TX MAC: ");
      Serial.println(WiFi.macAddress());
    #endif

    if (esp_now_init() != ESP_OK) {
        DEBUG_PRINTLN("ESP-NOW INIT FAILED");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    if (isBindMode) {
        esp_now_register_recv_cb(OnDataRecv);
    }

    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        DEBUG_PRINTLN("FAILED TO ADD PEER");
        return;
    }

    if (!isBindMode) {
        BP32.setup(&onConnectedController, &onDisconnectedController);
        BP32.forgetBluetoothKeys();
        BP32.enableVirtualDevice(false);
    }

    #ifdef DEBUG
      Serial.println("READY");
    #endif
}

void loop() {
    if (isBindMode) {
        if (bindSuccess) {
            digitalWrite(LED_PIN, (millis() / 200) % 2);
        }
        return;
    }

    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }
    
    // === ТАЙМАУТ КОНТРОЛЛЕРА ===
    // Если контроллер был подключен, но давно нет данных - отправляем сигнал отключения
    if (controllerReallyConnected && (millis() - lastValidDataTime > CONTROLLER_TIMEOUT)) {
        controllerReallyConnected = false;
        digitalWrite(LED_PIN, LED_OFF);
        txData.connected = 0;
        esp_now_send(receiverMac, (uint8_t *)&txData, sizeof(txData));
        DEBUG_PRINTLN("Controller timeout - disconnecting");
    }
    
    delay(5);
}