#include <Bluepad32.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h> 

#define BIND_PIN 13       // пин для входа в режим бинда
#define LED_PIN 22
#define LED_ON LOW
#define LED_OFF HIGH

// === НАСТРОЙКИ RC КАНАЛОВ ===
#define RC_CHANNELS 6
#define PULSE_MIN 1000    // Минимальный импульс (мкс)
#define PULSE_MAX 2000    // Максимальный импульс (мкс)
#define PULSE_CENTER 1500 // Центральное положение (мкс)

// === КАЛИБРОВКА СТИКОВ ===
#define CAL_CENTER_X 0
#define CAL_CENTER_Y 0
#define CAL_MIN_X -428
#define CAL_MAX_X 440
#define CAL_MIN_Y -436
#define CAL_MAX_Y 452

uint8_t receiverMac[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool isBindMode = false;     
bool bindSuccess = false;   
Preferences preferences;   

// === НОВАЯ СТРУКТУРА С RC КАНАЛАМИ ===
typedef struct {
    uint16_t channels[RC_CHANNELS]; // 6 каналов (12 байт)
    uint8_t connected;              // 1 байт
    uint32_t seq;                   // 4 байта
} struct_message;                   // ИТОГО: 17 байт

struct_message txData;
esp_now_peer_info_t peerInfo;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

volatile bool sendFinished = true;
uint32_t sequence = 0;

const int DEAD_ZONE = 10;

// === ФУНКЦИЯ: ось (-512..512) -> импульс (1000..2000) ===
uint16_t mapAxisToPulse(int axisValue, int center, int minVal, int maxVal) {
    // Применяем мертвую зону
    if (abs(axisValue - center) < DEAD_ZONE) {
        return PULSE_CENTER;
    }
    
    // Преобразуем в импульс
    int pulse;
    if (axisValue < center) {
        pulse = map(axisValue, minVal, center, PULSE_MIN, PULSE_CENTER);
    } else {
        pulse = map(axisValue, center, maxVal, PULSE_CENTER, PULSE_MAX);
    }
    
    // Ограничиваем
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

        Serial.print("BIND SUCCESS! RX MAC saved: ");
        for(int i=0; i<6; i++) Serial.printf("%02X%s", receiverMac[i], (i<5)?":":"");
        Serial.println();

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
            Serial.printf("Controller connected, index=%d\n", i);
            myControllers[i] = ctl;
            foundEmptySlot = true;
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
                digitalWrite(LED_PIN, LED_OFF);
                txData.connected = 0;
                esp_now_send(receiverMac, (uint8_t *)&txData, sizeof(txData));
            }
            break;
        }
    }
}

void processGamepad(ControllerPtr ctl) {
    if (!sendFinished) return;

    // === ЧИТАЕМ ОСИ ===
    int axisX = ctl->axisX();   // -512..512 (левый стик X)
    int axisY = ctl->axisY();   // -512..512 (левый стик Y)
    int axisRX = ctl->axisRX(); // -512..512 (правый стик X)
    int axisRY = ctl->axisRY(); // -512..512 (правый стик Y)
    
    // === ЗАПОЛНЯЕМ КАНАЛЫ ===
    txData.channels[0] = mapAxisToPulse(axisX, CAL_CENTER_X, CAL_MIN_X, CAL_MAX_X);   // Aileron (левый стик X)
    txData.channels[1] = mapAxisToPulse(axisY, CAL_CENTER_Y, CAL_MIN_Y, CAL_MAX_Y);   // Elevator (левый стик Y)
    txData.channels[2] = mapAxisToPulse(axisRY, 0, -512, 512);                        // Throttle (правый стик Y)
    txData.channels[3] = mapAxisToPulse(axisRX, 0, -512, 512);                        // Rudder (правый стик X)
    
    // === КНОПКИ → КАНАЛЫ 5-6 ===
    uint16_t buttons = ctl->buttons();
    txData.channels[4] = (buttons & 0x01) ? PULSE_MAX : PULSE_MIN;  // Кнопка A
    txData.channels[5] = (buttons & 0x02) ? PULSE_MAX : PULSE_MIN;  // Кнопка B
    
    txData.connected = 1;
    txData.seq = sequence++;

    sendFinished = false;
    esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&txData, sizeof(txData));
    if (result != ESP_OK) {
        sendFinished = true;
    }

    if (sequence % 10 == 0) {
        Serial.printf("Sent: CH1=%4d CH2=%4d CH3=%4d CH4=%4d CH5=%4d CH6=%4d\n",
                      txData.channels[0], txData.channels[1], 
                      txData.channels[2], txData.channels[3],
                      txData.channels[4], txData.channels[5]);
    }
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

    Serial.println("=== TRANSMITTER (RC CHANNELS) ===");
    Serial.printf("Channels: %d, Pulse: %d-%d us\n", RC_CHANNELS, PULSE_MIN, PULSE_MAX);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    pinMode(BIND_PIN, INPUT_PULLUP);
    delay(50);
    if (digitalRead(BIND_PIN) == LOW) {
        isBindMode = true;
        digitalWrite(LED_PIN, LED_ON);
        Serial.println("BIND MODE ON (Listening)");
    } else {
        preferences.begin("rx_conf", true);
        if (preferences.isKey("mac")) {
            preferences.getBytes("mac", receiverMac, 6);
        }
        preferences.end();
    }

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    Serial.print("TX MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW INIT FAILED");
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
        Serial.println("FAILED TO ADD PEER");
        return;
    }

    if (!isBindMode) {
        BP32.setup(&onConnectedController, &onDisconnectedController);
        BP32.forgetBluetoothKeys();
        BP32.enableVirtualDevice(false);
    }

    Serial.println("READY");
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
    delay(5);
}