#include <Bluepad32.h>
#include <WiFi.h>
#include <esp_now.h>

#define LED_PIN 22
#define LED_ON LOW
#define LED_OFF HIGH

// Калибровочные значения
#define CAL_CENTER_X 0
#define CAL_CENTER_Y 0
#define CAL_MIN_X -428
#define CAL_MAX_X 440
#define CAL_MIN_Y -436
#define CAL_MAX_Y 452

uint8_t receiverMac[] = {0xE8, 0x3D, 0xC1, 0x9F, 0x19, 0xC0};

typedef struct {
    uint8_t angle1;
    uint8_t angle2;
    uint8_t buttons;
    uint8_t connected;
    uint32_t seq;
} struct_message;

struct_message txData;
esp_now_peer_info_t peerInfo;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

volatile bool sendFinished = true;
uint32_t sequence = 0;

// Мертвая зона
const int DEAD_ZONE = 10;

int mapAxisToAngle(int axisValue, int center, int minVal, int maxVal) {
    if (abs(axisValue - center) < DEAD_ZONE) {
        return 90;
    }
    
    int normalized;
    if (axisValue < center) {
        normalized = map(axisValue, minVal, center, 0, 90);
    } else {
        normalized = map(axisValue, center, maxVal, 90, 180);
    }
    
    if (normalized < 0) normalized = 0;
    if (normalized > 180) normalized = 180;
    
    return normalized;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    sendFinished = true;
}

void onConnectedController(ControllerPtr ctl) {
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

    int axisX = ctl->axisX();
    int axisY = ctl->axisY();
    
    txData.angle1 = (uint8_t)mapAxisToAngle(axisX, CAL_CENTER_X, CAL_MIN_X, CAL_MAX_X);
    txData.angle2 = (uint8_t)mapAxisToAngle(axisY, CAL_CENTER_Y, CAL_MIN_Y, CAL_MAX_Y);
    txData.buttons = (uint8_t)(ctl->buttons() & 0xFF);
    txData.connected = 1;
    txData.seq = sequence++;

    sendFinished = false;
    esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&txData, sizeof(txData));
    if (result != ESP_OK) {
        sendFinished = true;
    }

    // Отладка (каждый 10-й пакет)
    if (sequence % 10 == 0) {
        Serial.printf("Sent: A1=%d, A2=%d, B=0x%02X\n", 
                      txData.angle1, txData.angle2, txData.buttons);
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

    Serial.println("=== TRANSMITTER WITH BLUEPAD32 ===");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    // ESP-NOW инициализация
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

    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("FAILED TO ADD PEER");
        return;
    }

    // Bluepad32 инициализация
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);

    Serial.println("READY");
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }
    delay(5);
}