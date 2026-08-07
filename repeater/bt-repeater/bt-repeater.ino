#include <Bluepad32.h>
#include <esp_now.h>
#include <WiFi.h>

#define LED_PIN 22
#define LED_ON LOW
#define LED_OFF HIGH

// === КАЛИБРОВОЧНЫЕ ЗНАЧЕНИЯ ===
#define CAL_CENTER_X 0
#define CAL_CENTER_Y 0
#define CAL_MIN_X -428
#define CAL_MAX_X 440
#define CAL_MIN_Y -436
#define CAL_MAX_Y 452
// =================================

// MAC-адрес приемника (ESP32C3 Pro Mini)
uint8_t receiverMac[] = {0xE8, 0x3D, 0xC1, 0x9F, 0x19, 0xC0};

// === МИНИМАЛЬНАЯ СТРУКТУРА (4 БАЙТА!) ===
typedef struct struct_message {
    uint8_t angle1;    // 0-180
    uint8_t angle2;    // 0-180
    uint8_t buttons;   // 8 кнопок (битовая маска)
    uint8_t connected; // 0 или 1
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Мертвая зона для центра
const int DEAD_ZONE = 10;

// Функция для преобразования значения оси в угол
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

// Callback при подключении контроллера
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", 
                         ctl->getModelName().c_str(), 
                         properties.vendor_id,
                         properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            
            digitalWrite(LED_PIN, LED_ON);
            
            myData.connected = 1;
            myData.angle1 = 90;
            myData.angle2 = 90;
            myData.buttons = 0;
            esp_now_send(receiverMac, (uint8_t *) &myData, sizeof(myData));
            
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

// Callback при отключении контроллера
void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            
            bool anyControllerConnected = false;
            for (int j = 0; j < BP32_MAX_GAMEPADS; j++) {
                if (myControllers[j] != nullptr) {
                    anyControllerConnected = true;
                    break;
                }
            }
            
            if (!anyControllerConnected) {
                digitalWrite(LED_PIN, LED_OFF);
                
                myData.connected = 0;
                esp_now_send(receiverMac, (uint8_t *) &myData, sizeof(myData));
            }
            
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void processGamepad(ControllerPtr ctl) {
    int axisX = ctl->axisX();
    int axisY = ctl->axisY();
    
    myData.angle1 = (uint8_t)mapAxisToAngle(axisX, CAL_CENTER_X, CAL_MIN_X, CAL_MAX_X);
    myData.angle2 = (uint8_t)mapAxisToAngle(axisY, CAL_CENTER_Y, CAL_MIN_Y, CAL_MAX_Y);
    myData.buttons = (uint8_t)(ctl->buttons() & 0xFF);
    myData.connected = 1;
    
    esp_now_send(receiverMac, (uint8_t *) &myData, sizeof(myData));
    
    Serial.printf("Sent: A1=%d, A2=%d, B=0x%02X\n", 
                  myData.angle1, myData.angle2, myData.buttons);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    
    esp_log_level_set("*", ESP_LOG_NONE);
    
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    Serial.printf("My MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  WiFi.macAddress()[0], WiFi.macAddress()[1], WiFi.macAddress()[2],
                  WiFi.macAddress()[3], WiFi.macAddress()[4], WiFi.macAddress()[5]);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 1;  
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

void loop() {
    bool dataUpdated = BP32.update();
    if (dataUpdated)
        processControllers();
    delay(20);
}