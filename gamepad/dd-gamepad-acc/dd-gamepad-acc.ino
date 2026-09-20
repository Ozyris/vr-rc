#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <Adafruit_TinyUSB.h>

#define LED_PIN 8

// --- USB HID Дескриптор (Геймпад: 4 оси + 16 кнопок) ---
uint8_t const desc_hid_report[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x05,        // Usage (Game Pad)
  0xA1, 0x01,        // Collection (Application)

  // 16 кнопок
  0x05, 0x09,        //   Usage Page (Button)
  0x19, 0x01,        //   Usage Minimum (1)
  0x29, 0x10,        //   Usage Maximum (16)
  0x15, 0x00,        //   Logical Minimum (0)
  0x25, 0x01,        //   Logical Maximum (1)
  0x95, 0x10,        //   Report Count (16)
  0x75, 0x01,        //   Report Size (1)
  0x81, 0x02,        //   Input (Data,Var,Abs)

  // Оси X, Y, Z, Rz (по 8 бит)
  0x05, 0x01,        //   Usage Page (Generic Desktop)
  0x09, 0x30,        //   Usage (X)
  0x09, 0x31,        //   Usage (Y)
  0x09, 0x32,        //   Usage (Z)
  0x09, 0x35,        //   Usage (Rz)
  0x15, 0x81,        //   Logical Minimum (-127)
  0x25, 0x7F,        //   Logical Maximum (127)
  0x75, 0x08,        //   Report Size (8)
  0x95, 0x04,        //   Report Count (4)
  0x81, 0x02,        //   Input (Data,Var,Abs)

  0xC0               // End Collection
};

Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_NONE, 2, false);

// Структура отчёта (должна соответствовать дескриптору)
typedef struct __attribute__((packed)) {
    uint16_t buttons;
    int8_t x;
    int8_t y;
    int8_t z;
    int8_t rz;
} GamepadReport;

// --- BLE UUID ---
static BLEUUID SERVICE_UUID("0000fe55-0000-1000-8000-00805f9b34fb");
static BLEUUID CHAR_UUID("00000001-1000-1000-8000-00805f9b34fb");
static BLEUUID CCCD_UUID("00002902-0000-1000-8000-00805f9b34fb");

enum ThrottleState { STATE_LOCKED, STATE_ACTIVE, STATE_HOLD };
ThrottleState cruise_state = STATE_LOCKED;
uint16_t saved_throttle = 1000;
bool prev_click = false;

bool connected = false;
BLEClient* client = nullptr;
BLEAdvertisedDevice* targetDevice = nullptr;

int16_t last_accel_x = 0;
int16_t last_accel_y = 0;

int16_t signExtend13(int16_t value) {
    if (value & 0x1000) value |= 0xE000;
    return value;
}

void parsePacket(const uint8_t* data, size_t len) {
    if (len < 20 || data == nullptr) return;

    uint8_t btns = data[18];
    bool btn_click = (btns & 0x01) != 0;
    bool btn_app   = (btns & 0x04) != 0;

    uint8_t trackpad_x = ((data[16] & 0x1F) << 3) | ((data[17] & 0xE0) >> 5);
    uint8_t trackpad_y = ((data[17] & 0x1F) << 3) | ((data[18] & 0xE0) >> 5);

    int16_t acc_x = signExtend13(((data[6] & 0x07) << 10) | (data[7] << 2) | ((data[8] & 0xC0) >> 6));
    int16_t acc_y = signExtend13(((data[9] & 0x01) << 12) | (data[10] << 4) | ((data[11] & 0xF0) >> 4));

    if (acc_x == 0 && acc_y == 0) {
        acc_x = last_accel_x;
        acc_y = last_accel_y;
    } else {
        last_accel_x = acc_x;
        last_accel_y = acc_y;
    }

    // --- Круиз-контроль ---
    if (!btn_click && prev_click) {
        if (cruise_state == STATE_LOCKED) cruise_state = STATE_ACTIVE;
        else if (cruise_state == STATE_ACTIVE) {
            cruise_state = STATE_HOLD;
            saved_throttle = (trackpad_x > 0 || trackpad_y > 0) ? map(trackpad_y, 255, 0, 1000, 2000) : 1000;
        }
        else if (cruise_state == STATE_HOLD) cruise_state = STATE_ACTIVE;
    }
    prev_click = btn_click;

    // --- Подготовка RC-каналов (как в оригинале) ---
    uint16_t rc_roll  = constrain(map(acc_x, -500, 500, 1000, 2000), 1000, 2000);
    uint16_t rc_pitch = constrain(map(acc_y, -500, 500, 1000, 2000), 1000, 2000);
    uint16_t rc_yaw, rc_throttle;

    if (trackpad_x > 0 || trackpad_y > 0) {
        rc_yaw = map(trackpad_x, 0, 255, 1000, 2000);
        if (cruise_state == STATE_ACTIVE) rc_throttle = map(trackpad_y, 255, 0, 1000, 2000);
        else if (cruise_state == STATE_HOLD) rc_throttle = saved_throttle;
        else rc_throttle = 1000;
    } else {
        rc_yaw = 1500;
        rc_throttle = (cruise_state == STATE_HOLD) ? saved_throttle : 1000;
    }

    uint8_t rc_arm  = (cruise_state == STATE_HOLD) ? 1 : 0;
    uint8_t rc_mode = btn_app ? 1 : 0;

    // --- ОТПРАВКА В USB HID ---
    GamepadReport report;
    
    // Маппинг RC (1000-2000) в HID оси (-127..127)
    report.x  = map(rc_roll, 1000, 2000, -127, 127);
    report.y  = map(rc_pitch, 1000, 2000, -127, 127);
    report.z  = map(rc_yaw, 1000, 2000, -127, 127);
    report.rz = map(rc_throttle, 1000, 2000, -127, 127);

    // Кнопки
    report.buttons = 0;
    if (rc_arm)  report.buttons |= (1 << 0);  // Кнопка 1 (Arm)
    if (rc_mode) report.buttons |= (1 << 1);  // Кнопка 2 (Mode)
    if (btn_click) report.buttons |= (1 << 2); // Кнопка 3 (Click)
    if (btn_app)   report.buttons |= (1 << 3); // Кнопка 4 (App)

    // Отправляем на ПК
    if (TinyUSBDevice.mounted()) {
        usb_hid.sendReport(0, &report, sizeof(report));
    }

    // Вывод в терминал для отладки
    static unsigned long lp = 0;
    if (millis() - lp > 50) {
        lp = millis();
        const char* mode_str = (cruise_state == STATE_LOCKED) ? "LOCKED" : ((cruise_state == STATE_ACTIVE) ? "ACTIVE" : "HOLD  ");
        Serial.printf("[USB HID] MODE:%s | X:%4d Y:%4d Z:%4d RZ:%4d BTN:0x%04X\n", 
                      mode_str, report.x, report.y, report.z, report.rz, report.buttons);
    }
}

// --- BLE Callbacks (без изменений) ---
class MyClientCB : public BLEClientCallbacks {
    void onConnect(BLEClient* pClient) { connected = true; digitalWrite(LED_PIN, HIGH); }
    void onDisconnect(BLEClient* pClient) { connected = false; digitalWrite(LED_PIN, LOW); }
};

class MyScanCB : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) {
        if (dev.haveServiceUUID() && dev.getServiceUUID().equals(SERVICE_UUID)) {
            targetDevice = new BLEAdvertisedDevice(dev);
            BLEDevice::getScan()->stop();
        }
    }
};

void connectToDevice() {
    if (!targetDevice || connected) return;
    if (client) { delete client; client = nullptr; }
    client = BLEDevice::createClient();
    client->setClientCallbacks(new MyClientCB());
    if (!client->connect(targetDevice)) { delete targetDevice; targetDevice = nullptr; return; }
    BLERemoteService* srv = client->getService(SERVICE_UUID);
    if (!srv) { client->disconnect(); delete targetDevice; targetDevice = nullptr; return; }
    BLERemoteCharacteristic* ch = srv->getCharacteristic(CHAR_UUID);
    if (!ch) { client->disconnect(); delete targetDevice; targetDevice = nullptr; return; }
    if (ch->canNotify()) {
        ch->registerForNotify([](BLERemoteCharacteristic* pChar, uint8_t* data, size_t len, bool isNotify) {
            parsePacket(data, len);
        });
        BLERemoteDescriptor* cccd = ch->getDescriptor(CCCD_UUID);
        if (cccd) { uint8_t val[] = {0x01, 0x00}; cccd->writeValue(val, 2); }
        delete targetDevice; targetDevice = nullptr;
    }
}

void startScan() {
    if (connected) return;
    BLEScan* scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new MyScanCB());
    scan->setActiveScan(true); scan->setInterval(100); scan->setWindow(99);
    scan->start(5, false);
}

void setup() {
    Serial.begin(115200);

    // --- USB HID Init ---
    TinyUSBDevice.setManufacturerDescriptor("Custom");
    TinyUSBDevice.setProductDescriptor("Daydream RC Gamepad");
    TinyUSBDevice.setID(0xCafe, 0x4001);

    if (!TinyUSBDevice.isInitialized()) {
        TinyUSBDevice.begin(0);
    }
    usb_hid.begin();

    // Форсируем переподключение (иногда нужно для определения на ПК)
    if (TinyUSBDevice.mounted()) {
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }

    // --- BLE Init ---
    pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);
    BLEDevice::init("DaydreamPureAccelRC");
    startScan();
}

void loop() {
    if (targetDevice && !connected) connectToDevice();
    if (!connected && !targetDevice && client) {
        static unsigned long lastRec = 0;
        if (millis() - lastRec > 5000) { lastRec = millis(); startScan(); }
    }
    delay(10);
}