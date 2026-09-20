/**
 * Daydream Controller → USB HID Gamepad
 * Управление через ОРИЕНТАЦИЮ (quaternion / forward-up vectors)
 * 
 * Оси HID:
 *   X  = Roll   (ориентация)
 *   Y  = Pitch  (ориентация)
 *   Z  = Yaw    (ориентация)
 *   Rz = Throttle (трекпад)
 *
 * Кнопки HID:
 *   1 = Arm        (зажат APP)
 *   2 = Mode       (клик)
 *   3 = Click      (трекпад-клик)
 *   4 = App
 *
 * Кнопки Daydream:
 *   APP      = калибровка нейтрали
 *   VOL UP   = чувствительность +
 *   VOL DOWN = чувствительность -
 *   BOOT     = сброс калибровки
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <math.h>
#include <Adafruit_TinyUSB.h>

#define LED_PIN       8
#define BOOT_BTN_PIN  0

// ─── USB HID DESCRIPTOR ─────────────────────────────────────────────────
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

Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report),
                          HID_ITF_PROTOCOL_NONE, 2, false);

typedef struct __attribute__((packed)) {
    uint16_t buttons;
    int8_t x;   // roll
    int8_t y;   // pitch
    int8_t z;   // yaw
    int8_t rz;  // throttle
} GamepadReport;

// ─── НАСТРОЙКИ ──────────────────────────────────────────────────────────
#define PITCH_RANGE_DEFAULT  45.0f
#define ROLL_RANGE_DEFAULT   45.0f
#define YAW_RANGE_DEFAULT    60.0f
#define DEADZONE_DEFAULT     1.0f

// ─── BLE UUID ──────────────────────────────────────────────────────────
static BLEUUID SERVICE_UUID("0000fe55-0000-1000-8000-00805f9b34fb");
static BLEUUID CHAR_UUID   ("00000001-1000-1000-8000-00805f9b34fb");
static BLEUUID CCCD_UUID   ("00002902-0000-1000-8000-00805f9b34fb");

// ─── КНОПКИ DAYDREAM ───────────────────────────────────────────────────
static const uint8_t BTN_CLICK    = 0x01;
static const uint8_t BTN_HOME     = 0x02;
static const uint8_t BTN_APP      = 0x04;
static const uint8_t BTN_VOL_DOWN = 0x08;
static const uint8_t BTN_VOL_UP   = 0x10;

// ─── СТРУКТУРЫ ──────────────────────────────────────────────────────────
struct Quat { float w, x, y, z;
    void normalize() {
        float len = sqrtf(w*w + x*x + y*y + z*z);
        if (len > 0.0001f) { w/=len; x/=len; y/=len; z/=len; }
    }
};
struct Vec3  { float x, y, z; };
struct Euler { float pitch, roll, yaw; };

// ─── ВЕКТОРНЫЕ ОПЕРАЦИИ ────────────────────────────────────────────────
Vec3 crossVec(Vec3 a, Vec3 b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
float dotVec(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 normalizeVec(Vec3 v) {
    float len = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len < 0.000001f) return {0, 0, 0};
    return { v.x/len, v.y/len, v.z/len };
}

// ─── СОСТОЯНИЕ ──────────────────────────────────────────────────────────
struct ControllerState {
    int16_t raw_x, raw_y, raw_z;
    Quat q;
    float angle_rad;
    float axis_x, axis_y, axis_z;
    uint8_t trackpad_x, trackpad_y;
    bool click, home, app, vd, vu;
} cs;

// ─── RC КАНАЛЫ ──────────────────────────────────────────────────────────
struct RCChannels {
    int16_t pitch, roll, yaw, throttle;
} rc;

// ─── КАЛИБРОВКА ─────────────────────────────────────────────────────────
struct Calibration {
    Quat q_neutral_inv = {1, 0, 0, 0};
    bool valid = false;
} cal;

// ─── ЧУВСТВИТЕЛЬНОСТЬ ──────────────────────────────────────────────────
struct Sensitivity {
    float pitch_range = PITCH_RANGE_DEFAULT;
    float roll_range  = ROLL_RANGE_DEFAULT;
    float yaw_range   = YAW_RANGE_DEFAULT;
    float deadzone    = DEADZONE_DEFAULT;
} sens;

// ─── ФИЛЬТР ─────────────────────────────────────────────────────────────
struct AngleFilter {
    float pitch = 0, roll = 0, yaw = 0;
    float alpha = 0.3f;
} filter;

// ─── ОТЛАДКА ────────────────────────────────────────────────────────────
struct DebugVectors {
    Vec3 forward, up, right, up_zero;
} dbg;

bool connected = false;
bool gotData = false;
BLEClient* client = nullptr;
BLEAdvertisedDevice* targetDevice = nullptr;
Preferences prefs;

// ─── LED ────────────────────────────────────────────────────────────────
void ledOn()  { digitalWrite(LED_PIN, HIGH); }
void ledOff() { digitalWrite(LED_PIN, LOW); }
void ledBlink(int count, int onMs, int offMs) {
    for (int i = 0; i < count; i++) { ledOn(); delay(onMs); ledOff(); delay(offMs); }
}

// ─── ЗНАКОВОЕ РАСШИРЕНИЕ ───────────────────────────────────────────────
int16_t signExtend13(int16_t v) {
    if (v & 0x1000) v |= 0xE000;
    return v;
}

// ─── ПАРСИНГ ────────────────────────────────────────────────────────────
void parsePacket(const uint8_t* data, size_t len) {
    if (len < 20 || data == nullptr) return;

    cs.raw_x = signExtend13(((data[1]&0x03)<<11) | (data[2]<<3) | ((data[3]&0xE0)>>5));
    cs.raw_z = signExtend13(((data[3]&0x1F)<<8) | (data[4]&0xFF));
    cs.raw_y = signExtend13(((data[5]&0xFF)<<5) | ((data[6]&0xF8)>>3));

    cs.trackpad_x = ((data[16]&0x1F)<<3) | ((data[17]&0xE0)>>5);
    cs.trackpad_y = ((data[17]&0x1F)<<3) | ((data[18]&0xE0)>>5);

    uint8_t btns = data[18];
    cs.click = (btns & BTN_CLICK)    != 0;
    cs.home  = (btns & BTN_HOME)     != 0;
    cs.app   = (btns & BTN_APP)      != 0;
    cs.vd    = (btns & BTN_VOL_DOWN) != 0;
    cs.vu    = (btns & BTN_VOL_UP)   != 0;

    gotData = true;
}

// ─── AXIS-ANGLE → QUATERNION ──────────────────────────────────────────
void computeQuaternion() {
    const float SCALE = 2.0f * M_PI / 4095.0f;

    float vx = cs.raw_x * SCALE;
    float vy = cs.raw_y * SCALE;
    float vz = cs.raw_z * SCALE;

    float angle = sqrtf(vx*vx + vy*vy + vz*vz);
    cs.angle_rad = angle;

    if (angle < 0.000001f) {
        cs.q = {1, 0, 0, 0};
        cs.axis_x = 0; cs.axis_y = 0; cs.axis_z = 1;
        return;
    }
    cs.axis_x = vx / angle;
    cs.axis_y = vy / angle;
    cs.axis_z = vz / angle;

    float halfAngle = angle * 0.5f;
    float s = sinf(halfAngle);
    cs.q.w = cosf(halfAngle);
    cs.q.x = cs.axis_x * s;
    cs.q.y = cs.axis_y * s;
    cs.q.z = cs.axis_z * s;
    cs.q.normalize();
}

// ─── КВАТЕРНИОНЫ ───────────────────────────────────────────────────────
Quat quatMul(Quat a, Quat b) {
    return {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
}

Vec3 rotateVector(Quat q, Vec3 v) {
    Vec3 r;
    r.x = (1.0f - 2.0f*(q.y*q.y + q.z*q.z))*v.x +
          2.0f*(q.x*q.y - q.w*q.z)*v.y +
          2.0f*(q.x*q.z + q.w*q.y)*v.z;
    r.y = 2.0f*(q.x*q.y + q.w*q.z)*v.x +
          (1.0f - 2.0f*(q.x*q.x + q.z*q.z))*v.y +
          2.0f*(q.y*q.z - q.w*q.x)*v.z;
    r.z = 2.0f*(q.x*q.z - q.w*q.y)*v.x +
          2.0f*(q.y*q.z + q.w*q.x)*v.y +
          (1.0f - 2.0f*(q.x*q.x + q.y*q.y))*v.z;
    return r;
}

float wrap180(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// ─── УГЛЫ ЧЕРЕЗ ВЕКТОРЫ ────────────────────────────────────────────────
Euler getVectorAngles() {
    Euler result = {0, 0, 0};
    if (!cal.valid) return result;

    computeQuaternion();

    Quat q_local = quatMul(cal.q_neutral_inv, cs.q);
    q_local.normalize();

    dbg.forward = rotateVector(q_local, {0, 0, -1});
    dbg.up      = rotateVector(q_local, {0, 1, 0});
    dbg.right   = rotateVector(q_local, {1, 0, 0});

    // PITCH
    result.pitch = atan2f(dbg.forward.y,
                          sqrtf(dbg.forward.x*dbg.forward.x +
                                dbg.forward.z*dbg.forward.z)) * 180.0f / M_PI;

    // YAW
    result.yaw = atan2f(dbg.forward.x, -dbg.forward.z) * 180.0f / M_PI;

    // ROLL через проекцию worldUp
    Vec3 worldUp = {0, 1, 0};
    float d = dotVec(worldUp, dbg.forward);
    Vec3 up_zero = {
        worldUp.x - dbg.forward.x * d,
        worldUp.y - dbg.forward.y * d,
        worldUp.z - dbg.forward.z * d
    };
    up_zero = normalizeVec(up_zero);
    dbg.up_zero = up_zero;

    Vec3 c = crossVec(dbg.up, up_zero);
    float sinRoll = dotVec(dbg.forward, c);
    float cosRoll = dotVec(dbg.up, up_zero);
    result.roll = atan2f(sinRoll, cosRoll) * 180.0f / M_PI;

    result.pitch = wrap180(result.pitch);
    result.roll  = wrap180(result.roll);
    result.yaw   = wrap180(result.yaw);
    return result;
}

// ─── КАЛИБРОВКА ────────────────────────────────────────────────────────
void calibrateNeutral() {
    computeQuaternion();

    cal.q_neutral_inv.w =  cs.q.w;
    cal.q_neutral_inv.x = -cs.q.x;
    cal.q_neutral_inv.y = -cs.q.y;
    cal.q_neutral_inv.z = -cs.q.z;
    cal.valid = true;

    filter.pitch = filter.roll = filter.yaw = 0;

    prefs.begin("fpv", false);
    prefs.putFloat("qw", cal.q_neutral_inv.w);
    prefs.putFloat("qx", cal.q_neutral_inv.x);
    prefs.putFloat("qy", cal.q_neutral_inv.y);
    prefs.putFloat("qz", cal.q_neutral_inv.z);
    prefs.putBool ("cal", true);
    prefs.end();

    Serial.printf("\n🎯 CALIBRATED: raw=(%d,%d,%d) angle=%.2f°\n",
                  cs.raw_x, cs.raw_y, cs.raw_z,
                  cs.angle_rad * 180.0f / M_PI);
    ledBlink(3, 100, 100);
}

void loadCalibration() {
    prefs.begin("fpv", true);
    if (prefs.getBool("cal", false)) {
        cal.q_neutral_inv.w = prefs.getFloat("qw", 1.0f);
        cal.q_neutral_inv.x = prefs.getFloat("qx", 0.0f);
        cal.q_neutral_inv.y = prefs.getFloat("qy", 0.0f);
        cal.q_neutral_inv.z = prefs.getFloat("qz", 0.0f);
        cal.q_neutral_inv.normalize();
        cal.valid = true;
        Serial.println(F("📂 Calibration loaded"));
    } else {
        Serial.println(F("⚠️ No saved calibration"));
    }
    prefs.end();
}

// ─── ФИЛЬТР ────────────────────────────────────────────────────────────
float angleLerp(float cur, float tgt, float a) {
    return cur + wrap180(tgt - cur) * a;
}
Euler filterEuler(Euler raw) {
    filter.pitch = angleLerp(filter.pitch, raw.pitch, filter.alpha);
    filter.roll  = angleLerp(filter.roll,  raw.roll,  filter.alpha);
    filter.yaw   = angleLerp(filter.yaw,   raw.yaw,   filter.alpha);
    return { filter.pitch, filter.roll, filter.yaw };
}

// ─── ПРЕОБРАЗОВАНИЕ В RC ───────────────────────────────────────────────
void convertToRC(Euler e) {
    float p = constrain(e.pitch, -sens.pitch_range, sens.pitch_range);
    float r = constrain(e.roll,  -sens.roll_range,  sens.roll_range);
    float y = constrain(e.yaw,   -sens.yaw_range,   sens.yaw_range);

    if (fabsf(p) < sens.deadzone) p = 0;
    if (fabsf(r) < sens.deadzone) r = 0;
    if (fabsf(y) < sens.deadzone) y = 0;

    rc.pitch = 1500 + (int16_t)(p / sens.pitch_range * 500);
    rc.roll  = 1500 + (int16_t)(r / sens.roll_range  * 500);
    rc.yaw   = 1500 + (int16_t)(y / sens.yaw_range   * 500);

    rc.pitch = constrain(rc.pitch, 1000, 2000);
    rc.roll  = constrain(rc.roll,  1000, 2000);
    rc.yaw   = constrain(rc.yaw,   1000, 2000);

    // Газ с трекпада — без сглаживания
    if (cs.trackpad_x > 0 || cs.trackpad_y > 0) {
        rc.throttle = map(cs.trackpad_y, 255, 0, 1000, 2000);
    } else {
        rc.throttle = 1000;
    }
    rc.throttle = constrain(rc.throttle, 1000, 2000);
}

// ─── ОТПРАВКА В USB HID ────────────────────────────────────────────────
void sendHIDReport() {
    GamepadReport report;

    // RC (1000..2000) → HID (-127..127)
    report.x  = map(rc.roll,     1000, 2000, -127, 127); // Roll
    report.y  = map(rc.pitch,    1000, 2000, -127, 127); // Pitch
    report.z  = map(rc.yaw,      1000, 2000, -127, 127); // Yaw
    report.rz = map(rc.throttle, 1000, 2000, -127, 127); // Throttle

    // Кнопки
    report.buttons = 0;
    if (cs.home)  report.buttons |= (1 << 0);  // Arm (HOME)
    if (cs.click) report.buttons |= (1 << 1);  // Mode
    if (cs.click) report.buttons |= (1 << 2);  // Click (можно убрать дубль)
    if (cs.app)   report.buttons |= (1 << 3);  // App

    if (TinyUSBDevice.mounted()) {
        usb_hid.sendReport(0, &report, sizeof(report));
    }

    static unsigned long lp = 0;
    if (millis() - lp > 50) {
        lp = millis();
        Serial.printf("[HID] E:%7.2f %7.2f %7.2f | RC:%4d %4d %4d %4d | "
                      "X:%4d Y:%4d Z:%4d RZ:%4d BTN:0x%04X\n",
                      filter.pitch, filter.roll, filter.yaw,
                      rc.pitch, rc.roll, rc.yaw, rc.throttle,
                      report.x, report.y, report.z, report.rz,
                      report.buttons);
    }
}

// ─── ОБРАБОТКА ─────────────────────────────────────────────────────────
void processData() {
    if (!gotData) return;
    gotData = false;

    if (cal.valid) {
        Euler raw = getVectorAngles();
        Euler flt = filterEuler(raw);
        convertToRC(flt);
        sendHIDReport();
    } else {
        computeQuaternion();
    }
}

// ─── ЧУВСТВИТЕЛЬНОСТЬ ─────────────────────────────────────────────────
void increaseSensitivity() {
    sens.pitch_range = max(5.0f,   sens.pitch_range * 0.85f);
    sens.roll_range  = max(5.0f,   sens.roll_range  * 0.85f);
    sens.yaw_range   = max(5.0f,   sens.yaw_range   * 0.85f);
    Serial.printf("↑ SENS: P=%.1f R=%.1f Y=%.1f\n",
                  sens.pitch_range, sens.roll_range, sens.yaw_range);
    ledBlink(2, 50, 50);
}
void decreaseSensitivity() {
    sens.pitch_range = min(180.0f, sens.pitch_range * 1.15f);
    sens.roll_range  = min(180.0f, sens.roll_range  * 1.15f);
    sens.yaw_range   = min(180.0f, sens.yaw_range   * 1.15f);
    Serial.printf("↓ SENS: P=%.1f R=%.1f Y=%.1f\n",
                  sens.pitch_range, sens.roll_range, sens.yaw_range);
    ledBlink(3, 50, 50);
}

// ─── BLE CALLBACKS ─────────────────────────────────────────────────────
class MyClientCB : public BLEClientCallbacks {
    void onConnect(BLEClient*)    { connected = true;  ledOn();  Serial.println("C"); }
    void onDisconnect(BLEClient*) { connected = false; ledOff(); Serial.println("D"); }
};

class MyScanCB : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) {
        if (dev.haveServiceUUID() && dev.getServiceUUID().equals(SERVICE_UUID)) {
            Serial.printf("F:%s\n", dev.getAddress().toString().c_str());
            targetDevice = new BLEAdvertisedDevice(dev);
            BLEDevice::getScan()->stop();
        }
    }
};

void connectToDevice() {
    if (!targetDevice || connected) return;
    Serial.println("Connecting...");
    if (client) { delete client; client = nullptr; }
    client = BLEDevice::createClient();
    client->setClientCallbacks(new MyClientCB());

    if (!client->connect(targetDevice)) {
        Serial.println("Connect failed");
        delete targetDevice; targetDevice = nullptr; return;
    }
    BLERemoteService* srv = client->getService(SERVICE_UUID);
    if (!srv) { client->disconnect(); delete targetDevice; targetDevice = nullptr; return; }

    BLERemoteCharacteristic* ch = srv->getCharacteristic(CHAR_UUID);
    if (!ch) { client->disconnect(); delete targetDevice; targetDevice = nullptr; return; }

    if (ch->canNotify()) {
        ch->registerForNotify([](BLERemoteCharacteristic*, uint8_t* d, size_t l, bool) {
            parsePacket(d, l);
        });
        BLERemoteDescriptor* cccd = ch->getDescriptor(CCCD_UUID);
        if (cccd) { uint8_t v[] = {0x01, 0x00}; cccd->writeValue(v, 2); }
        Serial.println("N");
        delete targetDevice; targetDevice = nullptr;
        loadCalibration();
    } else {
        Serial.println("No notify");
        client->disconnect();
        delete targetDevice; targetDevice = nullptr;
    }
}

void startScan() {
    if (connected) return;
    Serial.println("S");
    ledOn();
    BLEScan* scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(new MyScanCB());
    scan->setActiveScan(true);
    scan->setInterval(100);
    scan->setWindow(99);
    scan->start(5, false);
}

// ─── SETUP ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    // USB HID
    TinyUSBDevice.setManufacturerDescriptor("Custom");
    TinyUSBDevice.setProductDescriptor("Daydream FPV Gamepad");
    TinyUSBDevice.setID(0xCafe, 0x4001);
    if (!TinyUSBDevice.isInitialized()) TinyUSBDevice.begin(0);
    usb_hid.begin();
    if (TinyUSBDevice.mounted()) {
        TinyUSBDevice.detach(); delay(10); TinyUSBDevice.attach();
    }

    // GPIO
    pinMode(LED_PIN, OUTPUT); ledOff();
    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

    Serial.println("\n╔═══════════════════════════════════════════════════════════════╗");
    Serial.println("║  Daydream Orientation → USB HID Gamepad                       ║");
    Serial.println("║  APP=калибровка  VOL+=чувств+  VOL-=чувств-  BOOT=сброс       ║");
    Serial.println("╚═══════════════════════════════════════════════════════════════╝");

    // BLE
    BLEDevice::init("DaydreamFPV");
    BLEDevice::setPower(ESP_PWR_LVL_P9);
    startScan();
}

// ─── LOOP ──────────────────────────────────────────────────────────────
void loop() {
    static unsigned long lastBlink = 0, lastReconnect = 0;
    static bool prevApp = false, prevVu = false, prevVd = false;

    if (!connected && millis() - lastBlink > 2000) {
        lastBlink = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }

    if (cs.app && !prevApp) calibrateNeutral();
    prevApp = cs.app;

    if (cs.vu && !prevVu) increaseSensitivity();
    prevVu = cs.vu;

    if (cs.vd && !prevVd) decreaseSensitivity();
    prevVd = cs.vd;

    if (gotData) processData();

    if (targetDevice && !connected) connectToDevice();

    if (!connected && !targetDevice && client) {
        if (millis() - lastReconnect > 5000) {
            lastReconnect = millis();
            startScan();
        }
    }

    // BOOT — сброс калибровки
    if (digitalRead(BOOT_BTN_PIN) == LOW) {
        static unsigned long last = 0;
        if (millis() - last > 300) {
            last = millis();
            Serial.println("R: reset calibration");
            cal.valid = false;
            prefs.begin("fpv", false);
            prefs.putBool("cal", false);
            prefs.end();
            if (client) { client->disconnect(); delete client; client = nullptr; }
            connected = false; ledOff();
            delete targetDevice; targetDevice = nullptr;
            delay(100);
            startScan();
        }
    }

    delay(10);
}