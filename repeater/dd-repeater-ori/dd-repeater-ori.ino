/**
 * Daydream Controller - FPV Vector Control v3.1
 * 
 * Ключевые исправления:
 * 1. raw_x/y/z = axis-angle → правильный quaternion
 * 2. Управление через forward/up векторы
 * 3. Roll через проекцию worldUp (правильная изоляция!)
 * 
 * Управление:
 * - APP: калибровка нейтрали
 * - VOL UP: увеличить чувствительность
 * - VOL DOWN: уменьшить чувствительность
 * - BOOT: сброс калибровки
 */

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <math.h>

#define LED_PIN 8
#define BOOT_BTN_PIN 0

// ─── НАСТРОЙКИ ──────────────────────────────────────────────────────────────
#define PITCH_RANGE_DEFAULT  45.0f
#define ROLL_RANGE_DEFAULT   45.0f
#define YAW_RANGE_DEFAULT    60.0f
#define DEADZONE_DEFAULT     1.0f

// ─── BLE UUID ──────────────────────────────────────────────────────────────
static BLEUUID SERVICE_UUID("0000fe55-0000-1000-8000-00805f9b34fb");
static BLEUUID CHAR_UUID("00000001-1000-1000-8000-00805f9b34fb");
static BLEUUID CCCD_UUID("00002902-0000-1000-8000-00805f9b34fb");

// ─── КНОПКИ ──────────────────────────────────────────────────────────────
static const uint8_t BTN_CLICK = 0x01;
static const uint8_t BTN_HOME = 0x02;
static const uint8_t BTN_APP = 0x04;
static const uint8_t BTN_VOL_DOWN = 0x08;
static const uint8_t BTN_VOL_UP = 0x10;

// ─── СТРУКТУРЫ ──────────────────────────────────────────────────────────
struct Quat {
    float w, x, y, z;
    
    void normalize() {
        float len = sqrt(w*w + x*x + y*y + z*z);
        if (len > 0.0001f) { w/=len; x/=len; y/=len; z/=len; }
    }
};

struct Vec3 {
    float x, y, z;
};

struct Euler {
    float pitch, roll, yaw;
};

// ─── ВЕКТОРНЫЕ ОПЕРАЦИИ ────────────────────────────────────────────────
Vec3 crossVec(Vec3 a, Vec3 b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float dotVec(Vec3 a, Vec3 b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 normalizeVec(Vec3 v) {
    float len = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (len < 0.000001f) return {0, 0, 0};
    return {v.x/len, v.y/len, v.z/len};
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

// ─── RC КАНАЛЫ ────────────────────────────────────────────────────────────
struct RCChannels {
    int16_t pitch, roll, yaw, throttle;
} rc;

// ─── КАЛИБРОВКА ──────────────────────────────────────────────────────────
struct Calibration {
    Quat q_neutral_inv = {1, 0, 0, 0};
    bool valid = false;
} cal;

// ─── ЧУВСТВИТЕЛЬНОСТЬ ─────────────────────────────────────────────────
struct Sensitivity {
    float pitch_range = PITCH_RANGE_DEFAULT;
    float roll_range = ROLL_RANGE_DEFAULT;
    float yaw_range = YAW_RANGE_DEFAULT;
    float deadzone = DEADZONE_DEFAULT;
} sens;

// ─── ФИЛЬТР ──────────────────────────────────────────────────────────────
struct AngleFilter {
    float pitch = 0, roll = 0, yaw = 0;
    float alpha = 0.3f;
} filter;

// ─── ОТЛАДКА ─────────────────────────────────────────────────────────────
struct DebugVectors {
    Vec3 forward;
    Vec3 up;
    Vec3 right;
    Vec3 up_zero;
} dbg;

bool connected = false;
bool gotData = false;
BLEClient* client = nullptr;
BLEAdvertisedDevice* targetDevice = nullptr;

Preferences prefs;

// ─── LED ──────────────────────────────────────────────────────────────────
void ledOn() { digitalWrite(LED_PIN, HIGH); }
void ledOff() { digitalWrite(LED_PIN, LOW); }
void ledBlink(int count, int onMs, int offMs) {
    for (int i = 0; i < count; i++) { ledOn(); delay(onMs); ledOff(); delay(offMs); }
}

// ─── ЗНАКОВОЕ РАСШИРЕНИЕ ──────────────────────────────────────────────
int16_t signExtend13(int16_t v) {
    if (v & 0x1000) v |= 0xE000;
    return v;
}

// ─── ПАРСИНГ ──────────────────────────────────────────────────────────────
void parsePacket(const uint8_t* data, size_t len) {
    if (len < 20) return;
    
    cs.raw_x = signExtend13(((data[1]&0x03)<<11) | (data[2]<<3) | ((data[3]&0xE0)>>5));
    cs.raw_z = signExtend13(((data[3]&0x1F)<<8) | (data[4]&0xFF));
    cs.raw_y = signExtend13(((data[5]&0xFF)<<5) | ((data[6]&0xF8)>>3));
    
    cs.trackpad_x = ((data[16]&0x1F)<<3) | ((data[17]&0xE0)>>5);
    cs.trackpad_y = ((data[17]&0x1F)<<3) | ((data[18]&0xE0)>>5);
    
    uint8_t btns = data[18];
    cs.click = (btns & BTN_CLICK) != 0;
    cs.home = (btns & BTN_HOME) != 0;
    cs.app = (btns & BTN_APP) != 0;
    cs.vd = (btns & BTN_VOL_DOWN) != 0;
    cs.vu = (btns & BTN_VOL_UP) != 0;
    
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
        cs.q.w = 1.0f;
        cs.q.x = 0.0f;
        cs.q.y = 0.0f;
        cs.q.z = 0.0f;
        cs.axis_x = 0;
        cs.axis_y = 0;
        cs.axis_z = 1;
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

// ─── УМНОЖЕНИЕ КВАТЕРНИОНОВ ──────────────────────────────────────────
Quat quatMul(Quat a, Quat b) {
    Quat r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

// ─── ПОВОРОТ ВЕКТОРА КВАТЕРНИОНОМ ────────────────────────────────────
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

// ─── НОРМАЛИЗАЦИЯ УГЛА ──────────────────────────────────────────────────
float wrap180(float a) {
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

// ─── ПОЛУЧЕНИЕ УГЛОВ ЧЕРЕЗ ВЕКТОРЫ (v3.1) ────────────────────────────
Euler getVectorAngles() {
    Euler result = {0, 0, 0};
    if (!cal.valid) return result;
    
    computeQuaternion();
    
    // Локальный кватернион
    Quat q_local = quatMul(cal.q_neutral_inv, cs.q);
    q_local.normalize();
    
    // Forward, Up, Right векторы
    dbg.forward = rotateVector(q_local, {0, 0, -1});
    dbg.up      = rotateVector(q_local, {0, 1, 0});
    dbg.right   = rotateVector(q_local, {1, 0, 0});
    
    // ─── PITCH ───
    result.pitch = atan2f(dbg.forward.y,
                          sqrtf(dbg.forward.x*dbg.forward.x + 
                                dbg.forward.z*dbg.forward.z)) * 180.0f / M_PI;
    
    // ─── YAW ───
    result.yaw = atan2f(dbg.forward.x, -dbg.forward.z) * 180.0f / M_PI;
    
    // ─── ROLL (правильная формула через проекцию) ───
    Vec3 worldUp = {0, 1, 0};
    
    // Проецируем worldUp на плоскость, перпендикулярную forward
    float d = dotVec(worldUp, dbg.forward);
    Vec3 up_zero = {
        worldUp.x - dbg.forward.x * d,
        worldUp.y - dbg.forward.y * d,
        worldUp.z - dbg.forward.z * d
    };
    up_zero = normalizeVec(up_zero);
    dbg.up_zero = up_zero;
    
    // Угол между реальным up и нулевым up вокруг оси forward
    Vec3 c = crossVec(dbg.up, up_zero);
    float sinRoll = dotVec(dbg.forward, c);
    float cosRoll = dotVec(dbg.up, up_zero);
    
    result.roll = atan2f(sinRoll, cosRoll) * 180.0f / M_PI;
    
    result.pitch = wrap180(result.pitch);
    result.roll = wrap180(result.roll);
    result.yaw = wrap180(result.yaw);
    
    return result;
}

// ─── КАЛИБРОВКА ──────────────────────────────────────────────────────────
void calibrateNeutral() {
    computeQuaternion();
    
    cal.q_neutral_inv.w = cs.q.w;
    cal.q_neutral_inv.x = -cs.q.x;
    cal.q_neutral_inv.y = -cs.q.y;
    cal.q_neutral_inv.z = -cs.q.z;
    cal.valid = true;
    
    filter.pitch = 0;
    filter.roll = 0;
    filter.yaw = 0;
    
    prefs.begin("fpv", false);
    prefs.putFloat("qw", cal.q_neutral_inv.w);
    prefs.putFloat("qx", cal.q_neutral_inv.x);
    prefs.putFloat("qy", cal.q_neutral_inv.y);
    prefs.putFloat("qz", cal.q_neutral_inv.z);
    prefs.putBool("cal", true);
    prefs.end();
    
    Serial.printf("\n🎯 CALIBRATED:\n");
    Serial.printf("   raw=(%d, %d, %d)\n", cs.raw_x, cs.raw_y, cs.raw_z);
    Serial.printf("   angle=%.2f° axis=(%.3f, %.3f, %.3f)\n",
                 cs.angle_rad * 180.0f / M_PI,
                 cs.axis_x, cs.axis_y, cs.axis_z);
    Serial.printf("   q=(%.4f, %.4f, %.4f, %.4f)\n", 
                 cs.q.w, cs.q.x, cs.q.y, cs.q.z);
    ledBlink(3, 100, 100);
}

void loadCalibration() {
    prefs.begin("fpv", true);
    if (prefs.getBool("cal", false)) {
        cal.q_neutral_inv.w = prefs.getFloat("qw", 1.0f);
        cal.q_neutral_inv.x = prefs.getFloat("qx", 0.0f);
        cal.q_neutral_inv.y = prefs.getFloat("qy", 0.0f);
        cal.q_neutral_inv.z = prefs.getFloat("qz", 0.0f);
        cal.q_neutral_inv.normalize();  // страховка
        cal.valid = true;
        prefs.end();
        Serial.println(F("📂 Calibration loaded"));
    } else {
        prefs.end();
        Serial.println(F("⚠️ No saved calibration"));
    }
}

// ─── ФИЛЬТР ─────────────────────────────────────────────────────────────
float angleLerp(float current, float target, float alpha) {
    float d = wrap180(target - current);
    return current + d * alpha;
}

Euler filterEuler(Euler raw) {
    filter.pitch = angleLerp(filter.pitch, raw.pitch, filter.alpha);
    filter.roll  = angleLerp(filter.roll,  raw.roll,  filter.alpha);
    filter.yaw   = angleLerp(filter.yaw,   raw.yaw,   filter.alpha);
    return {filter.pitch, filter.roll, filter.yaw};
}

// ─── ПРЕОБРАЗОВАНИЕ В RC ──────────────────────────────────────────────
void convertToRC(Euler e) {
    float p = constrain(e.pitch, -sens.pitch_range, sens.pitch_range);
    float r = constrain(e.roll, -sens.roll_range, sens.roll_range);
    float y = constrain(e.yaw, -sens.yaw_range, sens.yaw_range);
    
    if (fabs(p) < sens.deadzone) p = 0;
    if (fabs(r) < sens.deadzone) r = 0;
    if (fabs(y) < sens.deadzone) y = 0;
    
    rc.pitch = 1500 + (int16_t)(p / sens.pitch_range * 500);
    rc.roll = 1500 + (int16_t)(r / sens.roll_range * 500);
    rc.yaw = 1500 + (int16_t)(y / sens.yaw_range * 500);
    
    rc.pitch = constrain(rc.pitch, 1000, 2000);
    rc.roll = constrain(rc.roll, 1000, 2000);
    rc.yaw = constrain(rc.yaw, 1000, 2000);
    
    // Газ
    bool touching = (cs.trackpad_x > 0 || cs.trackpad_y > 0);
    static int16_t throttleSmooth = 1000;
    if (touching) {
        float raw = 1.0f - (cs.trackpad_y / 255.0f);
        raw = constrain(raw, 0.0f, 1.0f);
        int16_t target = 1000 + (int16_t)(raw * 1000);
        throttleSmooth += (target - throttleSmooth) * 0.3f;
        rc.throttle = throttleSmooth;
    } else {
        throttleSmooth += (1000 - throttleSmooth) * 0.05f;
        rc.throttle = throttleSmooth;
    }
    rc.throttle = constrain(rc.throttle, 1000, 2000);
}

// ─── ВЫВОД ──────────────────────────────────────────────────────────────
void printData(Euler e) {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint < 50) return;
    lastPrint = millis();
    
    Serial.printf("RAW:%6d %6d %6d | ", cs.raw_x, cs.raw_y, cs.raw_z);
    Serial.printf("F:%.2f %.2f %.2f | ", dbg.forward.x, dbg.forward.y, dbg.forward.z);
    Serial.printf("U:%.2f %.2f %.2f | ", dbg.up.x, dbg.up.y, dbg.up.z);
    Serial.printf("U0:%.2f %.2f %.2f | ", dbg.up_zero.x, dbg.up_zero.y, dbg.up_zero.z);
    Serial.printf("E:%7.2f %7.2f %7.2f | ", e.pitch, e.roll, e.yaw);
    Serial.printf("RC:%4d %4d %4d %4d\n", rc.pitch, rc.roll, rc.yaw, rc.throttle);
}

// ─── ОБРАБОТКА ──────────────────────────────────────────────────────────
void processData() {
    if (!gotData) return;
    gotData = false;
    
    if (cal.valid) {
        Euler raw = getVectorAngles();
        Euler filtered = filterEuler(raw);
        convertToRC(filtered);
        printData(filtered);
    } else {
        computeQuaternion();
    }
}

// ─── РЕГУЛИРОВКА ────────────────────────────────────────────────────────
void increaseSensitivity() {
    sens.pitch_range = max(5.0f, sens.pitch_range * 0.85f);
    sens.roll_range = max(5.0f, sens.roll_range * 0.85f);
    sens.yaw_range = max(5.0f, sens.yaw_range * 0.85f);
    Serial.printf("↑ SENS: P=%.1f R=%.1f Y=%.1f\n", 
                 sens.pitch_range, sens.roll_range, sens.yaw_range);
    ledBlink(2, 50, 50);
}

void decreaseSensitivity() {
    sens.pitch_range = min(180.0f, sens.pitch_range * 1.15f);
    sens.roll_range = min(180.0f, sens.roll_range * 1.15f);
    sens.yaw_range = min(180.0f, sens.yaw_range * 1.15f);
    Serial.printf("↓ SENS: P=%.1f R=%.1f Y=%.1f\n", 
                 sens.pitch_range, sens.roll_range, sens.yaw_range);
    ledBlink(3, 50, 50);
}

// ─── BLE ──────────────────────────────────────────────────────────────────
class MyClientCB : public BLEClientCallbacks {
    void onConnect(BLEClient* p) { connected = true; ledOn(); Serial.println("C"); }
    void onDisconnect(BLEClient* p) { connected = false; ledOff(); Serial.println("D"); }
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
        ch->registerForNotify([](BLERemoteCharacteristic* p, uint8_t* d, size_t l, bool n) {
            parsePacket(d, l);
        });
        BLERemoteDescriptor* cccd = ch->getDescriptor(CCCD_UUID);
        if (cccd) { uint8_t val[] = {0x01, 0x00}; cccd->writeValue(val, 2); }
        Serial.println("N");
        delete targetDevice; targetDevice = nullptr;
        loadCalibration();
    } else {
        Serial.println("No notify");
        client->disconnect(); delete targetDevice; targetDevice = nullptr;
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

// ─── SETUP ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);
    pinMode(LED_PIN, OUTPUT); ledOff();
    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
    
    Serial.println("\n╔═══════════════════════════════════════════════════════════════════╗");
    Serial.println("║   FPV Vector Control v3.1                                         ║");
    Serial.println("╠═══════════════════════════════════════════════════════════════════╣");
    Serial.println("║  APP      = калибровка нейтрали                                  ║");
    Serial.println("║  VOL UP   = увеличить чувствительность                           ║");
    Serial.println("║  VOL DOWN = уменьшить чувствительность                           ║");
    Serial.println("║  BOOT     = сброс калибровки                                     ║");
    Serial.println("║                                                                   ║");
    Serial.println("║  Roll через проекцию worldUp — правильная изоляция!               ║");
    Serial.println("╚═══════════════════════════════════════════════════════════════════╝");
    
    BLEDevice::init("DaydreamFPV");
    BLEDevice::setPower(ESP_PWR_LVL_P9);
    startScan();
}

// ─── LOOP ──────────────────────────────────────────────────────────────────
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
    
    if (digitalRead(BOOT_BTN_PIN) == LOW) {
        static unsigned long last = 0;
        if (millis() - last > 300) {
            last = millis();
            Serial.println("R");
            cal.valid = false;
            prefs.begin("fpv", false);
            prefs.putBool("cal", false);
            prefs.end();
            if (client) { client->disconnect(); delete client; client = nullptr; }
            connected = false; ledOff(); delete targetDevice; targetDevice = nullptr;
            delay(100); startScan();
        }
    }
    
    delay(10);
}