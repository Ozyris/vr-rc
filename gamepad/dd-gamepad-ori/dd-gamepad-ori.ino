/**
 * Daydream Controller → USB HID Gamepad
 * Управление через ОРИЕНТАЦИЮ (кватернион, forward/up, gravity)
 *
 * ─── Схема калибровки ──────────────────────────────────────────────────
 *   APP → startCalibration():
 *       • yaw_reference = текущий абсолютный yaw (момент нажатия)
 *       • запускается сессия сбора 32 BLE-пакетов
 *       • фильтр НЕ сбрасывается — HID не прыгает в центр
 *
 *   Завершение сессии:
 *       • cal.valid = true
 *       • filter.pitch/roll = текущие абсолютные
 *       • filter.yaw = 0   (центрируем только yaw)
 *
 *   Проверка неподвижности через акселерометр временно отключена.
 *
 * ─── Важное про оси Daydream ───────────────────────────────────────────
 *   Yaw  — вращение вокруг МИРОВОЙ вертикали (формула через проекцию
 *          worldUp на плоскость ⟂ forward).
 *   Roll — вращение вокруг оси forward (atan2 по forward.x/z).
 *   Pitch — наклон forward вверх/вниз от горизонта.
 *
 * ─── Миксер осей (дефайны AXIS_*_SRC) ──────────────────────────────────
 *   SRC_ROLL / SRC_PITCH / SRC_YAW  — с ориентации
 *   SRC_TP_X / SRC_TP_Y             — с трекпада
 *   SRC_NONE                        — ось в центре
 *
 * ─── LED-индикация ─────────────────────────────────────────────────────
 *   1 вспышка           — начало калибровки
 *   3 вспышки (100/100) — калибровка успешна
 *   2 вспышки (50/50)   — VOL+ (чувствительность выше)
 *   3 вспышки (50/50)   — VOL− (чувствительность ниже)
 *   мигание 1 Гц        — ожидание подключения BLE
 *
 * ─── Кнопки Daydream ───────────────────────────────────────────────────
 *   APP      = калибровка (yaw reference)
 *   VOL UP   = чувствительность +
 *   VOL DOWN = чувствительность −
 *   BOOT     = сброс калибровки
 *
 * ─── Кнопки HID ────────────────────────────────────────────────────────
 *   1 = Arm    (HOME)
 *   2 = Mode   (клик)
 *   3 = Click  (трекпад-клик)
 *   4 = App
 *
 * Serial не используется.
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

#define LED_PIN       48
#define BOOT_BTN_PIN  0

// ─── МИКСЕР: ИСТОЧНИКИ ДЛЯ HID-ОСЕЙ ────────────────────────────────────
#define SRC_NONE   0
#define SRC_ROLL   1
#define SRC_PITCH  2
#define SRC_YAW    3
#define SRC_TP_X   4
#define SRC_TP_Y   5

#define AXIS_X_SRC   SRC_ROLL    // HID X  ← Roll
#define AXIS_Y_SRC   SRC_PITCH   // HID Y  ← Pitch
#define AXIS_Z_SRC   SRC_YAW     // HID Z  ← Yaw
#define AXIS_RZ_SRC  SRC_TP_Y    // HID Rz ← Throttle (трекпад Y)

#define INV_X   0
#define INV_Y   0
#define INV_Z   0
#define INV_RZ  0

// ─── USB HID DESCRIPTOR ─────────────────────────────────────────────────
uint8_t const desc_hid_report[] = {
  0x05, 0x01,
  0x09, 0x05,
  0xA1, 0x01,

  0x05, 0x09,
  0x19, 0x01,
  0x29, 0x10,
  0x15, 0x00,
  0x25, 0x01,
  0x95, 0x10,
  0x75, 0x01,
  0x81, 0x02,

  0x05, 0x01,
  0x09, 0x30,
  0x09, 0x31,
  0x09, 0x32,
  0x09, 0x35,
  0x15, 0x81,
  0x25, 0x7F,
  0x75, 0x08,
  0x95, 0x04,
  0x81, 0x02,

  0xC0
};

Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report),
                          HID_ITF_PROTOCOL_NONE, 2, false);

typedef struct __attribute__((packed)) {
    uint16_t buttons;
    int8_t x;
    int8_t y;
    int8_t z;
    int8_t rz;
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
    float acc_x, acc_y, acc_z;
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

// ─── КАЛИБРОВКА ────────────────────────────────────────────────────────
struct Calibration {
    float yaw_reference = 0.0f;
    bool valid = false;
} cal;

// ─── СЕССИЯ КАЛИБРОВКИ ─────────────────────────────────────────────────
struct CalibrationSession {
    bool active = false;
    uint16_t samples = 0;
} calSession;

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
void ledOff()  { digitalWrite(LED_PIN, HIGH); }
void ledOn()   { digitalWrite(LED_PIN, LOW);  }
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

    // Ориентация (13-бит, axis-angle)
    cs.raw_x = signExtend13(((data[1]&0x03)<<11) | (data[2]<<3) | ((data[3]&0xE0)>>5));
    cs.raw_z = signExtend13(((data[3]&0x1F)<<8) | (data[4]&0xFF));
    cs.raw_y = signExtend13(((data[5]&0xFF)<<5) | ((data[6]&0xF8)>>3));

    // Акселерометр (13-бит, ±8g → m/s²)
    int16_t acc_raw_x = signExtend13(((data[6]&0x03)<<11) | (data[7]<<3) | ((data[8]&0xE0)>>5));
    int16_t acc_raw_y = signExtend13(((data[8]&0x1F)<<8) | (data[9]&0xFF));
    int16_t acc_raw_z = signExtend13(((data[10]&0xFF)<<5) | ((data[11]&0xF8)>>3));

    const float ACC_SCALE = 16.0f * 9.81f / 4095.0f;
    cs.acc_x = acc_raw_x * ACC_SCALE;
    cs.acc_y = acc_raw_y * ACC_SCALE;
    cs.acc_z = acc_raw_z * ACC_SCALE;

    // Трекпад
    cs.trackpad_x = ((data[16]&0x1F)<<3) | ((data[17]&0xE0)>>5);
    cs.trackpad_y = ((data[17]&0x1F)<<3) | ((data[18]&0xE0)>>5);

    // Кнопки
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

// ─── ВСПОМОГАТЕЛЬНОЕ: абсолютный yaw ───────────────────────────────────
// Yaw = вращение вокруг МИРОВОЙ вертикали, измеряется через
// проекцию worldUp на плоскость ⟂ forward.
float computeYawAbsolute(Vec3 forward, Vec3 up) {
    Vec3 worldUp = {0, 1, 0};
    float d = dotVec(worldUp, forward);
    Vec3 up_zero = {
        worldUp.x - forward.x * d,
        worldUp.y - forward.y * d,
        worldUp.z - forward.z * d
    };
    up_zero = normalizeVec(up_zero);

    Vec3 c = crossVec(up, up_zero);
    float sinYaw = dotVec(forward, c);
    float cosYaw = dotVec(up, up_zero);
    return atan2f(sinYaw, cosYaw) * 180.0f / M_PI;
}

// ─── УГЛЫ: ПРЯМО ОТ ОРИЕНТАЦИИ ────────────────────────────────────────
Euler getVectorAngles() {
    Euler result = {0, 0, 0};

    computeQuaternion();

    dbg.forward = rotateVector(cs.q, {0, 0, -1});
    dbg.up      = rotateVector(cs.q, {0, 1, 0});
    dbg.right   = rotateVector(cs.q, {1, 0, 0});

    // PITCH — наклон forward от горизонта
    result.pitch = atan2f(dbg.forward.y,
                          sqrtf(dbg.forward.x*dbg.forward.x +
                                dbg.forward.z*dbg.forward.z)) * 180.0f / M_PI;

    // YAW — вращение вокруг мировой вертикали (через проекцию worldUp)
    float yawAbsolute = computeYawAbsolute(dbg.forward, dbg.up);
    result.yaw = wrap180(yawAbsolute - cal.yaw_reference);

    // ROLL — вращение вокруг оси forward (atan2 по forward.x/z)
    result.roll = atan2f(dbg.forward.x, -dbg.forward.z) * 180.0f / M_PI;

    result.pitch = wrap180(result.pitch);
    result.roll  = wrap180(result.roll);
    result.yaw   = wrap180(result.yaw);

    return result;
}

// ─── КАЛИБРОВКА ────────────────────────────────────────────────────────
// yaw_reference фиксируется сразу при APP.
void startCalibration() {
    calSession.active  = true;
    calSession.samples = 0;

    // Захватываем текущий абсолютный yaw — той же формулой,
    // что используется в getVectorAngles.
    computeQuaternion();
    Vec3 forward = rotateVector(cs.q, {0, 0, -1});
    Vec3 up      = rotateVector(cs.q, {0, 1, 0});

    cal.yaw_reference = computeYawAbsolute(forward, up);
    cal.valid = false;   // станет true после завершения сессии

    ledBlink(1, 50, 50);   // 1 вспышка — старт
}

// Завершение сессии: cal.valid = true, фильтр выравнивается
// в текущее абсолютное положение, но по yaw — в 0.
void collectCalibrationSample() {
    if (!calSession.active) return;

    calSession.samples++;

    const uint16_t N = 32;
    if (calSession.samples < N) return;

    calSession.active = false;

    // Проверка неподвижности через акселерометр временно отключена.
    // bool magnitudeOK = ...;
    // bool stableOK    = ...;
    // if (!magnitudeOK || !stableOK) { ledBlink(5, 30, 30); return; }

    cal.valid = true;

    prefs.begin("fpv", false);
    prefs.putFloat("yawref", cal.yaw_reference);
    prefs.putBool ("cal",    true);
    prefs.end();

    // Выравниваем фильтр в текущее откалиброванное положение.
    // pitch/roll — абсолютные, yaw — 0.
    Euler current = getVectorAngles();
    filter.pitch = current.pitch;
    filter.roll  = current.roll;
    filter.yaw   = 0.0f;

    ledBlink(3, 100, 100);   // 3 вспышки — успех
}

void loadCalibration() {
    prefs.begin("fpv", true);
    if (prefs.getBool("cal", false)) {
        cal.yaw_reference = prefs.getFloat("yawref", 0.0f);
        cal.valid = true;
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

// ─── МИКСЕР ────────────────────────────────────────────────────────────
int16_t resolveSource(uint8_t src, Euler e) {
    switch (src) {
        case SRC_ROLL:
            return 1500 + (int16_t)(constrain(e.roll, -sens.roll_range, sens.roll_range)
                                    / sens.roll_range * 500);
        case SRC_PITCH:
            return 1500 + (int16_t)(constrain(e.pitch, -sens.pitch_range, sens.pitch_range)
                                    / sens.pitch_range * 500);
        case SRC_YAW:
            return 1500 + (int16_t)(constrain(e.yaw, -sens.yaw_range, sens.yaw_range)
                                    / sens.yaw_range * 500);
        case SRC_TP_X:
            return (cs.trackpad_x > 0 || cs.trackpad_y > 0)
                   ? map(cs.trackpad_x, 0, 255, 1000, 2000) : 1500;
        case SRC_TP_Y:
            return (cs.trackpad_x > 0 || cs.trackpad_y > 0)
                   ? map(cs.trackpad_y, 255, 0, 1000, 2000) : 1000;
        case SRC_NONE:
        default:
            return 1500;
    }
}

void convertToRC(Euler e) {
    float p = e.pitch, r = e.roll, y = e.yaw;
    if (fabsf(p) < sens.deadzone) p = 0;
    if (fabsf(r) < sens.deadzone) r = 0;
    if (fabsf(y) < sens.deadzone) y = 0;
    Euler e_dz = { p, r, y };

    rc.roll     = constrain(resolveSource(AXIS_X_SRC,  e_dz), 1000, 2000);
    rc.pitch    = constrain(resolveSource(AXIS_Y_SRC,  e_dz), 1000, 2000);
    rc.yaw      = constrain(resolveSource(AXIS_Z_SRC,  e_dz), 1000, 2000);
    rc.throttle = constrain(resolveSource(AXIS_RZ_SRC, e_dz), 1000, 2000);
}

// ─── ОТПРАВКА В USB HID ────────────────────────────────────────────────
void sendHIDReport() {
    GamepadReport report;

    int8_t hx  = map(rc.roll,     1000, 2000, -127, 127);
    int8_t hy  = map(rc.pitch,    1000, 2000, -127, 127);
    int8_t hz  = map(rc.yaw,      1000, 2000, -127, 127);
    int8_t hrz = map(rc.throttle, 1000, 2000, -127, 127);

#if INV_X
    hx = -hx;
#endif
#if INV_Y
    hy = -hy;
#endif
#if INV_Z
    hz = -hz;
#endif
#if INV_RZ
    hrz = -hrz;
#endif

    report.x  = hx;
    report.y  = hy;
    report.z  = hz;
    report.rz = hrz;

    report.buttons = 0;
    if (cs.home)  report.buttons |= (1 << 0);  // Arm
    if (cs.click) report.buttons |= (1 << 1);  // Mode
    if (cs.click) report.buttons |= (1 << 2);  // Click
    if (cs.app)   report.buttons |= (1 << 3);  // App

    if (TinyUSBDevice.mounted()) {
        usb_hid.sendReport(0, &report, sizeof(report));
    }
}

// ─── ОБРАБОТКА ─────────────────────────────────────────────────────────
void processData() {
    if (!gotData) return;
    gotData = false;

    // Во время сессии калибровки просто считаем пакеты.
    if (calSession.active) {
        collectCalibrationSample();
        return;
    }

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
    ledBlink(2, 50, 50);
}
void decreaseSensitivity() {
    sens.pitch_range = min(180.0f, sens.pitch_range * 1.15f);
    sens.roll_range  = min(180.0f, sens.roll_range  * 1.15f);
    sens.yaw_range   = min(180.0f, sens.yaw_range   * 1.15f);
    ledBlink(3, 50, 50);
}

// ─── BLE CALLBACKS ─────────────────────────────────────────────────────
class MyClientCB : public BLEClientCallbacks {
    void onConnect(BLEClient*)    { connected = true;  ledOn();  }
    void onDisconnect(BLEClient*) { connected = false; ledOff(); }
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

    if (!client->connect(targetDevice)) {
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
        delete targetDevice; targetDevice = nullptr;
        loadCalibration();
    } else {
        client->disconnect();
        delete targetDevice; targetDevice = nullptr;
    }
}

void startScan() {
    if (connected) return;
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

    // APP — старт калибровки (yaw reference захватывается сразу)
    if (cs.app && !prevApp && !calSession.active) {
        startCalibration();
    }
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