#include "config.h"
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <BLE2902.h>
#include <math.h>

// ═══════════════════════════════════════════════════════════════════════
//  ПИНЫ
// ═══════════════════════════════════════════════════════════════════════
#define BIND_PIN     13
#define LED_PIN      22
#define BOOT_BTN_PIN 0

#define LED_ON   LOW
#define LED_OFF  HIGH

// ═══════════════════════════════════════════════════════════════════════
//  ОТЛАДКА
// ═══════════════════════════════════════════════════════════════════════
#ifdef DEBUG
  #define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTF(...)  Serial.printf(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTF(...)
  #define DEBUG_PRINTLN(...)
#endif

// ═══════════════════════════════════════════════════════════════════════
//  RC / ESP-NOW
// ═══════════════════════════════════════════════════════════════════════
#define RC_CHANNELS 8

typedef struct {
    uint16_t channels[RC_CHANNELS];
    uint8_t  connected;
    uint32_t seq;
} struct_message;

struct_message txData;
esp_now_peer_info_t peerInfo;

uint8_t receiverMac[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
bool isBindMode  = false;
bool bindSuccess = false;
Preferences prefs;

volatile bool sendFinished = true;
uint32_t sequence = 0;

// ═══════════════════════════════════════════════════════════════════════
//  BLE DAYDREAM
// ═══════════════════════════════════════════════════════════════════════
static BLEUUID SERVICE_UUID("0000fe55-0000-1000-8000-00805f9b34fb");
static BLEUUID CHAR_UUID   ("00000001-1000-1000-8000-00805f9b34fb");
static BLEUUID CCCD_UUID   ("00002902-0000-1000-8000-00805f9b34fb");

// ─── КНОПКИ DAYDREAM ───────────────────────────────────────────────────
static const uint8_t BTN_CLICK    = 0x01;
static const uint8_t BTN_HOME     = 0x02;
static const uint8_t BTN_APP      = 0x04;
static const uint8_t BTN_VOL_DOWN = 0x08;
static const uint8_t BTN_VOL_UP   = 0x10;

// ─── СТРУКТУРЫ ─────────────────────────────────────────────────────────
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

// ─── СОСТОЯНИЕ КОНТРОЛЛЕРА ─────────────────────────────────────────────
struct ControllerState {
    int16_t raw_x, raw_y, raw_z;
    float acc_x, acc_y, acc_z;
    Quat q;
    float angle_rad;
    float axis_x, axis_y, axis_z;
    uint8_t trackpad_x, trackpad_y;
    bool click, home, app, vd, vu;
} cs;

// ─── RC КАНАЛЫ ─────────────────────────────────────────────────────────
struct RCChannels {
    int16_t pitch, roll, yaw, throttle;
} rc;

// ─── КАЛИБРОВКА ────────────────────────────────────────────────────────
struct Calibration {
    float yaw_reference = 0.0f;
    bool valid = false;
} cal;

struct CalibrationSession {
    bool active = false;
    uint16_t samples = 0;
} calSession;

// ─── ЧУВСТВИТЕЛЬНОСТЬ ──────────────────────────────────────────────────
#define PITCH_RANGE_DEFAULT  45.0f
#define ROLL_RANGE_DEFAULT   45.0f
#define YAW_RANGE_DEFAULT    60.0f
#define DEADZONE_DEFAULT     1.0f

struct Sensitivity {
    float pitch_range = PITCH_RANGE_DEFAULT;
    float roll_range  = ROLL_RANGE_DEFAULT;
    float yaw_range   = YAW_RANGE_DEFAULT;
    float deadzone    = DEADZONE_DEFAULT;
} sens;

// ─── ФИЛЬТР ────────────────────────────────────────────────────────────
struct AngleFilter {
    float pitch = 0, roll = 0, yaw = 0;
    float alpha = 0.3f;
} filter;

// ─── ОТЛАДКА ВЕКТОРОВ ──────────────────────────────────────────────────
struct DebugVectors {
    Vec3 forward, up, right, up_zero;
} dbg;

bool connected = false;
bool gotData   = false;
BLEClient* client = nullptr;
BLEAdvertisedDevice* targetDevice = nullptr;

// ═══════════════════════════════════════════════════════════════════════
//  LED
// ═══════════════════════════════════════════════════════════════════════
void ledOff() { digitalWrite(LED_PIN, LED_OFF); }
void ledOn()  { digitalWrite(LED_PIN, LED_ON);  }
void ledBlink(int count, int onMs, int offMs) {
    for (int i = 0; i < count; i++) { ledOn(); delay(onMs); ledOff(); delay(offMs); }
}

// ═══════════════════════════════════════════════════════════════════════
//  ПАРСИНГ ПАКЕТА DAYDREAM
// ═══════════════════════════════════════════════════════════════════════
int16_t signExtend13(int16_t v) {
    if (v & 0x1000) v |= 0xE000;
    return v;
}

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

// ═══════════════════════════════════════════════════════════════════════
//  AXIS-ANGLE → QUATERNION
// ═══════════════════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════════════════
//  КВАТЕРНИОНЫ
// ═══════════════════════════════════════════════════════════════════════
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

// ─── АБСОЛЮТНЫЙ YAW (через проекцию мировой вертикали) ─────────────────
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

// ─── УГЛЫ ОТ ОРИЕНТАЦИИ ────────────────────────────────────────────────
Euler getVectorAngles() {
    Euler result = {0, 0, 0};

    computeQuaternion();

    dbg.forward = rotateVector(cs.q, {0, 0, -1});
    dbg.up      = rotateVector(cs.q, {0, 1, 0});
    dbg.right   = rotateVector(cs.q, {1, 0, 0});

    result.pitch = atan2f(dbg.forward.y,
                          sqrtf(dbg.forward.x*dbg.forward.x +
                                dbg.forward.z*dbg.forward.z)) * 180.0f / M_PI;

    float yawAbsolute = computeYawAbsolute(dbg.forward, dbg.up);
    result.yaw = wrap180(yawAbsolute - cal.yaw_reference);

    result.roll = atan2f(dbg.forward.x, -dbg.forward.z) * 180.0f / M_PI;

    result.pitch = wrap180(result.pitch);
    result.roll  = wrap180(result.roll);
    result.yaw   = wrap180(result.yaw);

    return result;
}

// ═══════════════════════════════════════════════════════════════════════
//  КАЛИБРОВКА
// ═══════════════════════════════════════════════════════════════════════
void startCalibration() {
    calSession.active  = true;
    calSession.samples = 0;

    computeQuaternion();
    Vec3 forward = rotateVector(cs.q, {0, 0, -1});
    Vec3 up      = rotateVector(cs.q, {0, 1, 0});

    cal.yaw_reference = computeYawAbsolute(forward, up);
    cal.valid = false;

    ledBlink(1, 50, 50);
}

void collectCalibrationSample() {
    if (!calSession.active) return;

    calSession.samples++;

    const uint16_t N = 32;
    if (calSession.samples < N) return;

    calSession.active = false;
    cal.valid = true;

    prefs.begin("fpv", false);
    prefs.putFloat("yawref", cal.yaw_reference);
    prefs.putBool ("cal",    true);
    prefs.end();

    Euler current = getVectorAngles();
    filter.pitch = current.pitch;
    filter.roll  = current.roll;
    filter.yaw   = 0.0f;

    ledBlink(3, 100, 100);
}

void loadCalibration() {
    prefs.begin("fpv", true);
    if (prefs.getBool("cal", false)) {
        cal.yaw_reference = prefs.getFloat("yawref", 0.0f);
        cal.valid = true;
        DEBUG_PRINTF("Calibration loaded: yawref=%.2f\n", cal.yaw_reference);
    } else {
        DEBUG_PRINTLN("No calibration in NVS");
    }
    prefs.end();
}

// ═══════════════════════════════════════════════════════════════════════
//  ФИЛЬТР
// ═══════════════════════════════════════════════════════════════════════
float angleLerp(float cur, float tgt, float a) {
    return cur + wrap180(tgt - cur) * a;
}
Euler filterEuler(Euler raw) {
    filter.pitch = angleLerp(filter.pitch, raw.pitch, filter.alpha);
    filter.roll  = angleLerp(filter.roll,  raw.roll,  filter.alpha);
    filter.yaw   = angleLerp(filter.yaw,   raw.yaw,   filter.alpha);
    return { filter.pitch, filter.roll, filter.yaw };
}

// ═══════════════════════════════════════════════════════════════════════
//  МИКСЕР (из джойстика) → RC КАНАЛЫ
// ═══════════════════════════════════════════════════════════════════════
#define SRC_NONE   0
#define SRC_ROLL   1
#define SRC_PITCH  2
#define SRC_YAW    3
#define SRC_TP_X   4
#define SRC_TP_Y   5

#define AXIS_X_SRC   SRC_ROLL
#define AXIS_Y_SRC   SRC_PITCH
#define AXIS_Z_SRC   SRC_YAW
#define AXIS_RZ_SRC  SRC_TP_Y

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

// ═══════════════════════════════════════════════════════════════════════
//  ЧУВСТВИТЕЛЬНОСТЬ
// ═══════════════════════════════════════════════════════════════════════
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

// ═══════════════════════════════════════════════════════════════════════
//  УПАКОВКА КАНАЛОВ + МИКШЕР + ОТПРАВКА ESP-NOW
// ═══════════════════════════════════════════════════════════════════════
void packChannelsAndSend() {
    if (!sendFinished) return;

    // 1. Раскладываем RC в каналы
    txData.channels[0] = rc.roll;
    txData.channels[1] = rc.pitch;
    txData.channels[2] = rc.throttle;
    txData.channels[3] = rc.yaw;
    txData.channels[4] = cs.click ? PULSE_MAX : PULSE_MIN;
    txData.channels[5] = PULSE_MIN;
    txData.channels[6] = PULSE_MIN;
    txData.channels[7] = PULSE_MIN;

    // 2. Миксер
    MixerData input;
    MixerData output;
    for (int i = 0; i < RC_CHANNELS; i++) input.channels[i] = txData.channels[i];
    applyMixer(&input, &output);
    for (int i = 0; i < RC_CHANNELS; i++) txData.channels[i] = output.channels[i];

    #ifdef DEBUG
      printMixerInfo(&output);
    #endif

    // 3. Метаданные
    txData.connected = 1;
    txData.seq = sequence++;

    // 4. Отправка
    sendFinished = false;
    esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&txData, sizeof(txData));
    if (result != ESP_OK) {
        sendFinished = true;
        DEBUG_PRINTF("esp_now_send error: %d\n", result);
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  ESP-NOW CALLBACKS
// ═══════════════════════════════════════════════════════════════════════
#if ESP_IDF_VERSION_MAJOR >= 5
  void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
  void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#endif
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
        for (int i = 0; i < 6; i++) DEBUG_PRINTF("%02X%s", receiverMac[i], (i<5)?":":"");
        DEBUG_PRINTLN();

        prefs.begin("rx_conf", false);
        prefs.putBytes("mac", receiverMac, 6);
        prefs.end();

        bindSuccess = true;
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  BLE CALLBACKS / СКАН / ПОДКЛЮЧЕНИЕ
// ═══════════════════════════════════════════════════════════════════════
class MyClientCB : public BLEClientCallbacks {
    void onConnect(BLEClient*)    { connected = true;  ledOn();
                                     DEBUG_PRINTLN("BLE connected"); }
    void onDisconnect(BLEClient*) { connected = false; ledOff();
                                     DEBUG_PRINTLN("BLE disconnected"); }
};

class MyScanCB : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice dev) {
        if (dev.haveServiceUUID() && dev.getServiceUUID().equals(SERVICE_UUID)) {
            targetDevice = new BLEAdvertisedDevice(dev);
            BLEDevice::getScan()->stop();
            DEBUG_PRINTLN("Daydream found");
        }
    }
};

void connectToDevice() {
    if (!targetDevice || connected) return;

    if (client) { delete client; client = nullptr; }
    client = BLEDevice::createClient();
    client->setClientCallbacks(new MyClientCB());

    if (!client->connect(targetDevice)) {
        DEBUG_PRINTLN("BLE connect failed");
        delete targetDevice; targetDevice = nullptr;
        return;
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
        DEBUG_PRINTLN("BLE notify enabled");
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

// ═══════════════════════════════════════════════════════════════════════
//  ОБРАБОТКА ДАННЫХ
// ═══════════════════════════════════════════════════════════════════════
void processData() {
    if (!gotData) return;
    gotData = false;

    if (calSession.active) {
        collectCalibrationSample();
        return;
    }

    if (cal.valid) {
        Euler raw = getVectorAngles();
        Euler flt = filterEuler(raw);
        convertToRC(flt);
        packChannelsAndSend();
    } else {
        computeQuaternion();
    }
}

// ═══════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);

    #ifdef DEBUG
      Serial.println("=== DAYDREAM → ESP-NOW TRANSMITTER ===");
      Serial.printf("Channels: %d, Pulse: %d-%d us\n", RC_CHANNELS, PULSE_MIN, PULSE_MAX);
    #endif

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_OFF);

    pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

    // ─── BIND MODE ─────────────────────────────────────────────────────
    pinMode(BIND_PIN, INPUT_PULLUP);
    delay(50);
    if (digitalRead(BIND_PIN) == LOW) {
        isBindMode = true;
        digitalWrite(LED_PIN, LED_ON);
        DEBUG_PRINTLN("BIND MODE ON (Listening for RX beacon)");
    } else {
        prefs.begin("rx_conf", true);
        if (prefs.isKey("mac")) {
            prefs.getBytes("mac", receiverMac, 6);
            #ifdef DEBUG
              DEBUG_PRINT("Loaded MAC from NVS: ");
              for (int i = 0; i < 6; i++) DEBUG_PRINTF("%02X%s", receiverMac[i], (i<5)?":":"");
              DEBUG_PRINTLN();
            #endif
        } else {
            DEBUG_PRINTLN("No MAC in NVS, waiting for bind");
        }
        prefs.end();
    }

    // ─── WIFI / ESP-NOW ────────────────────────────────────────────────
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

    // ─── BLE (только в рабочем режиме) ─────────────────────────────────
    if (!isBindMode) {
        BLEDevice::init("DaydreamFPV-TX");
        BLEDevice::setPower(ESP_PWR_LVL_P9);
        loadCalibration();
        startScan();
    }

    #ifdef DEBUG
      Serial.println("READY");
    #endif
}

// ═══════════════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════════════
void loop() {
    static unsigned long lastBlink = 0;
    static unsigned long lastReconnect = 0;
    static bool prevApp = false, prevVu = false, prevVd = false;

    // ─── BIND MODE ─────────────────────────────────────────────────────
    if (isBindMode) {
        if (bindSuccess) {
            digitalWrite(LED_PIN, (millis() / 200) % 2);
        }
        return;
    }

    // ─── LED ожидания BLE ──────────────────────────────────────────────
    if (!connected && millis() - lastBlink > 2000) {
        lastBlink = millis();
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }

    // ─── КНОПКИ DAYDREAM ───────────────────────────────────────────────
    if (cs.app && !prevApp && !calSession.active) {
        startCalibration();
    }
    prevApp = cs.app;

    if (cs.vu && !prevVu) increaseSensitivity();
    prevVu = cs.vu;

    if (cs.vd && !prevVd) decreaseSensitivity();
    prevVd = cs.vd;

    // ─── ОБРАБОТКА ДАННЫХ ──────────────────────────────────────────────
    if (gotData) processData();

    // ─── ПОДКЛЮЧЕНИЕ ───────────────────────────────────────────────────
    if (targetDevice && !connected) connectToDevice();

    if (!connected && !targetDevice && client) {
        if (millis() - lastReconnect > 5000) {
            lastReconnect = millis();
            startScan();
        }
    }

    // ─── BOOT: СБРОС КАЛИБРОВКИ + РЕКОННЕКТ ───────────────────────────
    if (digitalRead(BOOT_BTN_PIN) == LOW) {
        static unsigned long last = 0;
        if (millis() - last > 300) {
            last = millis();
            DEBUG_PRINTLN("BOOT: reset calibration");
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