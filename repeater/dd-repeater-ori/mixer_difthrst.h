// mixer_difthrst.h
// Дифференциальная тяга: вход AETR + AUX + Click, выход roll/pitch/motorL/motorR + AUX + Lock
//
// Вход:
//   CH1 (channels[0]) = Roll     (A)
//   CH2 (channels[1]) = Pitch    (E)
//   CH3 (channels[2]) = Throttle (T)
//   CH4 (channels[3]) = Yaw      (R)
//   CH5 (channels[4]) = AUX      (тачпад X, живой)
//   CH6 (channels[5]) = Click    (моментальный: PULSE_MAX = нажат)
//   CH7..CH8          = резерв
//
// Выход:
//   CH1 (channels[0]) = Roll     (проброс 1:1)
//   CH2 (channels[1]) = Pitch    (проброс 1:1)
//   CH3 (channels[2]) = MotorLeft
//   CH4 (channels[3]) = MotorRight
//   CH5 (channels[4]) = AUX      (проброс 1:1)
//   CH6 (channels[5]) = Throttle Lock (залипание: PULSE_MAX = заблокирован)
//   CH7..CH8          = PULSE_MIN
//
// Дифференциал:
//   yawDiff = (yaw - PULSE_CENTER) * DIFTHRST_SCALE / 100
//   yaw > 0  → правый мотор быстрее, левый медленнее
//   yaw < 0  → левый мотор быстрее, правый медленнее
//   при упоре одного мотора в PULSE_MAX — второй компенсируется
//
// Блокировка throttle (click):
//   Долгое нажатие click (>= CLICK_HOLD_MS) → toggle блокировки.
//   При блокировке throttle фиксируется на текущем значении.
//   Дифференциал продолжает работать от зафиксированного throttle.
//   Повторное долгое нажатие — разблокировка, throttle мгновенно
//   переходит к текущему значению трекпада.

#ifndef MIXER_DIFTHRST_H
#define MIXER_DIFTHRST_H

#include <stdint.h>
#include <Arduino.h>

#define PULSE_MIN 1000
#define PULSE_MAX 2000
#define PULSE_CENTER 1500
#define MAX_CHANNELS 8

// === НАСТРОЙКИ ===
#define DIFTHRST_SCALE 25
#define MOTOR_PULSE_MIN 1000
#define MOTOR_PULSE_MAX 1400

#define CLICK_HOLD_MS 1000   // длительность удержания click для toggle

typedef struct {
    uint16_t channels[MAX_CHANNELS];
} MixerData;

// ─── СОСТОЯНИЕ БЛОКИРОВКИ ──────────────────────────────────────────────
static bool     throttleLocked   = false;
static int16_t  lockedThrottle   = PULSE_MIN;
static bool     prevClickState   = false;
static bool     toggleFired      = false;
static uint32_t clickPressTime   = 0;

void applyMixer(MixerData *input, MixerData *output) {
    // === ПРОБРОС ROLL / PITCH / AUX (1:1) ===
    output->channels[0] = input->channels[0];
    output->channels[1] = input->channels[1];
    output->channels[4] = input->channels[4];   // AUX

    // === ОБРАБОТКА CLICK (CH6) — toggle по долгому нажатию ===
    bool clickNow = (input->channels[5] == PULSE_MAX);

    // Фронт нажатия
    if (clickNow && !prevClickState) {
        clickPressTime = millis();
        toggleFired    = false;
    }

    // Удержание дольше порога — toggle (один раз за нажатие)
    if (clickNow && !toggleFired &&
        (millis() - clickPressTime >= CLICK_HOLD_MS)) {

        throttleLocked = !throttleLocked;
        toggleFired    = true;

        if (throttleLocked) {
            lockedThrottle = input->channels[2];
            #ifdef DEBUG
                Serial.printf("Throttle LOCKED at %d\n", lockedThrottle);
            #endif
        } else {
            #ifdef DEBUG
                Serial.println("Throttle UNLOCKED");
            #endif
        }
    }

    // Спад (отпускание) — сбрасываем флаги
    if (!clickNow && prevClickState) {
        toggleFired = false;
    }

    prevClickState = clickNow;

    // === ВЫБОР THROTTLE ===
    int16_t throttle = throttleLocked ? lockedThrottle : input->channels[2];

    if (!throttleLocked && throttle == PULSE_MIN) {
        // ─── СТОП: моторы 1000, дифф. тяга выключена ───────────────────
        output->channels[2] = MOTOR_PULSE_MIN;
        output->channels[3] = MOTOR_PULSE_MIN;
    } else {
        // ─── ДИФФЕРЕНЦИАЛЬНАЯ ТЯГА (в полном диапазоне 1000-2000) ──────
        int16_t yaw = input->channels[3] - PULSE_CENTER;
        yaw = yaw * DIFTHRST_SCALE / 100;

        int16_t motorLeft  = throttle;
        int16_t motorRight = throttle;

        if (yaw > 0) {
            motorRight = throttle + yaw;
            if (motorRight > PULSE_MAX) {
                int16_t excess = motorRight - PULSE_MAX;
                motorRight = PULSE_MAX;
                motorLeft = throttle - excess;
                if (motorLeft < PULSE_MIN) motorLeft = PULSE_MIN;
            }
        } else if (yaw < 0) {
            motorLeft = throttle - yaw;
            if (motorLeft > PULSE_MAX) {
                int16_t excess = motorLeft - PULSE_MAX;
                motorLeft = PULSE_MAX;
                motorRight = throttle - excess;
                if (motorRight < PULSE_MIN) motorRight = PULSE_MIN;
            }
        }

        motorLeft  = constrain(motorLeft,  PULSE_MIN, PULSE_MAX);
        motorRight = constrain(motorRight, PULSE_MIN, PULSE_MAX);

        // ─── СЖАТИЕ В МОТОРНЫЙ ДИАПАЗОН (в самом конце) ────────────────
        output->channels[2] = MOTOR_PULSE_MIN +
            (int32_t)(motorLeft  - PULSE_MIN) * (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN) / (PULSE_MAX - PULSE_MIN);
        output->channels[3] = MOTOR_PULSE_MIN +
            (int32_t)(motorRight - PULSE_MIN) * (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN) / (PULSE_MAX - PULSE_MIN);
    }

    // === ВЫХОД CH6 С ЗАЛИПАНИЕМ ===
    output->channels[5] = throttleLocked ? PULSE_MAX : PULSE_MIN;

    // === РЕЗЕРВ ===
    output->channels[6] = PULSE_MIN;
    output->channels[7] = PULSE_MIN;
}

void printMixerInfo(MixerData *data) {
    #ifdef DEBUG
        Serial.printf("DIFTHRST: CH1(roll)=%4d CH2(pitch)=%4d CH3(motorL)=%4d CH4(motorR)=%4d CH5(aux)=%4d CH6(lock)=%4d\n",
                      data->channels[0], data->channels[1],
                      data->channels[2], data->channels[3],
                      data->channels[4], data->channels[5]);
    #endif
}

#endif // MIXER_DIFTHRST_H