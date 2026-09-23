// mixer_difthrst.h
// Дифференциальная тяга: вход AETR, выход roll/pitch/motorL/motorR
//
// Вход:
//   CH1 (channels[0]) = Roll     (A)
//   CH2 (channels[1]) = Pitch    (E)
//   CH3 (channels[2]) = Throttle (T)
//   CH4 (channels[3]) = Yaw      (R)
//   CH5..CH8          = резерв (игнорируются)
//
// Выход:
//   CH1 (channels[0]) = Roll     (проброс 1:1)
//   CH2 (channels[1]) = Pitch    (проброс 1:1)
//   CH3 (channels[2]) = MotorLeft
//   CH4 (channels[3]) = MotorRight
//   CH5..CH8          = PULSE_MIN
//
// Дифференциал:
//   yawDiff = (yaw - PULSE_CENTER) * DIFTHRST_SCALE / 100
//   yaw > 0  → правый мотор быстрее, левый медленнее
//   yaw < 0  → левый мотор быстрее, правый медленнее
//   при упоре одного мотора в MOTOR_PULSE_MAX — второй компенсируется

#ifndef MIXER_DIFTHRST_H
#define MIXER_DIFTHRST_H

#include <stdint.h>

#define PULSE_MIN 1000
#define PULSE_MAX 2000
#define PULSE_CENTER 1500
#define MAX_CHANNELS 8

// === НАСТРОЙКИ ===
#define DIFTHRST_SCALE 25
#define MOTOR_PULSE_MIN 1000
#define MOTOR_PULSE_MAX 1400

typedef struct {
    uint16_t channels[MAX_CHANNELS];
} MixerData;

void applyMixer(MixerData *input, MixerData *output) {
    // === ПРОБРОС ROLL / PITCH (1:1) ===
    output->channels[0] = input->channels[0];
    output->channels[1] = input->channels[1];

    // === ДИФФЕРЕНЦИАЛЬНАЯ ТЯГА (в полном диапазоне 1000-2000) ===
    int16_t throttle = input->channels[2];

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

    // Ограничение в полном диапазоне (страховка)
    motorLeft  = constrain(motorLeft,  PULSE_MIN, PULSE_MAX);
    motorRight = constrain(motorRight, PULSE_MIN, PULSE_MAX);

    // === СЖАТИЕ В МОТОРНЫЙ ДИАПАЗОН (в самом конце) ===
    // 1000..2000 → 1000..1400, коэффициент 0.4
    output->channels[2] = MOTOR_PULSE_MIN +
        (int32_t)(motorLeft  - PULSE_MIN) * (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN) / (PULSE_MAX - PULSE_MIN);
    output->channels[3] = MOTOR_PULSE_MIN +
        (int32_t)(motorRight - PULSE_MIN) * (MOTOR_PULSE_MAX - MOTOR_PULSE_MIN) / (PULSE_MAX - PULSE_MIN);

    // === РЕЗЕРВ ===
    output->channels[4] = PULSE_MIN;
    output->channels[5] = PULSE_MIN;
    output->channels[6] = PULSE_MIN;
    output->channels[7] = PULSE_MIN;
}

void printMixerInfo(MixerData *data) {
    #ifdef DEBUG
        Serial.printf("DIFTHRST: CH1(roll)=%4d CH2(pitch)=%4d CH3(motorL)=%4d CH4(motorR)=%4d\n",
                      data->channels[0], data->channels[1],
                      data->channels[2], data->channels[3]);
    #endif
}

#endif // MIXER_DIFTHRST_H