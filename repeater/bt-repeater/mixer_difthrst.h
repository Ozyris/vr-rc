#ifndef MIXER_DIFTHRST_H
#define MIXER_DIFTHRST_H

#include <stdint.h>

#define PULSE_MIN 1000
#define PULSE_MAX 2000
#define PULSE_CENTER 1500
#define MAX_CHANNELS 8

// === НАСТРОЙКИ ===
#define DIFTHRST_SCALE 50
#define MOTOR_PULSE_MIN 1000
#define MOTOR_PULSE_MAX 1300
#define TRIM_STEP 5
#define TRIM_MIN -400
#define TRIM_MAX 400

typedef struct {
    uint16_t channels[MAX_CHANNELS];
} MixerData;

static int16_t trimValue = 0;

uint16_t scaleMotorPulse(uint16_t pulse) {
    if (pulse <= PULSE_MIN) return MOTOR_PULSE_MIN;
    if (pulse >= PULSE_MAX) return MOTOR_PULSE_MAX;
    return map(pulse, PULSE_MIN, PULSE_MAX, MOTOR_PULSE_MIN, MOTOR_PULSE_MAX);
}

void applyMixer(MixerData *input, MixerData *output) {
    // === КАНАЛ 1 (Elevator) ===
    output->channels[0] = input->channels[0];
    
    // === КАНАЛ 2 (Ailerons) - резерв ===
    output->channels[1] = PULSE_CENTER;
    
    // === ТРИММИРОВАНИЕ ===
    // CH5 (кнопка 0x0001) → увеличиваем трим
    if (input->channels[4] == PULSE_MAX) {
        trimValue += TRIM_STEP;
        if (trimValue > TRIM_MAX) trimValue = TRIM_MAX;
        #ifdef DEBUG
            Serial.printf("Trim UP: %d\n", trimValue);
        #endif
    }
    
    // CH8 (кнопка 0x0008) → уменьшаем трим
    if (input->channels[7] == PULSE_MAX) {
        trimValue -= TRIM_STEP;
        if (trimValue < TRIM_MIN) trimValue = TRIM_MIN;
        #ifdef DEBUG
            Serial.printf("Trim DOWN: %d\n", trimValue);
        #endif
    }
    
    // === КАНАЛЫ 5-8 (для приемника) ===
    output->channels[4] = input->channels[5];  // CH5 = кнопка 0x0002
    output->channels[5] = input->channels[6];  // CH6 = кнопка 0x0004
    output->channels[6] = PULSE_MIN;           // CH7 = 0
    output->channels[7] = PULSE_MIN;           // CH8 = 0
    
    // === ДИФФЕРЕНЦИАЛЬНАЯ ТЯГА ===
    int16_t throttle = input->channels[2] - PULSE_CENTER;
    
    // === ТРИМ ПРИМЕНЯЕТСЯ СРАЗУ К YAW ===
    int16_t yaw = input->channels[1] - PULSE_CENTER + trimValue;
    
    // Масштабируем yaw
    yaw = yaw * DIFTHRST_SCALE / 100;
    
    // Вычисляем моторы
    int16_t motorLeft = throttle - yaw;
    int16_t motorRight = throttle + yaw;
    
    uint16_t rawLeft = constrain(motorLeft + PULSE_CENTER, PULSE_MIN, PULSE_MAX);
    uint16_t rawRight = constrain(motorRight + PULSE_CENTER, PULSE_MIN, PULSE_MAX);
    
    output->channels[2] = scaleMotorPulse(rawLeft);
    output->channels[3] = scaleMotorPulse(rawRight);
}

void printMixerInfo(MixerData *data) {
    #ifdef DEBUG
        Serial.printf("Output: CH1=%4d CH2=%4d CH3=%4d CH4=%4d CH5=%4d CH6=%4d CH7=%4d CH8=%4d\n",
                      data->channels[0], data->channels[1], 
                      data->channels[2], data->channels[3],
                      data->channels[4], data->channels[5],
                      data->channels[6], data->channels[7]);
    #endif
}

#endif // MIXER_DIFTHRST_H