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
#define MOTOR_PULSE_MAX 1400
#define TRIM_STEP 5
#define TRIM_MIN -400
#define TRIM_MAX 400

typedef struct {
    uint16_t channels[MAX_CHANNELS];
} MixerData;

static int16_t trimYaw = 0;      // Трим для рыскания (CH2)
static int16_t trimElevator = 0; // Трим для элеватора (CH1)

uint16_t scaleMotorPulse(uint16_t pulse) {
    if (pulse <= PULSE_MIN) return MOTOR_PULSE_MIN;
    if (pulse >= PULSE_MAX) return MOTOR_PULSE_MAX;
    return map(pulse, PULSE_MIN, PULSE_MAX, MOTOR_PULSE_MIN, MOTOR_PULSE_MAX);
}

void applyMixer(MixerData *input, MixerData *output) {
    // === ТРИММИРОВАНИЕ ===
    // CH5 (кнопка 0x0001) → увеличиваем трим рыскания
    if (input->channels[4] == PULSE_MAX) {
        trimYaw += TRIM_STEP;
        if (trimYaw > TRIM_MAX) trimYaw = TRIM_MAX;
        #ifdef DEBUG
            Serial.printf("Trim Yaw UP: %d\n", trimYaw);
        #endif
    }
    
    // CH8 (кнопка 0x0008) → уменьшаем трим рыскания
    if (input->channels[7] == PULSE_MAX) {
        trimYaw -= TRIM_STEP;
        if (trimYaw < TRIM_MIN) trimYaw = TRIM_MIN;
        #ifdef DEBUG
            Serial.printf("Trim Yaw DOWN: %d\n", trimYaw);
        #endif
    }
    
    // CH6 (кнопка 0x0002) → увеличиваем трим элеватора
    if (input->channels[5] == PULSE_MAX) {
        trimElevator += TRIM_STEP;
        if (trimElevator > TRIM_MAX) trimElevator = TRIM_MAX;
        #ifdef DEBUG
            Serial.printf("Trim Elevator UP: %d\n", trimElevator);
        #endif
    }
    
    // CH7 (кнопка 0x0004) → уменьшаем трим элеватора
    if (input->channels[6] == PULSE_MAX) {
        trimElevator -= TRIM_STEP;
        if (trimElevator < TRIM_MIN) trimElevator = TRIM_MIN;
        #ifdef DEBUG
            Serial.printf("Trim Elevator DOWN: %d\n", trimElevator);
        #endif
    }
    
    // === КАНАЛ 1 (Elevator) с тримом ===
    int16_t elevator = input->channels[0] - PULSE_CENTER + trimElevator;
    output->channels[0] = constrain(elevator + PULSE_CENTER, PULSE_MIN, PULSE_MAX);
    
    // === КАНАЛ 2 (Ailerons) - резерв ===
    output->channels[1] = PULSE_CENTER;
    
    // === КАНАЛЫ 5-8 (для приемника) ===
    output->channels[4] = PULSE_MIN;  // CH5 = 0
    output->channels[5] = PULSE_MIN;  // CH6 = 0
    output->channels[6] = PULSE_MIN;  // CH7 = 0
    output->channels[7] = PULSE_MIN;  // CH8 = 0
    
    // === ДИФФЕРЕНЦИАЛЬНАЯ ТЯГА ===
    int16_t throttle = input->channels[2] - PULSE_CENTER;
    
    // Трим применяется к yaw
    int16_t yaw = input->channels[1] - PULSE_CENTER + trimYaw;
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
        Serial.printf("Trim Yaw: %d, Trim Elev: %d, Output: CH1=%4d CH2=%4d CH3=%4d CH4=%4d\n",
                      trimYaw, trimElevator,
                      data->channels[0], data->channels[1], 
                      data->channels[2], data->channels[3]);
    #endif
}

#endif // MIXER_DIFTHRST_H