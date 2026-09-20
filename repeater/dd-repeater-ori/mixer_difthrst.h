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
#define TRIM_STEP 5
#define TRIM_MIN -400
#define TRIM_MAX 400

typedef struct {
    uint16_t channels[MAX_CHANNELS];
} MixerData;

static int16_t trimYaw = 0;      // Трим для рыскания (CH2)
static int16_t trimElevator = 0; // Трим для элеватора (CH1)

void applyMixer(MixerData *input, MixerData *output) {
    // === ТРИММИРОВАНИЕ ===
    if (input->channels[4] == PULSE_MAX) {
        trimYaw += TRIM_STEP;
        if (trimYaw > TRIM_MAX) trimYaw = TRIM_MAX;
        #ifdef DEBUG
            Serial.printf("Trim Yaw UP: %d\n", trimYaw);
        #endif
    }
    
    if (input->channels[7] == PULSE_MAX) {
        trimYaw -= TRIM_STEP;
        if (trimYaw < TRIM_MIN) trimYaw = TRIM_MIN;
        #ifdef DEBUG
            Serial.printf("Trim Yaw DOWN: %d\n", trimYaw);
        #endif
    }
    
    if (input->channels[5] == PULSE_MAX) {
        trimElevator += TRIM_STEP;
        if (trimElevator > TRIM_MAX) trimElevator = TRIM_MAX;
        #ifdef DEBUG
            Serial.printf("Trim Elevator UP: %d\n", trimElevator);
        #endif
    }
    
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
    output->channels[4] = PULSE_MIN;
    output->channels[5] = PULSE_MIN;
    output->channels[6] = PULSE_MIN;
    output->channels[7] = PULSE_MIN;
    
    // === ДИФФЕРЕНЦИАЛЬНАЯ ТЯГА ===
    int16_t throttle = input->channels[2];
    
    int16_t yaw = input->channels[1] - PULSE_CENTER + trimYaw;
    yaw = yaw * DIFTHRST_SCALE / 100;
    
    int16_t motorLeft = throttle;
    int16_t motorRight = throttle;
    
    if (yaw > 0) {
        motorRight = throttle + yaw;
        if (motorRight > MOTOR_PULSE_MAX) {
            int16_t excess = motorRight - MOTOR_PULSE_MAX;
            motorRight = MOTOR_PULSE_MAX;
            motorLeft = throttle - excess;
            if (motorLeft < MOTOR_PULSE_MIN) motorLeft = MOTOR_PULSE_MIN;
        }
    } else if (yaw < 0) {
        motorLeft = throttle - yaw;
        if (motorLeft > MOTOR_PULSE_MAX) {
            int16_t excess = motorLeft - MOTOR_PULSE_MAX;
            motorLeft = MOTOR_PULSE_MAX;
            motorRight = throttle - excess;
            if (motorRight < MOTOR_PULSE_MIN) motorRight = MOTOR_PULSE_MIN;
        }
    }
    
    // === ПРОСТО ОГРАНИЧИВАЕМ (без масштабирования) ===
    output->channels[2] = constrain(motorLeft, MOTOR_PULSE_MIN, MOTOR_PULSE_MAX);
    output->channels[3] = constrain(motorRight, MOTOR_PULSE_MIN, MOTOR_PULSE_MAX);
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