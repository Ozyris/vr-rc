// mixer_difthrst.h

#ifndef MIXER_DIFTHRST_H
#define MIXER_DIFTHRST_H

#include <stdint.h>

#define PULSE_MIN 1000
#define PULSE_MAX 2000
#define PULSE_CENTER 1500
#define MAX_CHANNELS 8

// === НАСТРОЙКИ ДИФФЕРЕНЦИАЛЬНОЙ ТЯГИ ===
#define DIFTHRST_SCALE 50  // Масштаб рыскания (0-100%)

typedef struct {
    uint16_t channels[MAX_CHANNELS];
} MixerData;

// Микшер дифференциальной тяги
void applyMixer(MixerData *input, MixerData *output) {
    // Канал 1 (Elevator) - без изменений
    output->channels[0] = input->channels[0];
    
    // Канал 2 (Ailerons) - резерв, отправляем центр
    output->channels[1] = PULSE_CENTER;
    
    // === ДИФФЕРЕНЦИАЛЬНАЯ ТЯГА ===
    // Вход: CH2 = Y (левый стик Y), CH3 = Throttle (газ)
    // Выход: CH3 = Левый мотор, CH4 = Правый мотор
    
    // Получаем значения и переводим в смещения от центра (-500..500)
    int16_t throttle = input->channels[2] - PULSE_CENTER;  // Газ с CH3
    int16_t yaw = input->channels[1] - PULSE_CENTER;       // Y с CH2
    
    // Применяем масштаб рыскания
    yaw = yaw * DIFTHRST_SCALE / 100;
    
    // Вычисляем моторы (лево-право)
    int16_t motorLeft = throttle + yaw;   // Газ + Рыскание
    int16_t motorRight = throttle - yaw;  // Газ - Рыскание
    
    // Приводим к диапазону 1000-2000
    output->channels[2] = constrain(motorLeft + PULSE_CENTER, PULSE_MIN, PULSE_MAX);   // Левый мотор
    output->channels[3] = constrain(motorRight + PULSE_CENTER, PULSE_MIN, PULSE_MAX);  // Правый мотор
    
    // Каналы 5-8 без изменений
    for (int i = 4; i < MAX_CHANNELS; i++) {
        output->channels[i] = input->channels[i];
    }
}

void printMixerInfo(MixerData *data) {
    #ifdef DEBUG
        Serial.println("Mixer: DIFFERENTIAL THRUST");
        Serial.printf("Scale: %d%%\n", DIFTHRST_SCALE);
        Serial.printf("Output: CH1=%4d CH2=%4d CH3=%4d CH4=%4d CH5=%4d CH6=%4d CH7=%4d CH8=%4d\n",
                      data->channels[0], data->channels[1], 
                      data->channels[2], data->channels[3],
                      data->channels[4], data->channels[5],
                      data->channels[6], data->channels[7]);
    #endif
}

#endif // MIXER_DIFTHRST_H