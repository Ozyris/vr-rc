// mixer_default.h

#ifndef MIXER_DEFAULT_H
#define MIXER_DEFAULT_H

#include <stdint.h>

#define PULSE_MIN 1000
#define PULSE_MAX 2000
#define PULSE_CENTER 1500
#define MAX_CHANNELS 8

typedef struct {
    uint16_t channels[MAX_CHANNELS];
} MixerData;

// Микшер по умолчанию - просто копирует каналы 1:1
void applyMixer(MixerData *input, MixerData *output) {
    for (int i = 0; i < MAX_CHANNELS; i++) {
        output->channels[i] = input->channels[i];
    }
}

void printMixerInfo(MixerData *data) {
    #ifdef DEBUG
        Serial.println("Mixer: DEFAULT (1:1)");
        Serial.printf("Output: CH1=%4d CH2=%4d CH3=%4d CH4=%4d CH5=%4d CH6=%4d CH7=%4d CH8=%4d\n",
                      data->channels[0], data->channels[1], 
                      data->channels[2], data->channels[3],
                      data->channels[4], data->channels[5],
                      data->channels[6], data->channels[7]);
    #endif
}

#endif // MIXER_DEFAULT_H