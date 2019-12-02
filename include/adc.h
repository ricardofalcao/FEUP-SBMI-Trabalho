#ifndef _ADC_H
#define _ADC_H 1

#include <stdio.h>
#include "pins.h"

#define ADC_VOLTAGE_REF 5

#define ADC_CHANNEL_0 0
#define ADC_CHANNEL_1 1
#define ADC_CHANNEL_2 2
#define ADC_CHANNEL_3 3
#define ADC_CHANNEL_4 4
#define ADC_CHANNEL_5 5

void init_adc();

uint16_t analogRead(uint8_t channel);

#endif