#ifndef _ADC_H
#define _ADC_H 1

#include <stdio.h>
#include "pins.h"

#define ADC_VOLTAGE_REF 5

typedef enum Analog_Channel_t {
	ADC_CHANNEL_0,
	ADC_CHANNEL_1,
	ADC_CHANNEL_2,
	ADC_CHANNEL_3,
	ADC_CHANNEL_4,
	ADC_CHANNEL_5,
} Analog_Channel; 

void init_adc();

uint16_t analog_read(Analog_Channel channel);

#endif