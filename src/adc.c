#include "adc.h"
#include "util.h"
#include <avr/io.h>


void init_adc() {
    ADMUX |= (1 << REFS0); // Vref = AVcc
    ADCSRA |= (7 << ADPS0) | (1 << ADEN); // Predivisor em 128 e ativar ADC

    DIDR0 |= 31; // Desativar buffer digital
}

uint16_t analog_read(Analog_Channel channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);

    while(ADCSRA & (1 << ADSC));
    return ADC;
}