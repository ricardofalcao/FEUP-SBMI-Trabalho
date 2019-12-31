#include "pwm.h"
#include "util.h"
#include <avr/io.h>

void pwm_init(PWM_Channel pwm, PWM_Prescaler prescaler) {
    uint8_t letterBits = (pwm & 0xF0);
    uint8_t idBits = (pwm & 0x0F);

    switch(idBits) {
        case 0x00: {
            TCCR0B = 0; // stop timer
            TIFR0 = (7 << TOV0); // clear pending interrupts
            TCCR0B = (prescaler << CS00);
            TCCR0A |= (3 << WGM00);

            switch(letterBits) {
                case 0xA0: {
                    TCCR0A |= (1 << COM0A1);
                    OCR0A = 0;
                    pin_mode(D, 6, OUTPUT);
                    break;
                }
                case 0xB0: {
                    TCCR0A |= (1 << COM0B1);
                    OCR0B = 0;
                    pin_mode(D, 5, OUTPUT);
                    break;
                }
            }

            break;
        }
    }
}

void pwm_write(PWM_Channel pwm, float dutyCycle) {
    switch(pwm) {
        case PWM_0A: {
            OCR0A = 255 * clamp_f(dutyCycle, 0.0f, 1.0f);
            break;
        }

        case PWM_0B: {
            OCR0B = 255 * clamp_f(dutyCycle, 0.0f, 1.0f);
            break;
        }
    }
}