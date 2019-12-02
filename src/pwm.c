#include "pwm.h"
#include "util.h"
#include <avr/io.h>

void pwm_init(uint8_t pwm, uint8_t prescaler) {
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

        /*case 0x01: {
            TCCR1B = 0; // stop timer
            TIFR1 = (7 << TOV1); // clear pending interrupts
            TCCR1A = (3 << WGM10);

            switch(letterBits) {
                case 0xA0: {
                    TCCR1A |= (1 << COM1A1);
                    OCR1A = 0;
                    pin_mode(B, 1, OUTPUT);
                    break;
                }
                case 0xB0: {
                    TCCR1A |= (1 << COM1B1);
                    OCR1B = 0;
                    pin_mode(B, 2, OUTPUT);
                    break;
                }
            }
            
            break;
        }

        case 0x02: {
            TCCR2B = 0; // stop timer
            TIFR2 = (7 << TOV2); // clear pending interrupts
            TCCR2A = (3 << WGM20);

            switch(letterBits) {
                case 0xA0: {
                    TCCR2A |= (1 << COM2A1);
                    OCR2A = 0;
                    pin_mode(B, 3, OUTPUT);
                    break;
                }
                case 0xB0: {
                    TCCR2A |= (1 << COM2B1);
                    OCR2B = 0;
                    pin_mode(D, 3, OUTPUT);
                    break;
                }
            }
            
            break;
        }*/
    }
}

void pwm_write(uint8_t pwm, float dutyCycle) {
    switch(pwm) {
        case PWM_0A: {
            OCR0A = clamp_8(255 * dutyCycle, 0, 255);
            break;
        }

        case PWM_0B: {
            OCR0B = clamp_8(255 * dutyCycle, 0, 255);
            break;
        }
        
        /*case PWM_1A: {
            OCR1A = clamp_16(1023 * dutyCycle, 0, 1023);
            break;
        }

        case PWM_1B: {
            OCR1B = clamp_16(1023 * dutyCycle, 0, 1023);
            break;
        }

        case PWM_2A: {
            OCR2A = clamp_8(255 * dutyCycle, 0, 255);
            break;
        }

        case PWM_2B: {
            OCR2B = clamp_8(255 * dutyCycle, 0, 255);
            break;
        } */
    }
}