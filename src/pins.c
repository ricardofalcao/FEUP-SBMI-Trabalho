#include "pins.h"
#include <avr/io.h>

void pin_mode(uint8_t section, uint8_t pin, uint8_t mode) {
    switch(section) {
        case B: {
            if(mode == OUTPUT) {
                DDRB |= (1 << pin);
            } else {
                DDRB &= ~(1 << pin);
            }
            
            break;
        }

        case C: {
            if(mode == OUTPUT) {
                DDRC |= (1 << pin);
            } else {
                DDRC &= ~(1 << pin);
            }
            
            break;
        }

        case D: {
            if(mode == OUTPUT) {
                DDRD |= (1 << pin);
            } else {
                DDRD &= ~(1 << pin);
            }
            
            break;
        }
    }
}

void digital_write(uint8_t section, uint8_t pin, uint8_t value) {
    switch(section) {
        case B: {
            if(value == LOW) {
                PORTB &= ~(1 << pin);
            } else {
                PORTB |= (1 << pin);
            } 

            break;
        }
        
        case C: {
            if(value == LOW) {
                PORTC &= ~(1 << pin);
            } else {
                PORTC |= (1 << pin);
            } 

            break;
        }

        case D: {
            if(value == LOW) {
                PORTD &= ~(1 << pin);
            } else {
                PORTD |= (1 << pin);
            } 

            break;
        }
    }
}

uint8_t digital_read(uint8_t section, uint8_t pin) {
    switch(section) {
        case B: {
            if(PINB & (1 << pin)) {
                return HIGH;
            }

            return LOW;
        }
        
        case C: {
            if(PINC & (1 << pin)) {
                return HIGH;
            }

            return LOW;
        }

        case D: {
            if(PIND & (1 << pin)) {
                return HIGH;
            }

            return LOW;
        }
    }

    return LOW;
}