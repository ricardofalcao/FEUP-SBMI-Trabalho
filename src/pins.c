#include "pins.h"
#include <avr/io.h>

void pin_mode(Pin_Section section, uint8_t pin, Pin_Mode mode) {
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

void digital_write(Pin_Section section, uint8_t pin, Digital_State value) {
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

Digital_State digital_read(Pin_Section section, uint8_t pin) {
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