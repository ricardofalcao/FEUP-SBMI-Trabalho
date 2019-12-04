#ifndef _PINS_H
#define _PINS_H 1

#include <stdio.h>

typedef enum Pin_Section_t {
	B, C, D
} Pin_Section;

typedef enum Pin_Mode_t {
	INPUT, OUTPUT
} Pin_Mode;

typedef enum Digital_State_t {
	LOW, HIGH
} Digital_State;

void pin_mode(Pin_Section section, uint8_t pin, Pin_Mode mode);

void digital_write(Pin_Section section, uint8_t pin, Digital_State value);

Digital_State digital_read(Pin_Section section, uint8_t pin);

#endif