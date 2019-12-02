#ifndef _PINS_H
#define _PINS_H 1

#include <stdio.h>

#define B 0
#define C 1
#define D 2

#define OUTPUT 1
#define INPUT 0

#define HIGH 1
#define LOW 0

void pinMode(uint8_t section, uint8_t pin, uint8_t mode);

void digitalWrite(uint8_t section, uint8_t pin, uint8_t value);

uint8_t digitalRead(uint8_t section, uint8_t pin);

#endif