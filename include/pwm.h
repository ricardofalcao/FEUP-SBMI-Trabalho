#ifndef _PWM_H
#define _PWM_H 1

#include <stdio.h>
#include "pins.h"

#define PWM_0A 0xA0
#define PWM_0B 0xB0
// #define PWM_1A 0xA1 RESERVED FOR INFRA RED
// #define PWM_1B 0xB1
// #define PWM_2A 0xA2 RESERVED FOR MAIN TIMER
// #define PWM_2B 0xB2

#define PRESCALER_1 1
#define PRESCALER_8 2
#define PRESCALER_64 3
#define PRESCALER_256 4

void pwm_init(uint8_t pwm, uint8_t prescaler);

void pwm_write(uint8_t pwm, float dutyCycle);

#endif