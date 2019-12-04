#ifndef _PWM_H
#define _PWM_H 1

#include <stdio.h>
#include "pins.h"



typedef enum PWM_Channel_t {
	PWM_0A = 0xA0,
    PWM_0B = 0xB0
} PWM_Channel;

typedef enum PWM_Prescaler_t {
    PRESCALER_1 = 1,
    PRESCALER_8 = 2,
    PRESCALER_64 = 3,
    PRESCALER_256 = 4
} PWM_Prescaler;

void pwm_init(PWM_Channel pwm, PWM_Prescaler prescaler);

void pwm_write(PWM_Channel pwm, float dutyCycle);

#endif