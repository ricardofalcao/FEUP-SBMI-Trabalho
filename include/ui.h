#ifndef _UI_H
#define _UI_H 1

#include <stdio.h>
#include "pins.h"

#include "lcd.h"

void init_ui();

void draw_battery(float battery);

void draw_state(uint8_t state);

void draw_info(char *text);

void draw_sensors(uint8_t mask, float weighted_position);

#endif