#ifndef _H_UTIL
#define _H_UTIL 1

#include <stdio.h>

void init_util(void);

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

uint16_t clamp_16(uint16_t value, uint16_t min, uint16_t max);

uint8_t clamp_8(uint8_t value, uint8_t min, uint8_t max);

float clamp_f(float value, float min, float max);

uint64_t get_millis(void);

float calculate_battery(uint16_t voltage, uint16_t min_voltage, uint16_t max_voltage);

#endif