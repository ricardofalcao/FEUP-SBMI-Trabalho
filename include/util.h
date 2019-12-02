#ifndef _H_UTIL
#define _H_UTIL 1

#include <stdio.h>

void init_util(void);

uint16_t clamp_16(uint16_t value, uint16_t min, uint16_t max);

uint8_t clamp_8(uint8_t value, uint8_t min, uint8_t max);

uint64_t get_millis(void);


#endif