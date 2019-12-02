#include "util.h"

uint16_t clamp16(uint16_t value, uint16_t min, uint16_t max) {
    if(value > max) {
        value = max;
    } else if(value < min)  {
        value = min;
    }

    return value;
}

uint8_t clamp8(uint8_t value, uint8_t min, uint8_t max) {
    if(value > max) {
        value = max;
    } else if(value < min)  {
        value = min;
    }

    return value;
}
