#ifndef _MAIN_H
#define _MAIN_H 1

#include <stdio.h>

#define SENSORS_NUM 5
#define LINE_TYPE 0 // 0 - Black line on white floor | 1 - White line on black floor

typedef enum Car_State_t {
	IDLE,
    CALIBRATING,
    MANUAL,
    AUTOMATIC
} Car_State; 

typedef enum Line_Sensor_Read_t {
	UNKNOWN,
    LINE,
    FLOOR
} Line_Sensor_Read; 

typedef struct Line_Sensor_t {
    uint8_t index;
    uint16_t floor_threshold;
    uint16_t line_threshold;
} Line_Sensor;

#endif