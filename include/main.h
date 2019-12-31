#ifndef _MAIN_H
#define _MAIN_H 1

#include <stdio.h>

#define SENSORS_NUM 5
#define SENSORS_SAMPLES 4
#define LINE_TYPE 0 // 0 - Black line on white floor | 1 - White line on black floor
#define LINE_THRESHOLD 0.5

typedef enum Car_State_t {
	IDLE,
    CALIBRATING,
    MANUAL,
    AUTOMATIC
} Car_State; 

typedef enum Line_Sensor_Read_t {
    LINE,
    FLOOR
} Line_Sensor_Read; 

typedef struct Line_Sensor_t {
    uint8_t index;
    uint8_t pin;
    uint16_t minimum_value ;
    uint16_t maximum_value;
} Line_Sensor;

typedef enum Cross_Section_Decision_t {
	LEFT,
    RIGHT
} Cross_Section_Decision; 

#endif