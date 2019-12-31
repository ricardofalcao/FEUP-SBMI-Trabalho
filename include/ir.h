#ifndef _IR_H
#define _IR_H 1

#include <stdio.h>
#include "pins.h"

#define NEC_MAX_PACKET_BIT_NUMBER 32

typedef enum IR_Packet_Code_t {
	IR_NUMBER_ZERO = 25,
	IR_NUMBER_ONE = 69,
	IR_NUMBER_TWO = 70,
	IR_NUMBER_THREE = 71,
	IR_NUMBER_FOUR = 68,
	IR_NUMBER_FIVE = 64,
	IR_NUMBER_SIX = 67,
	IR_NUMBER_SEVEN = 7,
	IR_NUMBER_EIGHT = 21,
	IR_NUMBER_NINE = 9,
	IR_ASTERISK = 22,
	IR_HASHTAG = 13,
	IR_UP = 24,
	IR_DOWN = 82,
	IR_LEFT = 8,
	IR_RIGHT = 90,
	IR_OK = 28
} IR_Packet_Code; 

typedef struct IR_Packet_t {
	uint16_t address;
	uint8_t command; 
	uint8_t repeat;	
} IR_Packet;

typedef void (*IR_Listener_Function)(IR_Packet);

typedef struct IR_Listener_t {
	IR_Packet_Code code;
	IR_Listener_Function function;
} IR_Listener;

void init_ir(void);

void ir_add_listener(IR_Packet_Code code, IR_Listener_Function function);

IR_Packet * ir_run(void);

#endif