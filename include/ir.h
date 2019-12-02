#ifndef _IR_H
#define _IRH 1

#include <stdio.h>
#include "pins.h"

#define NEC_MAX_PACKET_BIT_NUMBER 32

struct IR_Packet {
	uint16_t address;
	uint8_t command; 
	uint8_t repeat;	
};

void init_ir(void);

#endif