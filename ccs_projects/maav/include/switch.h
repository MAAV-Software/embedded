#ifndef SWITCH_H_
#define SWITCH_H_

// Contains functions that handle the switches

#include "general.h"

// Struct for lighted 3-position switch states
typedef struct
{
	uint32_t periph;
	uint32_t portBase;
	uint8_t  pinNum		: 4;
	uint8_t  readState	: 1;
	uint8_t  driveState	: 1;
} SwitchData_t;

// GENERAL SWITCH FUNCTIONS
void switchesInit(SwitchData_t switches[3]);

void switchesUpdate(SwitchData_t switches[3]);


/**************** Utility Functions for Lighted 3-pos Switch ******************/
// Initilializes the 3-position, lighted switch
void initSwitch(uint32_t periph, uint32_t base, uint32_t pin,
				SwitchData_t *sData);

// Reads the switch position
void readSwitch(SwitchData_t *sData);

// Drives the switch's LED
void driveSwitch(SwitchData_t *sData, uint8_t direction);

#endif /* SWITCH_H_ */
