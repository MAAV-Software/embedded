/*
 * EEPROM.h
 *
 *  Created on: Jul 31, 2015
 *      Author: Zhengjie
 */

#include "EEPROM.h"

float floatArray[NUM_FLOAT];

void Config_EEPROM()
{
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
	ROM_EEPROMInit();
}

void Write_PID_EEPROM(float * dataIn)
{
	ROM_EEPROMProgram((uint32_t*)dataIn, 0x10, sizeof(floatArray));
}

void Read_PID_EEPROM(float * dataOut)
{
	ROM_EEPROMRead((uint32_t*)dataOut, 0x10, sizeof(floatArray));
//	return floatArray;
}

uint32_t Read_LOG_EEPROM()
{
	uint32_t counter = 0;
	ROM_EEPROMRead(&counter, 0x0, sizeof(counter));
	return counter;
}

void Write_LOG_EEPROM(uint32_t counter)
{
	ROM_EEPROMProgram(&counter, 0x0, sizeof(counter));
}
