/*
 * EEPROM for PID 24 Floats and Log file counter
 * Zhengjie 7/30/2015
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/eeprom.h"

#define NUM_FLOAT 			24
#define MAX_FILE_COUNT	20
extern float floatDate[NUM_FLOAT];

void Config_EEPROM();
void Read_PID_EEPROM(float* dataOut);
void Write_PID_EEPROM(float* dataIn);
uint32_t Read_LOG_EEPROM();
void Write_LOG_EEPROM(uint32_t counter);

#ifdef __cplusplus
}
#endif

#endif /* EEPROM_H_ */
