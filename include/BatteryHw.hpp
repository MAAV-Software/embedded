#ifndef BATTERYHW_HPP_
#define BATTERYHW_HPP_

#include <stdint.h>
#include <cstdlib>
#include <cstdio>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/adc.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

void  ConfigADC(const uint32_t sysctlPeriphADC, const uint32_t adcBase,  const uint32_t sysctlPeriphGPIO, const uint32_t gpioPortBase, const uint8_t gpioADCPin, const uint8_t adcChannel);

uint32_t getADC(const uint32_t adcBase);

#endif /* BATTERYHW_HPP_ */
