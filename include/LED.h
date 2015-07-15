#ifndef LED_H_
#define LED_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"

// GPIO Pins for LEDs
#define RED_LED   GPIO_PIN_1
#define GREEN_LED GPIO_PIN_3
#define BLUE_LED  GPIO_PIN_2

void Config_LED();
void Toggle_LED(uint32_t ledPin, uint32_t time);

#ifdef __cplusplus
}
#endif

#endif /* LED_H_ */

