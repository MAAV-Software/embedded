//#include <stdint.h>
//#include <stdlib.h>
//#include <stdbool.h>
//#include <stdio.h>
//
//#include "inc/hw_ints.h"
//#include "inc/hw_memmap.h"
//#include "inc/hw_types.h"
//#include "inc/tm4c123gh6pm.h"
//
//#include "driverlib/sysctl.h"
//#include "driverlib/gpio.h"
//#include "driverlib/pin_map.h"
//#include "driverlib/rom.h"
//#include "driverlib/rom_map.h"
//#include "driverlib/timer.h"

#include "LED.h"

//****************************************************************************************
void Config_LED()
{
	// Enable the GPIO port that is used for the on-board LED.
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	// Enable the GPIO pins for the LED (PF1 & PF2 & PF3).
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1);

	// Turn off All LEDs.
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0);
}

//****************************************************************************************
void Toggle_LED(uint32_t ledPin, uint32_t time)
{
	// Turn off LED.
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, ledPin, ledPin);
	ROM_SysCtlDelay(time);

	// Turn on LED.
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, ledPin, 0);
	ROM_SysCtlDelay(time);
}
