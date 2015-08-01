/*
 * LED.c
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

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
	TurnOff_LED( ledPin);
	ROM_SysCtlDelay(time);

	TurnOn_LED(ledPin);
	ROM_SysCtlDelay(time);
}

void TurnOn_LED(uint32_t ledPin)
{
	// Turn on LED.
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, ledPin, 0);
}

void TurnOff_LED(uint32_t ledPin)
{
	// Turn off LED.
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, ledPin, ledPin);
}
