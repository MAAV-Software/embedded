#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

void Config_sys_Clock(void)
{
    // Set the clocking to run from the PLL at 50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
//    SysCtlPeripheralClockGating(true);
}

//****************************************************************************************
void Config_LED(void)
{
	// Enable the GPIO port that is used for the on-board LED.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	// Enable the GPIO pins for the LED (PF1 & PF2 & PF3).
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1);

	// Turn off All LEDs.
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0);
}

//****************************************************************************************
void Toggle_LED(uint32_t led, uint32_t time)
{
	// Turn off LED.
	GPIOPinWrite(GPIO_PORTF_BASE, led, led);
	SysCtlDelay(time);
	// Turn on LED.
	GPIOPinWrite(GPIO_PORTF_BASE, led, 0);
	SysCtlDelay(time);
}

void togglePin(uint32_t port, uint32_t pin, uint32_t time)
{
	// Turn off LED.
	GPIOPinWrite(port, pin, pin);
	SysCtlDelay(time);
	// Turn on LED.
	GPIOPinWrite(port, pin, 0);
	SysCtlDelay(time);
}


//****************************************************************************************
int main(void)
{
	Config_sys_Clock();

	// Enable the GPIO port that is used for the on-board LED.
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// Enable the GPIO pins for the LED (PF1 & PF2 & PF3).
//	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);

	// Turn off All LEDs.
//	GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0);

	Config_LED();

	while (1)
	{
//		togglePin(GPIO_PORTE_BASE, GPIO_PIN_4, SysCtlClockGet()/300000);

//		Toggle_LED(GPIO_PIN_1, SysCtlClockGet()/3);
//		Toggle_LED(GPIO_PIN_2, SysCtlClockGet()/3);
		Toggle_LED(GPIO_PIN_3, SysCtlClockGet()/3);
	}
}
