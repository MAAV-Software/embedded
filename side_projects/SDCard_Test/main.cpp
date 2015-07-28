/*
 * main.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

#include "Sd.hpp"
#include <string.h>

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"

int main()
{
	ROM_FPULazyStackingEnable();
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_3 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

//	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
//	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);
//	ROM_GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, 0);

//	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
//	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_4);
//	ROM_GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_4, 0);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	ROM_GPIOPinConfigure(GPIO_PD0_SSI1CLK);
//	ROM_GPIOPinConfigure(GPIO_PD1_SSI1FSS);
	ROM_GPIOPinConfigure(GPIO_PD2_SSI1RX);
	ROM_GPIOPinConfigure(GPIO_PD3_SSI1TX);

	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2);
	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2, GPIO_PIN_3|GPIO_PIN_2);

	ROM_SysCtlDelay(ROM_SysCtlClockGet());

	ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3|GPIO_PIN_2, 0);

	char write1[] = "This is SD class test\n";
	char write2[] = "This is the 2nd line\n";
	char write3[100] = "";
	char *filename = "test.txt";
//	char read[100];

	Sd sd;
	if (sd.sdMount())
	{
		// Create a new file
		sd.sdCreate(filename);

		sd.sdWrite(write1, (uint32_t)strlen(write1));

		// We can write multiply lines without closing the file
		sd.sdWrite(write2, (uint32_t)strlen(write2));

		// use snprintf to customize the string
		for (int i = 3; i < 500; i++)
		{
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
			snprintf(write3, sizeof(write3), "This is the line #:%u\n", i);
			sd.sdWrite(write3, (uint32_t)strlen(write3));
			ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
		}
		sd.sdClose();
		sd.sdSync();
		ROM_GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);

	}
//	sd.sdUnmount();

	while(1);
}
