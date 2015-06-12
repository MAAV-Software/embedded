
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "utils/uartstdio.h"

// System Clock Frequency
#define SYSCLOCK 80000000

int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
				   SYSCTL_XTAL_16MHZ);

	// Init the LEDs on the Launchpad for debugging and init them to off
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 2);

    // Enable the GPIO Peripheral used by the UART0 on PA.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Enable UART0.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	UARTConfigSetExpClk(UART0_BASE, SYSCLOCK, 115200,
	    				    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
	    					 UART_CONFIG_PAR_NONE));

	// Configure UART Clock.
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
	UARTStdioConfig(0, 115200, SYSCLOCK);

	uint32_t count = 0;
	uint8_t ui8LED = 2;
	for (;;)
	{
		if (count > 100000)
		{
			count = 0;
			// Turn on the LED
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3,
						 ui8LED);
			// Cycle through Red, Green and Blue LEDs
			if ( ui8LED == 8 ) ui8LED = 2;
			else ui8LED = ui8LED * 2;

			UARTprintf("Loop Count = %u\n\r", count);
		}
		++count;
	}

	//return 0;
}
