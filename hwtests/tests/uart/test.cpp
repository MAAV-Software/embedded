#include "TestFunction.hpp"

/*
 * Send 'abc' on UART0 and UART1
 */
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
#include "driverlib/uart.h"

void sender_uart_config_sys_clock(void)
{
    // Set the clocking to run from the PLL at 50MHz
   // SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
//    SysCtlPeripheralClockGating(true);
}

void sender_uart_config_uart(void)
{
    // Enable the GPIO Peripheral used by the UART1 on PC. UART0 on PA.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART1 and UART0.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
//    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
//    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Configure UART Clock.
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);

	//Configure UART for operation in the specified data format.
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

    // Register and Enable UART1 RX Interrupt.
//    UARTIntRegister(UART1_BASE, reader_uart_int_handler);
//    IntMasterEnable();
//    IntEnable(INT_UART1);
//    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);

}
//****************************************************************************************
void sender_uart_send(uint32_t Base, unsigned char *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
        // Write the next character to the UART.
        UARTCharPut(Base, *pui8Buffer++);
    }
}

void TestFunction::run()
{
	// Might not be called if it has already been called somewhere else
	sender_uart_config_sys_clock();
	sender_uart_config_uart();

	unsigned char buff[] = {'a','b','c','\n','\r'};

	// Main application loop.
	while(1)
	{
		sender_uart_send(UART1_BASE, buff, sizeof(buff));
		sender_uart_send(UART0_BASE, buff, sizeof(buff));
		SysCtlDelay(SysCtlClockGet()/3/1000*50);
	}

}
