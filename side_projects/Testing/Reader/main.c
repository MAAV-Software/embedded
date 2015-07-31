/*
 * Read on UART1(PC4_Rx, PC5_Tx)
 * Send on UART0(PA0,PA1)
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

void reader_uart_config_sys_clock(void)
{
    // Set the clocking to run from the PLL at 50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
//    SysCtlPeripheralClockGating(true);
}

//****************************************************************************************
void reader_uart_int_handler(void)
{
    uint32_t ui32Status;

    // Get the interrrupt status.
    ui32Status = UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    UARTIntClear(UART0_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while(UARTCharsAvail(UART0_BASE))
    {
        // Read next data.
        UARTCharPut(UART1_BASE, UARTCharGetNonBlocking(UART0_BASE));
    }

}

//****************************************************************************************
void reader_uart_config_uart(void)
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

    UARTIntRegister(UART0_BASE, reader_uart_int_handler);
    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}


int main(void)
{
	// Might not be callled if it has already been called somewhere else
	reader_uart_config_sys_clock();

    reader_uart_config_uart();

    // Main application loop.
    while(1)
    {
    }

}
