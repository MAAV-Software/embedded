/*
 * How to use protobuf messaging Tiva side
 *
 * Remember to:
 * * have messaging.c, messaging.h, ringbuf.c, and ringbuf.h
 * * stick messaging_UARTRxIntHAndler in as the interrupt handler for the UART for messaging
 * * have enough heap space around
 * * define MESSAGING_UART_BASE in messaging.h to the UART_BASE (e.g. UART2_BASE) for messaging
 */


#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

//Remember to include messaging.h
#include "protobuf.h"
#include "messaging.h"

int copyIntegerToBuffer(uint8_t *outBuf, int32_t input, int32_t offset) {
	uint8_t values[] = "0123456789";
	int32_t i32at = offset;
    if (input < 0) {
        outBuf[i32at++] = '-';
    }
    while(input != 0)
    {
    	outBuf[i32at++] = values[input%10];
    	input /= 10;
    }

    return i32at;
 }

int copyStringToBuffer(uint8_t *outBuf, uint8_t *inBuf, int32_t count, int32_t offset)
{
	int32_t iter;
	for(iter = 0; iter < count; iter++)
	{
		outBuf[offset++] = inBuf[iter];
	}
	return offset;
}

int main(void) {
	//messagedata struct for message data
	messaging_t mdat;

	//configure clock
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	//enable peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//enable interrupts
	IntMasterEnable();

	//peripherals take 5 cycles to enable
	SysCtlDelay(5);

	//configure pins
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	GPIOPinConfigure(GPIO_PD6_U2RX);
	GPIOPinConfigure(GPIO_PD7_U2TX);
	GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	//configure UART stuff
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	//enable messaging
	messaging_init(&mdat);

	//enable UART Receive Timeout and Receive interrupt for UART0
	IntEnable(INT_UART2);
	UARTIntEnable(UART2_BASE, UART_INT_RT | UART_INT_RX);

	uint8_t output[1024];
	int32_t at = 0;

	//Loop through and print whenever we get a command
	while(1)
	{
		if(mdat._cmd & MESSAGING_FLAG_NEW_SETPOINT)
		{
			//New Setpoint
			mdat._cmd &= ~MESSAGING_FLAG_NEW_SETPOINT;
			at = copyStringToBuffer(output, "New Setpoint\r\n", 14, at);
			at = copyIntegerToBuffer(output, mdat._x, at);
			at = copyStringToBuffer(output, "\r\n", 2, at);
			at = copyIntegerToBuffer(output, mdat._y, at);
			at = copyStringToBuffer(output, "\r\n", 2, at);
			at = copyIntegerToBuffer(output, mdat._z, at);
			at = copyStringToBuffer(output, "\r\n", 2, at);
			at = copyIntegerToBuffer(output, mdat._h, at);
			at = copyStringToBuffer(output, "\r\n", 4, at);
		}
		if(mdat._cmd & MESSAGING_FLAG_LAND)
		{
			//Land
			mdat._cmd &= ~MESSAGING_FLAG_LAND;
			at = copyStringToBuffer(output, "Land\r\n", 6, at);
		}
		if(mdat._cmd & MESSAGING_FLAG_SET_DOT_SETPOINT)
		{
			//New Dot Setpoint
			mdat._cmd &= ~MESSAGING_FLAG_SET_DOT_SETPOINT;
			at = copyStringToBuffer(output, "New Setpoint of Derivatives\r\n", 29, at);
			at = copyIntegerToBuffer(output, mdat._xdot, at);
			at = copyStringToBuffer(output, "\r\n", 2, at);
			at = copyIntegerToBuffer(output, mdat._ydot, at);
			at = copyStringToBuffer(output, "\r\n", 2, at);
			at = copyIntegerToBuffer(output, mdat._zdot, at);
			at = copyStringToBuffer(output, "\r\n", 4, at);
		}
		if(mdat._cmd & MESSAGING_FLAG_SET_LOCATION)
		{
			//Update current location
			mdat._cmd &= ~MESSAGING_FLAG_SET_LOCATION;
			at = copyStringToBuffer(output, "Current Location updated\r\n", 26, at);
			at = copyIntegerToBuffer(output, mdat._xactual, at);
			at = copyStringToBuffer(output, "\r\n", 2, at);
			at = copyIntegerToBuffer(output, mdat._yactual, at);
			at = copyStringToBuffer(output, "\r\n", 2, at);
			at = copyIntegerToBuffer(output, mdat._zactual, at);
			at = copyStringToBuffer(output, "\r\n", 4, at);
		}
		if(mdat._cmd & MESSAGING_FLAG_SET_PIDCONST)
		{
			//New PID constants
			mdat._cmd &= ~MESSAGING_FLAG_SET_PIDCONST;
			at = copyStringToBuffer(output, "New KPID\r\n", 10, at);

			at = copyIntegerToBuffer(output, mdat._KPX, at);
			at = copyStringToBuffer(output, "\r\nKPX ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KIX, at);
			at = copyStringToBuffer(output, "\r\nKIX ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KDX, at);
			at = copyStringToBuffer(output, "\r\nKDX ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KPXdot, at);
			at = copyStringToBuffer(output, "\r\nKPXdot ", 9, at);
			at = copyIntegerToBuffer(output, mdat._KIXdot, at);
			at = copyStringToBuffer(output, "\r\nKIXdot ", 9, at);
			at = copyIntegerToBuffer(output, mdat._KDXdot, at);
			at = copyStringToBuffer(output, "\r\nKDXdot ", 9, at);
			at = copyIntegerToBuffer(output, mdat._KPY, at);
			at = copyStringToBuffer(output, "\r\n", 2, at);

			at = copyIntegerToBuffer(output, mdat._KPY, at);
			at = copyStringToBuffer(output, "\r\nKPY ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KIY, at);
			at = copyStringToBuffer(output, "\r\nKIY ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KDY, at);
			at = copyStringToBuffer(output, "\r\nKDY ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KPYdot, at);
			at = copyStringToBuffer(output, "\r\nKPYdot ", 9, at);
			at = copyIntegerToBuffer(output, mdat._KIYdot, at);
			at = copyStringToBuffer(output, "\r\nKIYdot ", 9, at);
			at = copyIntegerToBuffer(output, mdat._KDYdot, at);
			at = copyStringToBuffer(output, "\r\nKDYdot ", 9, at);
			at = copyIntegerToBuffer(output, mdat._KPY, at);
			at = copyStringToBuffer(output, "\r\n", 2, at);

			at = copyIntegerToBuffer(output, mdat._KPZ, at);
			at = copyStringToBuffer(output, "\r\nKPZ ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KIZ, at);
			at = copyStringToBuffer(output, "\r\nKIZ ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KDZ, at);
			at = copyStringToBuffer(output, "\r\nKDZ ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KPZdot, at);
			at = copyStringToBuffer(output, "\r\nKPZdot ", 9, at);
			at = copyIntegerToBuffer(output, mdat._KIZdot, at);
			at = copyStringToBuffer(output, "\r\nKIZdot ", 9, at);
			at = copyIntegerToBuffer(output, mdat._KDZdot, at);
			at = copyStringToBuffer(output, "\r\nKDZdot ", 9, at);
			at = copyIntegerToBuffer(output, mdat._KPY, at);
			at = copyStringToBuffer(output, "\r\n", 2, at);

			at = copyIntegerToBuffer(output, mdat._KPH, at);
			at = copyStringToBuffer(output, "\r\nKPH ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KIH, at);
			at = copyStringToBuffer(output, "\r\nKIH ", 6, at);
			at = copyIntegerToBuffer(output, mdat._KDH, at);
			at = copyStringToBuffer(output, "\r\nKDH ", 6, at);

			at = copyStringToBuffer(output, "\r\n", 4, at);
		}
		if(mdat._cmd & MESSAGING_FLAG_TAKEOFF)
		{
			//Take off
			mdat._cmd &= ~MESSAGING_FLAG_TAKEOFF;
			at = copyStringToBuffer(output, "Take Off\r\n", 10, at);
		}

		int32_t i32Iter;
		for(i32Iter = 0; i32Iter < at; i32Iter++)
		{
			UARTCharPut(UART0_BASE, output[i32Iter]);
		}
		at = 0;

		SysCtlDelay(5000000);
	}

	return 0;
}
