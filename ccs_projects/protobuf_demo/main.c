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

#include "testmessage.h"

#define UART_SW_BUF_MAX_LENGTH 100

uint8_t *pui8UART2recvBuf;
int32_t i32UART2recvBufLoc;

int itoa(int32_t i32input, uint8_t *pui8Buf, int32_t offset) {
	uint8_t values[] = "0123456789";
	int32_t i32at = offset;
    if (i32input < 0) {
        pui8Buf[i32at++] = '-';
    }
    while(i32input != 0)
    {
    	pui8Buf[i32at++] = values[i32input%10];
    	i32input /= 10;
    }

    return i32at;
 }

/*
 * Handles the interrupt of UART2
 */
void UART2IntHandler(void)
{
	struct testmessage message;
	uint32_t ui32Status;

	//clear the interrupts
	ui32Status = UARTIntStatus(UART2_BASE, true);
	UARTIntClear(UART2_BASE, ui32Status);

	//Read whatever is in the hardware FIFO into our software buffer
	while(UARTCharsAvail(UART2_BASE) && i32UART2recvBufLoc < UART_SW_BUF_MAX_LENGTH)
	{
		pui8UART2recvBuf[i32UART2recvBufLoc] = UARTCharGet(UART2_BASE);
		i32UART2recvBufLoc++;
	}

	//try to parse
	if(Message_can_read_delimited_from(pui8UART2recvBuf,0,i32UART2recvBufLoc))
	{
		testmessage_read_delimited_from(pui8UART2recvBuf, &message, 0);
		i32UART2recvBufLoc = 0;

		//We have the message, tell UART0 (computer USB serial emulator link) about it
		uint8_t *pui8SendBuf = malloc(UART_SW_BUF_MAX_LENGTH);
		int32_t i32Count = 0;
		pui8SendBuf[i32Count++] = 'R';
		pui8SendBuf[i32Count++] = 'E';
		pui8SendBuf[i32Count++] = 'C';
		pui8SendBuf[i32Count++] = 'E';
		pui8SendBuf[i32Count++] = 'I';
		pui8SendBuf[i32Count++] = 'V';
		pui8SendBuf[i32Count++] = 'E';
		pui8SendBuf[i32Count++] = 'D';
		pui8SendBuf[i32Count++] = '\r';
		pui8SendBuf[i32Count++] = '\n';
		pui8SendBuf[i32Count++] = '1';
		pui8SendBuf[i32Count++] = ':';
		i32Count = itoa(message._i32thefirst, pui8SendBuf, i32Count);
		pui8SendBuf[i32Count++] = '\r';
		pui8SendBuf[i32Count++] = '\n';
		pui8SendBuf[i32Count++] = '2';
		pui8SendBuf[i32Count++] = ':';
		i32Count = itoa(message._i32thesecond, pui8SendBuf, i32Count);
		pui8SendBuf[i32Count++] = '\r';
		pui8SendBuf[i32Count++] = '\n';
		pui8SendBuf[i32Count++] = '\r';
		pui8SendBuf[i32Count++] = '\n';
		int32_t i32Iter;
		for(i32Iter = 0; i32Iter < i32Count; i32Iter++)
		{
			UARTCharPut(UART0_BASE, pui8SendBuf[i32Iter]);
		}
		free(pui8SendBuf);
	}
}



/*
 * main.c
 */
int main(void) {
	
	//configure clock
	SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	//enable peripherals
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

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
	GPIOPinConfigure(GPIO_PC6_U3RX);
	GPIOPinConfigure(GPIO_PC7_U3TX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	//configure UART stuff
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	pui8UART2recvBuf = malloc(UART_SW_BUF_MAX_LENGTH);
	i32UART2recvBufLoc = 0;

	//enable UART Receive Timeout and Receive interrupt for UART0
	IntEnable(INT_UART2);
	UARTIntEnable(UART2_BASE, UART_INT_RT | UART_INT_RX);

	//seed the rand generator
	int32_t i32Seed = 9001;
	srand(i32Seed++);

	while(1)
	{
		SysCtlDelay(20000000);
		testmessage message;
		message._i32thefirst = rand();
		message._i32thesecond = rand();

		//We have the message, tell UART0 (computer USB serial emulator link) about it
		uint8_t *pui8SendBuf = malloc(UART_SW_BUF_MAX_LENGTH);
		int32_t i32Count = 0;
		pui8SendBuf[i32Count++] = 'S';
		pui8SendBuf[i32Count++] = 'E';
		pui8SendBuf[i32Count++] = 'N';
		pui8SendBuf[i32Count++] = 'D';
		pui8SendBuf[i32Count++] = 'I';
		pui8SendBuf[i32Count++] = 'N';
		pui8SendBuf[i32Count++] = 'G';
		pui8SendBuf[i32Count++] = '\r';
		pui8SendBuf[i32Count++] = '\n';
		pui8SendBuf[i32Count++] = '1';
		pui8SendBuf[i32Count++] = ':';
		i32Count = itoa(message._i32thefirst, pui8SendBuf, i32Count);
		pui8SendBuf[i32Count++] = '\r';
		pui8SendBuf[i32Count++] = '\n';
		pui8SendBuf[i32Count++] = '2';
		pui8SendBuf[i32Count++] = ':';
		i32Count = itoa(message._i32thesecond, pui8SendBuf, i32Count);
		pui8SendBuf[i32Count++] = '\r';
		pui8SendBuf[i32Count++] = '\n';
		pui8SendBuf[i32Count++] = '\r';
		pui8SendBuf[i32Count++] = '\n';
		int32_t i32Iter;
		for(i32Iter = 0; i32Iter < i32Count; i32Iter++)
		{
			UARTCharPut(UART0_BASE, pui8SendBuf[i32Iter]);
		}

		i32Count = testmessage_write_delimited_to(&message, pui8SendBuf, 0);
		for(i32Iter = 0; i32Iter < i32Count; i32Iter++)
		{
			UARTCharPut(UART3_BASE, pui8SendBuf[i32Iter]);
		}

		free(pui8SendBuf);
	}
}
