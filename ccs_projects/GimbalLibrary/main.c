#include <stdint.h>
#include <stdbool.h>
//#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

void GPIOF_INTERRUPT_HANDLER(void);

uint32_t lastGPIOstatus = 0;
uint32_t lastTime = 0;


int main(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	TimerConfigure(TIMER0_BASE, TIMER_CFG_A_ONE_SHOT_UP);
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_4);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	GPIOIntRegister(GPIO_PORTF_BASE, GPIOF_INTERRUPT_HANDLER);
	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_4, GPIO_BOTH_EDGES);

	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_4);
	IntMasterEnable();

	TimerLoadSet(TIMER0_BASE, TIMER_A, 0xFFFFFFFF);
	TimerEnable(TIMER0_BASE, TIMER_A);
	UARTCharPut(UART0_BASE, 'H');
}


void GPIOF_INTERRUPT_HANDLER(void) {
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_2);
	uint32_t gpioValue = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_2);

	if (gpioValue == 0) {
		 // falling edge
		UARTCharPut(UART0_BASE, 'F');
		UARTCharPut(UART0_BASE, '\n');
		if (lastGPIOstatus == 1) {
			lastTime = TimerValueGet(TIMER0_BASE, TIMER_A);
		}
		lastGPIOstatus = 0;
	} else {
		// rising edge
		UARTCharPut(UART0_BASE, 'R');
		UARTCharPut(UART0_BASE, '\n');
		if (lastGPIOstatus == 0) {
			uint32_t timeElapsed = TimerValueGet(TIMER0_BASE, TIMER_A) - lastTime;
		}
		lastGPIOstatus = 1;
	}
}
