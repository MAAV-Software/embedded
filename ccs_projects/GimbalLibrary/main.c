//Predefined Symbol: PART_TM4C123GH6PM

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
//#include "utils/uartstdio.h"

void GPIO_A_Int_Handler(void);
void Timer0_Int_Handler(void);
void LED_Init(void);
void Timer_Init(void);
void GPIO_A_Init(void);
void UART_Init(void);
void itos(uint32_t,char[],uint8_t);

uint8_t led = 8;
uint32_t ui32_dt;

int main(void)
{
	//Clock
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	//LED GPIO Settings
	LED_Init();
	//Timers Setting
	Timer_Init();
	//Setup Port A2 For Interrupts
	GPIO_A_Init();
	// UART
	UART_Init();
	// Start! Turn Off 3 LEDs
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);

	while(1);
}

void GPIO_A_Int_Handler(void)
{
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_PIN_2);

	//Change LED Color
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, led);
	led++;

	if (GPIOPinRead(GPIO_PORTA_BASE,GPIO_PIN_2)){
		UARTCharPut(UART0_BASE, 'H');
		char buffer[13];
		itos(ui32_dt,buffer,13);
		uint8_t index = 0;
		for (index = 0; index < 13; index++)
		{
			UARTCharPut(UART0_BASE, buffer[index]);
		}
		ui32_dt = 0;
	}
	else{
		UARTCharPut(UART0_BASE, 'L');
		char buffer[13];
		itos(ui32_dt,buffer,13);
		uint8_t index = 0;
		for (index = 0; index < 13; index++)
		{
			UARTCharPut(UART0_BASE, buffer[index]);
		}
		ui32_dt = 0;
	}

}
void Timer0_Int_Handler(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	ui32_dt++;
}

void LED_Init()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}

void Timer_Init()
{
	TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

	//System clock 40MHz = 0.025us = 25ns;
	//Setting Timer clock = 10us
	//Timer load value = 10/.025  =  400 clock ticks
	TimerLoadSet(TIMER0_BASE, TIMER_A, 400);

	TimerIntRegister(TIMER0_BASE, TIMER_A,	Timer0_Int_Handler);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	TimerEnable(TIMER0_BASE, TIMER_A);
}
void GPIO_A_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2 , GPIO_STRENGTH_2MA , GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_BOTH_EDGES);

	GPIOIntRegister(GPIO_PORTA_BASE, GPIO_A_Int_Handler);
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_2);
	IntMasterEnable();
}
void UART_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void itos(uint32_t integer,char buffer[], uint8_t len)
{
	uint8_t i;
	uint32_t num = integer;
	for (i = 0; i<len; i++) buffer[i] = 0;
	buffer[len-1] = '\n';
	buffer[len-2] = '\r';
	i = len-3;
	while(num != 0)
	{
		buffer[i] = num % 10 + 48;
		num = num / 10;
		i--;
	}

}
