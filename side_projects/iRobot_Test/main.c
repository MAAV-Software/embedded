/*
 * Author: Zhengjie
 * Date: June 27 2015
 * iRobot test
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"

#include "utils/uartstdio.h"

#define MAXDRIVE	95600
#define MINDRIVE	44500
#define MAXTURN		95800
#define MINTURN		54000

void Config_Sys_clock(void);
void Config_LED(void);
void Config_UART(void);
void Toggle_LED(uint32_t led, uint32_t time);
void UART_Send(uint32_t Base, const uint8_t *pui8Buffer, uint32_t ui32Count);
void Config_Timer(void);
void Config_Receiver_GPIO(void);
void Receiver_PortA_IntHandler(void);

// Pulse witdh from Receiver
volatile uint32_t Pulse[2];

int main(void) {

	Config_Sys_clock();
	Config_LED();
	Config_UART();
	Config_Timer();
	Config_Receiver_GPIO();
	uint32_t g_one_sec = SysCtlClockGet()/3;

	Toggle_LED(GPIO_PIN_2, g_one_sec);
	Toggle_LED(GPIO_PIN_2, g_one_sec);
	Toggle_LED(GPIO_PIN_2, g_one_sec);

	uint8_t cmd1[] = {128, 132}; // roomba full mode
	UART_Send(UART1_BASE, cmd1, 2);
	Toggle_LED(GPIO_PIN_2, g_one_sec);

	uint8_t SpeedHigh = 0;
	uint8_t SpeedLow = 0;
	uint8_t RadiusHigh = 255;
	uint8_t RadiusLow = 0;
	// Speed range +-500mm/s Radius +-2000m
	int speed = 0;
	int radius = 2000;

	uint8_t cmd2[] = {137, SpeedHigh, SpeedLow, RadiusHigh, RadiusLow};
//	UART_Send(UART1_BASE, cmd2, 5);
//	Toggle_LED(GPIO_PIN_2, g_one_sec);

	Toggle_LED(GPIO_PIN_2, g_one_sec);
	uint8_t song1[] = {140, 0, 16, 76, 16, 76, 16, 76, 32, 76, 16, 76, 16, 76, 32, 76, 16, 79, 16, 72, 16, 74, 16, 76, 32, 77, 16, 77, 16, 77, 16, 77, 32, 77, 16};
	uint8_t song2[] = {140, 1, 7, 76, 16, 76, 32, 79, 16, 79, 16, 77, 16, 74, 16, 72, 32};
	uint8_t play1[] = {141, 0};
	uint8_t play2[] = {141, 1};
	UART_Send(UART1_BASE, song1, sizeof(song1));
	Toggle_LED(GPIO_PIN_2, g_one_sec/1000*15);
	UART_Send(UART1_BASE, song2, sizeof(song2));
	Toggle_LED(GPIO_PIN_2, g_one_sec/1000*15);
	UART_Send(UART1_BASE, play1, sizeof(play1));
	Toggle_LED(GPIO_PIN_2, g_one_sec*4/2);
	UART_Send(UART1_BASE, play2, sizeof(play2));
	Toggle_LED(GPIO_PIN_2, g_one_sec*5);

	while(1)
	{
//		UART_Send(UART1_BASE, cmd1, 1);
//		Toggle_LED(GPIO_PIN_3, g_one_sec/1000*15);

		UARTprintf("%d\t%d\t", Pulse[0], Pulse[1]);
		speed = 500 - (MAXDRIVE - (float)Pulse[0])/(MAXDRIVE - MINDRIVE)*(500+500);
		SpeedHigh = speed >> 8;
		SpeedLow = speed & 255;
		cmd2[1] = SpeedHigh;
		cmd2[2] = SpeedLow;

		radius = 1950 - (MAXTURN - (float)Pulse[1])/(MAXTURN - MINTURN)*(1950+1950);
		if (radius >= 0)
		{
			radius = radius - 2000;
		}
		else
		{
			radius += 2000;
		}
		RadiusHigh = radius >> 8;
		RadiusLow = radius & 255;
		cmd2[3] = RadiusHigh;
		cmd2[4] = RadiusLow;
		UARTprintf("%d\t%d\t", speed, radius);
		UARTprintf("%u\t%u\t%u\t%u\n", SpeedHigh, SpeedLow, RadiusHigh, RadiusLow);
		UART_Send(UART1_BASE, cmd2, 5);

//		wait for 20ms LED toggle
		Toggle_LED(GPIO_PIN_3, g_one_sec/1000*15);
	}

//	return 0;
}

void Config_Sys_clock()
{
	// Set the clocking to run from the PLL at 50MHz
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
}

void Config_LED()
{
	// Enable the GPIO port that is used for the on-board LED.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	// Enable the GPIO pins for the LED (PF1 & PF2 & PF3).
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1);

	// Turn off All LEDs.
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1, 0);
}

void Config_UART()
{
	// Enable the GPIO Peripheral used by the UART.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	// Enable UART1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	// Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Use the internal 16MHz oscillator as the UART clock source.
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	UARTClockSourceSet(UART1_BASE, UART_CLOCK_SYSTEM);

	// Initialize the UART for console I/O.
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 57600, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTStdioConfig(0, 57600, 16000000);
//	UARTStdioConfig(0, 57600, UARTClockSourceGet(UART0_BASE));
}

void UART_Send(uint32_t Base, const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    // Loop while there are more characters to send.
    while(ui32Count--)
    {
        // Write the next character to the UART.
        UARTCharPut(Base, *pui8Buffer++);
    }
}

void Toggle_LED(uint32_t led, uint32_t time)
{
	// Turn off LED.
	GPIOPinWrite(GPIO_PORTF_BASE, led, led);
	SysCtlDelay(time);
	// Turn on LED.
	GPIOPinWrite(GPIO_PORTF_BASE, led, 0);
	SysCtlDelay(time);
}

void Config_Timer()
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC_UP);
	TimerClockSourceSet(TIMER0_BASE, TIMER_CLOCK_SYSTEM);
	TimerControlStall(TIMER0_BASE, TIMER_A, true);

	//System clock 50MHz = 0.020us = 20ns;
	//Setting Timer clock = 10us
	//Timer load value = 10/.020  =  500 clock ticks
	// 2^32-1 = 4294967295
//	TimerLoadSet(TIMER0_BASE, TIMER_A, 4294967295);
	TimerEnable(TIMER0_BASE, TIMER_A);
}

void Config_Receiver_GPIO()
{
	// Set up GPIO on Port A for input PWM signal interrupts
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2|GPIO_PIN_3, GPIO_BOTH_EDGES);
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_3);
	GPIOIntRegister(GPIO_PORTA_BASE, Receiver_PortA_IntHandler);
}

void Receiver_PortA_IntHandler(void)
{
	static volatile uint32_t RiseTime[2]; // 0 for Pin2, 1 for Pin3
	uint32_t Status = GPIOIntStatus(GPIO_PORTA_BASE, true);
	uint32_t PinRead = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);
	uint32_t CurrTime = TimerValueGet(TIMER0_BASE, TIMER_A);
	GPIOIntClear(GPIO_PORTA_BASE, Status);
	// if PIN2 interrupt
	if (Status & GPIO_PIN_2)
	{
		// Rising edge
		if (PinRead & GPIO_PIN_2)
		{
			RiseTime[0] = CurrTime;
		}
		// Falling edge
		else
		{
			Pulse[0] = CurrTime-RiseTime[0];
		}
	}
	// if Pin3 interrupt
	if (Status & GPIO_PIN_3)
	{
		// Rising edge
		if (PinRead & GPIO_PIN_3)
		{
			RiseTime[1] = CurrTime;
		}
		// Falling edge
		else
		{
			Pulse[1] = CurrTime-RiseTime[1];
		}
	}

}
