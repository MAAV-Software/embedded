/*
 * servoIn.c
 *
 *  Created on: Jul 3, 2014
 *      Author: Leonard
 */

#include <stdint.h>
//#include <stdbool.h>

#include "servoIn.hpp"

#include "inc/hw_memmap.h"

#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include "Pair.hpp"

volatile uint32_t pulseIn_PortA[8];
//volatile uint32_t pulseIn_PortE[8];
volatile uint32_t pulseIn_PortC[8];


static void servoIn_PortA_IntHandler(void);	// for PULSE in
//static void servoIn_PortE_IntHandler(void);	// for PULSE in
static void servoIn_PortC_IntHandler(void); // for PULSE in

void servoIn_init(uint32_t ui32Peripheral, uint32_t ui32Base)
{
	// Set up timer for pulses (and other things if needed)
	MAP_SysCtlPeripheralEnable(ui32Peripheral);
	MAP_TimerConfigure(ui32Base, TIMER_CFG_PERIODIC_UP);
	MAP_TimerControlStall(ui32Base, TIMER_A, true);
	MAP_TimerEnable(ui32Base, TIMER_A);
	return;
}
void servoIn_attachPin(void)
{
	// Set up GPIO on Port A for input PWM signal interrupts
	const uint32_t GPIO_MASK_A =         GPIO_PIN_2 |     GPIO_PIN_3 |     GPIO_PIN_4 |      GPIO_PIN_5 |     GPIO_PIN_6 |     GPIO_PIN_7;
	const uint32_t GPIO_INT_MASK_A = GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4 |  GPIO_INT_PIN_5 | GPIO_INT_PIN_6 | GPIO_INT_PIN_7;
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_MASK_A);
	MAP_GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_MASK_A, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	MAP_GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_MASK_A, GPIO_BOTH_EDGES);
	MAP_GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_MASK_A);
	GPIOIntRegister(GPIO_PORTA_BASE, servoIn_PortA_IntHandler);

	/*
	// Set up GPIO on Port E for input PWM signal interrupts
	const uint32_t GPIO_MASK_E =         GPIO_PIN_0 |     GPIO_PIN_1 |     GPIO_PIN_2 |     GPIO_PIN_3 |     GPIO_PIN_4 |     GPIO_PIN_5;
	const uint32_t GPIO_INT_MASK_E = GPIO_INT_PIN_0 | GPIO_INT_PIN_1 | GPIO_INT_PIN_2 | GPIO_INT_PIN_3 | GPIO_INT_PIN_4 | GPIO_INT_PIN_5;
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_MASK_E);
	MAP_GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_MASK_E, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	MAP_GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_MASK_E, GPIO_BOTH_EDGES);
	MAP_GPIOIntEnable(GPIO_PORTE_BASE,  GPIO_INT_MASK_E);
	GPIOIntRegister(GPIO_PORTE_BASE, servoIn_PortE_IntHandler);
	*/

	// Set up GPIO on Port E for input PWM signal interrupts
	const uint32_t GPIO_MASK_C =         GPIO_PIN_6 |     GPIO_PIN_7;
	const uint32_t GPIO_INT_MASK_C = GPIO_INT_PIN_6 | GPIO_INT_PIN_7;
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_MASK_C);
	MAP_GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_MASK_C, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	MAP_GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_MASK_C, GPIO_BOTH_EDGES);
	MAP_GPIOIntEnable(GPIO_PORTC_BASE,  GPIO_INT_MASK_C);
	GPIOIntRegister(GPIO_PORTC_BASE, servoIn_PortC_IntHandler);

	return;
}

void servoIn_detachPin(void)
{
	return;
}
uint32_t servoIn_getPulse(uint32_t ui32Base, uint8_t pinIdx)
{
	if(ui32Base == GPIO_PORTA_BASE)
	{
		return pulseIn_PortA[pinIdx];
	}
	/*
	else if(ui32Base == GPIO_PORTE_BASE)
	{
		return pulseIn_PortE[pinIdx];
	}
	*/
	else if (ui32Base == GPIO_PORTC_BASE)
	{
		return pulseIn_PortC[pinIdx];
	}

	return 0;
}

uint32_t servoIn_getPulse(const Maav::Pair<uint32_t, uint8_t>& pinInfo)
{
	uint32_t ui32Base = pinInfo.first;
	uint8_t pinIdx = pinInfo.second;

	if (ui32Base == GPIO_PORTA_BASE)
	{
		return pulseIn_PortA[pinIdx];
	}
	/*
	else if (ui32Base == GPIO_PORTE_BASE)
	{
		return pulseIn_PortE[pinIdx];
	}
	*/
	else if (ui32Base == GPIO_PORTC_BASE)
	{
		return pulseIn_PortC[pinIdx];
	}

	return 0;
}


void capturePortPulse(uint32_t IntMask, uint32_t pinStat, volatile uint32_t riseTime[], uint32_t currTime, volatile uint32_t pulse[])
{
	uint8_t i, pos;
	for(i = 0; i < 8; i++)
	{							// Check all 8 pins on port
		pos = (1 << i);
		if(IntMask & pos)
		{  							// there was a logic change on this Pin
			if(pinStat & pos)
			{
				riseTime[i] = currTime;	// Rising edge: log the time
			}
			else if(riseTime[i] < currTime)
			{				// Falling edge: calculate and report pulse width
				pulse[i] = currTime-riseTime[i];
			}
		}
	}
	return;
}
void servoIn_PortA_IntHandler(void)
{	// This port is reserved for signals from the RC controller.
	static volatile uint32_t riseTime[8];
	uint32_t intStat = MAP_GPIOIntStatus(GPIO_PORTA_BASE, true);
	const uint32_t pinMask = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	uint32_t pinStat = MAP_GPIOPinRead(GPIO_PORTA_BASE, pinMask);
	uint32_t currTime = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);
	MAP_GPIOIntClear(GPIO_PORTA_BASE, intStat);		// Clear the interrupt(s)
	capturePortPulse(intStat, pinStat, riseTime, currTime, pulseIn_PortA);
}

/*
void servoIn_PortE_IntHandler(void)
{	// This port is reserved for signals from the Kill Switch controller.
	static volatile uint32_t riseTime[8];
	uint32_t intStat = MAP_GPIOIntStatus(GPIO_PORTE_BASE, true);
	const uint32_t pinMask = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
	uint32_t pinStat = MAP_GPIOPinRead(GPIO_PORTE_BASE, pinMask);
	uint32_t currTime = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);
	MAP_GPIOIntClear(GPIO_PORTE_BASE, intStat);		// Clear the interrupt(s)
	capturePortPulse(intStat, pinStat, riseTime, currTime, pulseIn_PortE);
}
*/

void servoIn_PortC_IntHandler(void)
{	// This port is reserved for signals from the RC controller.
	static volatile uint32_t riseTime[8];
	uint32_t intStat = MAP_GPIOIntStatus(GPIO_PORTC_BASE, true);
	const uint32_t pinMask = GPIO_PIN_6 | GPIO_PIN_7;
	uint32_t pinStat = MAP_GPIOPinRead(GPIO_PORTC_BASE, pinMask);
	uint32_t currTime = MAP_TimerValueGet(TIMER4_BASE, TIMER_A);
	MAP_GPIOIntClear(GPIO_PORTC_BASE, intStat);		// Clear the interrupt(s)
	capturePortPulse(intStat, pinStat, riseTime, currTime, pulseIn_PortC);
}

