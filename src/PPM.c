/*
 * PPM.c
 *
 *  Created on: Jul 2, 2014
 *      Author: Jonathan Kurzer
 */

#include <stdint.h>
#include <stdbool.h>

#include "PPM.h"

#include "inc/hw_memmap.h"

#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"

static volatile uint32_t ui32_PPM_FrameLen;
static volatile uint32_t timerBase;
static volatile uint32_t GPIO_portBase;
static volatile uint32_t GPIO_pinMask;
static volatile uint32_t ui32_PPM_TimeLow;
static volatile uint32_t ui32_PPM_Arr[10];
static volatile uint8_t  PPM_Channel;
static volatile int8_t  num_PPM_Channels;

static uint32_t ui32_PPM_Max;
static uint32_t ui32_PPM_Min;
static uint32_t CENTERTHROT;
static uint32_t sysClock;
static uint32_t stickPosScale;


static void PPM_Int_Handler(void);

void PPM_init(uint32_t timerPeripheral, uint32_t _sysClock, uint32_t _timerBase,
		uint32_t timerIntVect, uint32_t _GPIO_portBase, uint32_t _GPIO_pinMask,
		int8_t _num_PPM_Channels)
{
	// Initialize PPM variables
	timerBase 			= _timerBase;
	GPIO_portBase 		= _GPIO_portBase;
	GPIO_pinMask 		= _GPIO_pinMask;
	sysClock 			= _sysClock;
	num_PPM_Channels	= _num_PPM_Channels;
	CENTERTHROT 		= _sysClock / 667; // time on ~1.5ms
	stickPosScale		= (sysClock/2000000);

	PPM_Channel 		= 0;
	ui32_PPM_Min 		= _sysClock / 1111;		// Min pulse 0.9ms
	ui32_PPM_Max 		= _sysClock / 476;		// Max pulse 2.1ms
	ui32_PPM_TimeLow  	= _sysClock / 2500;		// time low  0.4ms
	ui32_PPM_FrameLen	= (2 + num_PPM_Channels) * ui32_PPM_Max;  // 2ms per channel

	int i;
	for(i = 0; i<num_PPM_Channels; i++) ui32_PPM_Arr[i] = CENTERTHROT; // time on ~1.5ms

	ui32_PPM_Arr[num_PPM_Channels] 	= ui32_PPM_FrameLen;

	// Set up Timer0 for PPM out
	SysCtlPeripheralEnable(timerPeripheral);
	TimerConfigure(timerBase, TIMER_CFG_PERIODIC);
	TimerControlStall(timerBase, TIMER_A, true);

	// Configure GPIO for the PPM driver pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(	GPIO_portBase, GPIO_pinMask);
	GPIOPadConfigSet(		GPIO_portBase, GPIO_pinMask, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD);	// standard push-pull output pin
	GPIOPinWrite(			GPIO_portBase, GPIO_pinMask, GPIO_pinMask);					// Set initial position to HIGH

	// Register interrupt and enable PPM timer
	TimerIntRegister(timerBase, TIMER_A, PPM_Int_Handler);
	TimerLoadSet(timerBase, TIMER_A, ui32_PPM_TimeLow);
	IntEnable(timerIntVect);
	TimerIntEnable(timerBase, TIMER_TIMA_TIMEOUT);
	TimerEnable(timerBase, TIMER_A);
	return;
}
void PPM_setPulse(uint8_t chNum, uint32_t pulseWidth) {
	if(chNum < num_PPM_Channels) {
		ui32_PPM_Arr[chNum] = MinMaxPulseThresh(pulseWidth);
	}
	return;
}
void PPM_setStickPos(uint8_t chNum, int16_t pos) {
	if(chNum < num_PPM_Channels) {
		pos = pos < -1000 ? -1000 : pos > 1000 ? 1000 : pos;	// Threshold pos to [-1000, 1000]
		ui32_PPM_Arr[chNum] = CENTERTHROT + (pos * stickPosScale);
	}
	return;
}
uint32_t MinMaxPulseThresh(uint32_t dataIn) {
	return  dataIn < ui32_PPM_Min ? ui32_PPM_Min :
			dataIn > ui32_PPM_Max ? ui32_PPM_Max : dataIn;
}
void PPM_Int_Handler(void) {	// This ISR is for sending the PPM signal to the DJI
	TimerIntClear(timerBase, TIMER_TIMA_TIMEOUT);		// Clear the timer interrupt

	if(GPIOPinRead(GPIO_portBase, GPIO_pinMask)) { 				// The PIN is HIGH
		GPIOPinWrite(GPIO_portBase, GPIO_pinMask, 0); 			// Turn off PPM pin
		if(PPM_Channel == 0) {									// Start of frame, reset acc
			ui32_PPM_Arr[num_PPM_Channels] = ui32_PPM_FrameLen; // reset frame time
		}
		TimerLoadSet(timerBase, TIMER_A, ui32_PPM_TimeLow-1);
	} else {													// the PIN is LOW
		GPIOPinWrite(GPIO_portBase, GPIO_pinMask, GPIO_pinMask); // Turn on PPM pin
		TimerLoadSet(timerBase, TIMER_A, ui32_PPM_Arr[PPM_Channel]-ui32_PPM_TimeLow-1);
		ui32_PPM_Arr[num_PPM_Channels] -= ui32_PPM_Arr[PPM_Channel];
		PPM_Channel++;
		PPM_Channel%=(num_PPM_Channels+1);
	}
}
