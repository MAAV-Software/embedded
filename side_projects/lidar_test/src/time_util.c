/*
 * timer_util.c
 *
 *  Created on: Jan 11, 2014
 *      Author: jonathan
 *
 *  Edited on:	July 2, 2014
 *  	Author:	Jonathan Kurzer
 *  	Descr:	Added arguments to time_init so that arbitrary timer module can be used
 *  			Added millis() funciton
 */

#include <stdbool.h>
#include <stdint.h>

#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

static void Time_Util_Int_Handler(void);

volatile int32_t uptime_overflow = 0;
volatile int32_t timerBase;

static uint64_t frequency;
static uint64_t timer_cycles_per_overflow;
static uint64_t timer_period_us;
static bool 	init = false;

void time_init(uint32_t ui32Peripheral, uint32_t sysClock, uint32_t ui32Base, uint32_t ui32Interrupt) {
	if(init) return;
	init = true;
	timerBase = ui32Base;
	frequency    			  = sysClock;
	timer_cycles_per_overflow = sysClock;
	timer_period_us			  = timer_cycles_per_overflow * 1000000.0 / frequency;

    SysCtlPeripheralEnable(ui32Peripheral);
    TimerConfigure(timerBase, TIMER_CFG_PERIODIC);

    TimerLoadSet(timerBase, TIMER_A, timer_cycles_per_overflow);

    TimerIntRegister(timerBase, TIMER_A, &Time_Util_Int_Handler);

    IntPrioritySet(ui32Interrupt, 0x00);

    IntEnable(ui32Interrupt);
    TimerIntEnable(timerBase, TIMER_TIMA_TIMEOUT);

	TimerEnable(timerBase, TIMER_A);
}
uint64_t timestamp_now() {
	uint64_t us_sec, us_sec_confirm, us_us;

	us_sec = uptime_overflow * timer_period_us;

	uint64_t ticks = TimerValueGet(timerBase, TIMER_A);

	us_sec_confirm = uptime_overflow * timer_period_us;

	if(us_sec != us_sec_confirm) {
		ticks = TimerValueGet(timerBase, TIMER_A);
		us_sec = uptime_overflow * timer_period_us;
	}

	us_us = ((uint64_t)((timer_cycles_per_overflow - ticks) * 1000000) / frequency);

    uint64_t time = us_sec + us_us;

    return time;
}
uint32_t millis() {
	return(timestamp_now()/1000);
}

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
static void Time_Util_Int_Handler(void) {
    //
    // Clear the timer interrupt.
    //
	uint32_t mode = TimerIntStatus(timerBase, 1);
    TimerIntClear(timerBase, mode);
    //XXX MAKE SURE TIMER VALUE IS WORKING
    uptime_overflow += 1;
}
