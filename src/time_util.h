/*
 * time_util.h
 *
 *  Created on: Jan 11, 2014
 *      Author: Jonathan Bendes
 *
 *  Edited on:	July 2, 2014
 *  	Author:	Jonathan Kurzer
 *  	Descr:	Added arguments to time_init so that arbitrary timer module can be used
 *  			Added millis() funciton
 */

#ifndef TIME_UTIL_H_
#define TIME_UTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>


// System Clock Frequency
#define SYSCLOCK 80000000
//#define SYSCLOCK 50000000
//#define SYSCLOCK 66666667

/**
 * @brief Initializers a Timer Module
 * @details Initializers the A Timer on the specified Timer Module. This is required for the millis() and timestamp_now() functions to work. While Timer Modules also have a B Timer, the B Timer only operates in 16 bit mode.
 *
 * @param ui32Peripheral Timer Peripheral to init on e.g. SYSCTL_PERIPH_TIMER1
 * @param sysClock System Clock, usually SYSCLOCK
 * @param ui32Base Timer Base to init on e.g. TIMER1_BASE
 * @param ui32Interrupt Timer Interrupt for the A Timer on the Base/Periph specified, e.g., INT_TIMER1A
 */
void time_init(uint32_t ui32Peripheral, uint32_t sysClock, uint32_t ui32Base, uint32_t ui32Interrupt);

/**
 * @brief Gets the current timestamp in microseconds
 */
uint64_t timestamp_now();

/**
 * @brief Gets the current timestamp in milliseconds
 */
uint32_t millis();

#ifdef __cplusplus
}
#endif

#endif /* TIME_UTIL_H_ */
