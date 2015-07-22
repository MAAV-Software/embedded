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
//#define SYSCLOCK   66666667

void time_init(uint32_t ui32Peripheral, uint32_t sysClock, uint32_t ui32Base, uint32_t ui32Interrupt);
uint64_t timestamp_now();
uint32_t millis();

#ifdef __cplusplus
}
#endif

#endif /* TIME_UTIL_H_ */
