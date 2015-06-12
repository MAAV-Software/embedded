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

#include <stdbool.h>
#include <stdint.h>

void time_init(uint32_t ui32Peripheral, uint32_t sysClock, uint32_t ui32Base, uint32_t ui32Interrupt);
uint64_t timestamp_now();
uint32_t millis();

#endif /* TIME_UTIL_H_ */
