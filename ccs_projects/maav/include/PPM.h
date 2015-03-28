/*
 * PPM.h
 *
 *  Created on: Jul 2, 2014
 *      Author: Jonathan Kurzer
 */

#ifndef PPM_H_
#define PPM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

//typedef struct PPM_Driver_t PPM_Driver_t;


void PPM_init(uint32_t ui32Peripheral, uint32_t sysClock, uint32_t ui32Base,
			  uint32_t ui32Interrupt, uint32_t _GPIO_portBase,
			  uint32_t _GPIO_pinMask, int8_t _num_PPM_Channels);
void PPM_setPulse(uint8_t chNum, uint32_t pulseWidth);
void PPM_setStickPos(uint8_t chNum, int16_t pos);
uint32_t MinMaxPulseThresh(uint32_t dataIn);

#ifdef __cplusplus
}
#endif

#endif /* PPM_H_ */
