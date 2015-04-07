/*
 * servoIn.h
 *
 *  Created on: Jul 3, 2014
 *      Author: Jonathan Kurzer
 */

#ifndef SERVOIN_H_
#define SERVOIN_H_

#ifdef __cplusplus
extern "C" {
#endif

void servoIn_init(uint32_t ui32Peripheral, uint32_t ui32Base);
void servoIn_attachPin(void);
void servoIn_detachPin(void);
uint32_t servoIn_getPulse(uint32_t ui32Base, uint8_t pinIdx);
void capturePortPulse(uint32_t IntMask, uint32_t pinStat, volatile uint32_t riseTime[], uint32_t currTime, volatile uint32_t pulse[]);

#ifdef __cplusplus
}
#endif

#endif /* SERVOIN_H_ */
