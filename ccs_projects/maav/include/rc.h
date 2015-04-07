#ifndef RC_H_
#define RC_H_

#ifdef __cplusplus
extern "C" {
#endif

// STUFF FOR RADIO CONTROL


#include "general.h"

// GPIO Port Basses for pilot RC controller
#define RC_CHAN1 GPIO_PORTA_BASE,2
#define RC_CHAN2 GPIO_PORTA_BASE,3
#define RC_CHAN3 GPIO_PORTA_BASE,4
#define RC_CHAN4 GPIO_PORTA_BASE,5
#define RC_CHAN5 GPIO_PORTA_BASE,6 // shoulder button
#define RC_CHAN6 GPIO_PORTA_BASE,7

// GPIO Port Basses for Kill Switch (also an RC controller)
#define KILL_CHAN1 GPIO_PORTE_BASE,4
#define KILL_CHAN2 GPIO_PORTE_BASE,5
#define KILL_CHAN3 GPIO_PORTE_BASE,3
#define KILL_CHAN4 GPIO_PORTE_BASE,2
#define KILL_CHAN5 GPIO_PORTE_BASE,1
#define KILL_CHAN6 GPIO_PORTE_BASE,0


// Returns true if the pulse is longer than 1.66ms
bool pulseUpperThird(volatile uint32_t pulseWidth);

// Returns true if the pulse is shorter than 1.33ms
bool pulseLowerThird(volatile uint32_t pulseWidth);

// Converts pulse ticks to miliseconds
float pulse2ms(uint32_t pulse);

// Converts miliseconds to pulse ticks
uint32_t ms2pulse(float ms);

// Maps x which is in the range of [fromLow, fromHigh] into the range of
// [toLow, toHigh] and returns the result.
float map(float x, float fromLow, float fromHigh, float toLow, float toHigh);

// Calculate XY_rate pass-through from pulse width
// (i.e. signal from RC controller modified and passed to DJI)
float ms2XY_rate(float ms);

// Calculates height pass-through from pulse width
float ms2height(float ms);

// Calcualtes PID XY setpoints from pulse width
float PID_XY_2ms(float val);

#ifdef __cplusplus
}
#endif

#endif /* RC_H_ */
