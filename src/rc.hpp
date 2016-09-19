#ifndef RC_HPP_
#define RC_HPP_

#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

#include "Pair.hpp"

// STUFF FOR RADIO CONTROL

// WARNING: ANY INDEX THE MACRO REPRESENTS IS OFF-BY-1.
// I.E. RC_CHAN1 IS THE PITCH CONTROL, WHICH ACTUALLY IS
// CHANNEL 0 IN THE REST OF THE CONTROLS CODE.
// DON'T ASK SAJAN WHY THIS IS LABELED AS SUCH SINCE HE
// DIDN'T CREATE THE NAMING CONVENTION.
//
// STOP BLAMING SASAWAT FOR THIS NAMING CONVENTION. 
// THIS NAMING CONVENTION PREDATES SASAWAT BY AT LEAST 2 YEARS

// GPIO Port Basses for pilot RC controller
extern const Maav::Pair<uint32_t, uint8_t> RC_CHAN1;
extern const Maav::Pair<uint32_t, uint8_t> RC_CHAN2;
extern const Maav::Pair<uint32_t, uint8_t> RC_CHAN3;
extern const Maav::Pair<uint32_t, uint8_t> RC_CHAN4;
extern const Maav::Pair<uint32_t, uint8_t> RC_CHAN5; // futaba top left switch
extern const Maav::Pair<uint32_t, uint8_t> RC_CHAN6; // futaba center knob

// GPIO Port Basses for Kill Switch (also an RC controller)
extern const Maav::Pair<uint32_t, uint8_t> KILL_CHAN1;
extern const Maav::Pair<uint32_t, uint8_t> KILL_CHAN2;
extern const Maav::Pair<uint32_t, uint8_t> KILL_CHAN3;
extern const Maav::Pair<uint32_t, uint8_t> KILL_CHAN4;
extern const Maav::Pair<uint32_t, uint8_t> KILL_CHAN5;
extern const Maav::Pair<uint32_t, uint8_t> KILL_CHAN6;

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

// calculates pulse width from thrust
float thrust2ms(const float val);

#endif /* RC_H_ */
