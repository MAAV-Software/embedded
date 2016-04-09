#include <stdint.h>
#include "rc.hpp"
#include "time_util.h"

#include "Pair.hpp"

using namespace std;

using Maav::Pair;

const Pair<uint32_t, uint8_t> RC_CHAN1 = Pair<uint32_t, uint8_t>(GPIO_PORTA_BASE, 2);
const Pair<uint32_t, uint8_t> RC_CHAN2 = Pair<uint32_t, uint8_t>(GPIO_PORTA_BASE, 3);
const Pair<uint32_t, uint8_t> RC_CHAN3 = Pair<uint32_t, uint8_t>(GPIO_PORTA_BASE, 4);
const Pair<uint32_t, uint8_t> RC_CHAN4 = Pair<uint32_t, uint8_t>(GPIO_PORTA_BASE, 5);
const Pair<uint32_t, uint8_t> RC_CHAN5 = Pair<uint32_t, uint8_t>(GPIO_PORTA_BASE, 6);
const Pair<uint32_t, uint8_t> RC_CHAN6 = Pair<uint32_t, uint8_t>(GPIO_PORTA_BASE, 7);

const Pair<uint32_t, uint8_t> KILL_CHAN1 = Pair<uint32_t, uint8_t>(GPIO_PORTE_BASE, 4);
const Pair<uint32_t, uint8_t> KILL_CHAN2 = Pair<uint32_t, uint8_t>(GPIO_PORTE_BASE, 5);
const Pair<uint32_t, uint8_t> KILL_CHAN3 = Pair<uint32_t, uint8_t>(GPIO_PORTE_BASE, 3);
const Pair<uint32_t, uint8_t> KILL_CHAN4 = Pair<uint32_t, uint8_t>(GPIO_PORTE_BASE, 2);
const Pair<uint32_t, uint8_t> KILL_CHAN5 = Pair<uint32_t, uint8_t>(GPIO_PORTE_BASE, 1);
const Pair<uint32_t, uint8_t> KILL_CHAN6 = Pair<uint32_t, uint8_t>(GPIO_PORTE_BASE, 0);


// Returns true if the pulse is longer than 1.66ms
bool pulseUpperThird(volatile uint32_t pulseWidth)
{
	return (pulseWidth > (SYSCLOCK / 602));
}

// Returns true if the pulse is shorter than 1.33ms
bool pulseLowerThird(volatile uint32_t pulseWidth)
{
	return (pulseWidth < (SYSCLOCK / 752));
}

// Converts pulse ticks to miliseconds
float pulse2ms(uint32_t pulse)
{
	static float one_ms = SYSCLOCK / 1000;
	return ((float)(pulse)) / one_ms;
}

// Converts miliseconds to pulse ticks
uint32_t ms2pulse(float ms)
{
	static float one_ms = (float)SYSCLOCK / 1000.0f;
	return (uint32_t)(ms * one_ms);
}

// Maps x which is in the range of [fromLow, fromHigh] into the range of
// [toLow, toHigh] and returns the result.
float map(const float x, const float fromLow, const float fromHigh, const float toLow, const float toHigh)
{
	return (((x - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow;
}

// Calculate XY_rate pass-through from pulse width
// (i.e. signal from RC controller modified and passed to DJI)
float ms2XY_rate(float ms)
{
	return map(ms, 1.0, 2.0, -2.0, 2.0);
}

// Calculates height pass-through from pulse width
float ms2height(const float ms)
{
	return map(ms, 1.0, 2.0, 0, 3.0);
	//return map(ms, 0.89, 1.92, 0.0, 3.0);
}

// Calcualtes PID XY setpoints from pulse width
float PID_XY_2ms(const float val)
{
	return map(val, -1.0, 1.0, 1.0, 2.0);
}

float thrust2ms(const float val)
{
	return map(val, -50.0f, 50.0f, 1.0f, 2.0f);
}

