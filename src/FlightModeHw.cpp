#include "FlightMode.hpp"
#include "FlightModeHw.hpp"
#include "rc.hpp"
#include "servoIn.hpp"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"


FlightMode flightModeGet(FlightMode last)
{
	if (pulseUpperThird(servoIn_getPulse(RC_CHAN5))) // Futaba Top Left Switch
	{
		return MANUAL;
	}
	if (pulseUpperThird(servoIn_getPulse(RC_CHAN6))) // Futaba Center Knob
	{
		return ASSISTED;
	}
	else if (pulseLowerThird(servoIn_getPulse(RC_CHAN6))) //Futaba Center Knob
	{
		return AUTONOMOUS;
	}
	else
	{
	    return last;
	}
}



