#include "FlightMode.hpp"
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


FlightMode flightModeGet()
{
	if (pulseUpperThird(servoIn_getPulse(RC_CHAN5))) // was RC_CHAN5
	{
		return MANUAL;
	}
	if (pulseUpperThird(servoIn_getPulse(KILL_CHAN5))) // was KILL_CHAN5
	{
		return ASSISTED;
	}
	else
	{
		return AUTONOMOUS;
	}
}



