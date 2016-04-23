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
    static int changecount = 0;

    FlightMode next;
	if (pulseUpperThird(servoIn_getPulse(RC_CHAN5))) // Futaba Top Left Switch
	{
		next = MANUAL;
	}
	else if (pulseLowerThird(servoIn_getPulse(RC_CHAN5)
	        && pulseUpperThird(servoIn_getPulse(RC_CHAN6))) // Futaba Center Knob
	{
		next = ASSISTED;
	}
	else if (pulseLowerThird(servoIn_getPulse(RC_CHAN5)
            && pulseLowerThird(servoIn_getPulse(RC_CHAN6))) //Futaba Center Knob
	{
		next = AUTONOMOUS;
	}
	else
	{
	    next = last;
	}

	if(next == last)
	{
	    changecount = 0;
	    return next;
	}
	else
	{
	    ++changecount;
	    if(changecount > CHANGE_ONLY_IF_EXCEEDS)
	    {
	        changecount = 0;
	        return next;
	    }
	    else
	    {
	        return last;
	    }
	}
	return last;
}



