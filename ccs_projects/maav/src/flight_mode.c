#include "flight_mode.h"
#include "servoIn.h"
//#include "general.h"
#include "rc.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"


FLIGHT_MODE flightModeGet()
{
	if (pulseUpperThird(servoIn_getPulse(GPIO_PORTA_BASE,6))) // was RC_CHAN5
	{
		return MANUAL;
	}
	if (pulseUpperThird(servoIn_getPulse(GPIO_PORTE_BASE,1))) // was KILL_CHAN5
	{
		return ASSISTED;
	}
	else
	{
		return AUTONOMOUS;
	}
}



