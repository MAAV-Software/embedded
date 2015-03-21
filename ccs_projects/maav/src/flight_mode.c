#include "flight_mode.h"
#include "servoIn.h"

FLIGHT_MODE flightModeGet() {
	if (pulseUpperThird(servoIn_getPulse(RC_CHAN5))) {
		return MANUAL;
	}
	if (pulseUpperThird(servoIn_getPulse(KILL_CHAN5))) {
		return ASSISTED;
	} else {
		return AUTONOMOUS;
	}
}



