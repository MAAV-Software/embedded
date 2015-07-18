#include "runnables/DjiRunnable.hpp"
#include "PPM.h"
#include "servoIn.hpp"
#include "rc.hpp"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

DjiRunnable::DjiRunnable(ProgramState* pState)
	: state(pState) { }

DjiRunnable::~DjiRunnable() { }

void DjiRunnable::run() {
	//switch (vehicle->getFlightMode())
	switch(state->mode)
	{
		case AUTONOMOUS:
		case ASSISTED:
//			PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));
//			PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));
//
//			// convert Z Uval into PWM Pulse
//			float zPulse = PID_XY_2ms(qc.xyzh[Z_AXIS].Uval);
//
//			zPulse = (zPulse > 1.2) ? zPulse : 1.2;
//
//			PPM_setPulse(2, ms2pulse(zPulse));	// Z control to DJI
//			PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));
			break;
		case MANUAL:
			PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));	// Y Accel
			PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));	// X Accel
			PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));	// Z Accel
			PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));	// Yaw Rate
			break;
		default:
			break;
	}
}

