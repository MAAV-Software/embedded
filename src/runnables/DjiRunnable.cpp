#include "runnables/DjiRunnable.hpp"
#include "PPM.h"
#include "servoIn.hpp"
#include "rc.hpp"
#include "ProgramState.hpp"
#include "Vehicle.hpp"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

DjiRunnable::DjiRunnable(ProgramState* pState)
	: state(pState) { }

// TODO Add 10% Z Pulse Guard
void DjiRunnable::run()
{
	uint32_t throttle;
	Dji dji = state->vehicle->getDjiVals();

	switch(state->mode)
	{
		case AUTONOMOUS: // currently do this for safety
            PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));    // Y Accel
            PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));    // X Accel

//			//PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));	// Z Accel
//			uint32_t throttle = ms2pulse(PID_XY_2ms(state->vehicle->getDjiVals().thrust));
            throttle = servoIn_getPulse(RC_CHAN3);
			if (throttle < 80000)
				throttle = 80000;

            PPM_setPulse(2, throttle);    // Z Accel
            PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));    // Yaw Rate
		    break;
		case ASSISTED:
			PPM_setPulse(0, servoIn_getPulse(RC_CHAN1)); // for testing only, directly pass through roll and pitch
			PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));

			//PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));	// Z Accel
			//throttle = ms2pulse(thrust2ms(dji.thrust));
			throttle = map(dji.thrust, -10, 10, 75700, 153300);
			if (throttle < 75700)
				throttle = 75700;
			else if (throttle > 153300)
				throttle = 153300;

			PPM_setPulse(2, throttle);

			PPM_setPulse(3, servoIn_getPulse(RC_CHAN4)); // directly pass through yaw ratr
			break;
		case MANUAL:
			PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));	// Y Accel
			PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));	// X Accel
			PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));	// Z Accel
			PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));	// Yaw Rate
			break;
		default: break;
	}
}
