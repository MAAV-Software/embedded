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
	: state(pState), ppmXdlast(0), ppmYdlast(0), ppmFzlast(0), ppmYawdlast(0) { }

// TODO Add 10% Z Pulse Guard
void DjiRunnable::run()
{
	uint32_t throttle;
	Dji dji = state->vehicle->getDjiVals();

    uint32_t ppmXd = servoIn_getPulse(RC_CHAN1);
    uint32_t ppmYd = servoIn_getPulse(RC_CHAN2);
    uint32_t ppmFz = servoIn_getPulse(RC_CHAN3);
    uint32_t ppmYawd = servoIn_getPulse(RC_CHAN4);

	switch(state->mode)
	{
		case AUTONOMOUS: // currently do this for safety
			//PPM_setPulse(0, (uint32_t)map(dji.pitch, -0.5, 0.5, 105600, 135000));
            PPM_setPulse(0, servoIn_getPulse(RC_CHAN1));    // X Accel

            PPM_setPulse(1, (uint32_t)map(dji.roll, -0.5, 0.5, 104200, 135000)); // Y Accel
            //PPM_setPulse(1, servoIn_getPulse(RC_CHAN2));    // Y Accel

//			//PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));	// Z Accel
//			uint32_t throttle = ms2pulse(PID_XY_2ms(state->vehicle->getDjiVals().thrust));
//            throttle = servoIn_getPulse(RC_CHAN3);
//			if (throttle < 80000)
//				throttle = 80000;

        	throttle = (uint32_t)map(dji.thrust, 0, 46.6956, 114000, 124000);
			if (throttle < 75700)
				throttle = 75700;
			else if (throttle > 153300)
				throttle = 153300;

            PPM_setPulse(2, throttle);    // Z Accel

            //PPM_setPulse(3, servoIn_getPulse(RC_CHAN4));    // Yaw Rate
            PPM_setPulse(3, (uint32_t)map(dji.yawRate, -1, 1, 113000, 120400));

		    break;
		case ASSISTED:
			//PPM_setPulse(0, (uint32_t)map(dji.pitch, -0.5, 0.5, 105600, 135000));
            if(ppmXd > 200000)   ppmXd = ppmXdlast;
            else                 ppmXdlast = ppmXd;
            PPM_setPulse(0, ppmXd);    // X Accel

            PPM_setPulse(1, (uint32_t)map(dji.roll, -0.5, 0.5, 104200, 135000)); // Y Accel
            //PM_setPulse(1, servoIn_getPulse(RC_CHAN2));    // Y Accel

			//PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));	// Z Accel
			//throttle = ms2pulse(thrust2ms(dji.thrust));
			//throttle = (uint32_t)map(dji.thrust, -125.0, 125.0, 75700, 153300);
			throttle = (uint32_t)map(dji.thrust, 0, 46.6956, 114000, 124000);
			if (throttle < 75700)
				throttle = 75700;
			else if (throttle > 153300)
				throttle = 153300;

			PPM_setPulse(2, throttle);

            if(ppmYawd > 200000) ppmYawd = ppmYawdlast;
            else                 ppmYawdlast = ppmYawd;
			PPM_setPulse(3, ppmYawd); // directly pass through yaw ratr
			break;
		case MANUAL:
		    //Try to filter out garbage values
		    if(ppmXd > 200000)   ppmXd = ppmXdlast;
		    else                 ppmXdlast = ppmXd;
            if(ppmYd > 200000)   ppmYd = ppmYdlast;
            else                 ppmYdlast = ppmYd;
            if(ppmFz > 200000)   ppmFz = ppmFzlast;
            else                 ppmFzlast = ppmFz;
            if(ppmYawd > 200000) ppmYawd = ppmYawdlast;
            else                 ppmYawdlast = ppmYawd;
            //Set values
			PPM_setPulse(0, ppmXd);  // X Accel
			PPM_setPulse(1, ppmYd);  // Y Accel
			PPM_setPulse(2, ppmFz);  // Z Accel
			PPM_setPulse(3, ppmYawd);// Yaw Rate
			break;
		default: break;
	}
}
