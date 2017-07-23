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
 //   uint8_t error = 0;
	uint32_t throttle;
	uint32_t yawRateCmd;
	uint32_t pitch;
	uint32_t roll;
	Dji dji = state->vehicle->getDjiVals();

    float dutyXd = state->pilot->dutyCycle(servoIn_getPulse(RC_CHAN1), 0); // pitch (vertical right stick)
    float dutyYd = state->pilot->dutyCycle(servoIn_getPulse(RC_CHAN2), 1); // roll (horizontal right stick)
    float dutyFz = state->pilot->dutyCycle(servoIn_getPulse(RC_CHAN3), 2); // thrust (verticle left stick)
    float dutyYawd = state->pilot->dutyCycle(servoIn_getPulse(RC_CHAN4), 3); // yaw rate (horizontal right stick)

    /*
    //Try to filter out garbage values
    state->vehicle->clearRCInputError();
    if(dutyXd < 0.0 || dutyXd > 1.0)
    {
        dutyXd = 0.5;
        error |= DJI_SERVOIN_XD_OOB;
    }
    if(dutyYd < 0.0 || dutyYd > 1.0)
    {
        dutyYd = 0.5;
        error |= DJI_SERVOIN_YD_OOB;
    }
    if(dutyFz < -0.01 || dutyFz > 1.0)
    {
        dutyFz = 0.01;
        error |= DJI_SERVOIN_FZ_OOB;
    }
    if(dutyYawd < 0.0 || dutyYawd > 1.0)
    {
        dutyYawd = 0.5;
        error |= DJI_SERVOIN_YAWD_OOB;
    }
    state->vehicle->setRCInputError(error);
    */

    switch(state->mode)
    {
		case AUTONOMOUS: // currently do this for safety

			pitch = (uint32_t) map(dji.pitch, 0.0, 1.0, state->djiout->dutyCycle(0.0, 0), state->djiout->dutyCycle(1.00, 0));
			roll = (uint32_t) map(dji.roll, 0.0, 1.0, state->djiout->dutyCycle(0.0, 1), state->djiout->dutyCycle(1.00, 1));
			throttle = (uint32_t) map(dji.thrust, 0.0, 1.0, state->djiout->dutyCycle(0.0, 2), state->djiout->dutyCycle(1.00, 2));
			yawRateCmd = (uint32_t) map(dji.yawRate, 0.0, 1.0, state->djiout->dutyCycle(0.0, 3), state->djiout->dutyCycle(1.00, 3));

			PPM_setPulse(0, pitch);  // X Accel
			PPM_setPulse(1, roll);  // Y Accel
			PPM_setPulse(2, throttle);  // Z Accel
			PPM_setPulse(3, yawRateCmd);// Yaw Rate
			break;
            
		case ASSISTED:
			//PPM_setPulse(0, (uint32_t)map(dji.pitch, -0.5, 0.5, 105600, 135000));
		    //X Accel
            //if(state->isAssistedXYPassThrough()) PPM_setPulse(0, state->djiout->dutyCycle(dutyXd, 0));
            //else                                 PPM_setPulse(0, (uint32_t)map(dji.pitch, -0.5, 0.5, 105600, 135000));

			pitch = (uint32_t)map(dji.pitch, -0.25, 0.25, state->djiout->dutyCycle(0.00, 0), state->djiout->dutyCycle(1.00, 0));

        	if (pitch < state->djiout->dutyCycle(0.00, 0)) pitch = state->djiout->dutyCycle(0.00, 0);
        	else if (pitch > state->djiout->dutyCycle(1.00, 0)) pitch = state->djiout->dutyCycle(1.00, 0);

        	PPM_setPulse(0, pitch);

            //PPM_setPulse(1, (uint32_t)map(dji.roll, -0.5, 0.5, 104200, 135000));
            //Y Accel
            //if(true) PPM_setPulse(1, state->djiout->dutyCycle(dutyYd, 1));
            //else                                 PPM_setPulse(1, (uint32_t)map(dji.roll, -0.5, 0.5, 104200, 135000));
        	//PPM_setPulse(1, state->djiout->dutyCycle(dutyYd, 1));  // Y Accel

        	roll = (uint32_t)map(dji.roll, -0.25, 0.25, state->djiout->dutyCycle(0.00, 1), state->djiout->dutyCycle(1.00, 1));

        	if (roll < state->djiout->dutyCycle(0.00, 1)) roll = state->djiout->dutyCycle(0.00, 1);
        	else if (roll > state->djiout->dutyCycle(1.00, 1)) roll = state->djiout->dutyCycle(1.00, 1);

        	PPM_setPulse(1, roll);


			//PPM_setPulse(2, servoIn_getPulse(RC_CHAN3));	// Z Accel
			//throttle = ms2pulse(thrust2ms(dji.thrust));
			//throttle = (uint32_t)map(dji.thrust, -125.0, 125.0, 75700, 153300);
			//throttle = (uint32_t)map(dji.thrust, 0, 46.6956, 114000, 124000);


        	throttle = (uint32_t)map(dji.thrust, 0, 46.6956, state->djiout->dutyCycle(0.09, 2), state->djiout->dutyCycle(1.00, 2));
//            if (throttle < 75700)
//				throttle = 75700;
//			else if (throttle > 153300)
//				throttle = 153300;
        	if (throttle < state->djiout->dutyCycle(0.09, 2)) throttle = state->djiout->dutyCycle(0.09, 2);
        	else if (throttle > state->djiout->dutyCycle(1.00, 2)) throttle = state->djiout->dutyCycle(1.00, 2);
        	PPM_setPulse(2, throttle);

            //Z PASSTHROUGH, REMOVE
			//PPM_setPulse(2, state->djiout->dutyCycle(dutyFz, 2));  // Z Accel

			PPM_setPulse(3, state->djiout->dutyCycle(dutyYawd, 3)); // directly pass through yaw ratr
			break;

		case MANUAL:
            //Set values
			PPM_setPulse(0, state->djiout->dutyCycle(dutyXd, 0));  // X Accel
			PPM_setPulse(1, state->djiout->dutyCycle(dutyYd, 1));  // Y Accel
			PPM_setPulse(2, state->djiout->dutyCycle(dutyFz, 2));  // Z Accel
			PPM_setPulse(3, state->djiout->dutyCycle(dutyYawd, 3));// Yaw Rate
			break;

		default: break;
	}
}
