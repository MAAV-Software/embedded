/*
 * KillRunnable.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

#include "runnables/KillRunnable.hpp"

#include <stdlib.h>
#include "LED.h"
#include "PPM.h"
#include "time_util.h"
#include "servoIn.hpp"
#include "rc.hpp"
#include "ImuHw.hpp"

KillRunnable::KillRunnable(ProgramState *pState) : state(pState)
{
//	fileNumber = 1;
}

void KillRunnable::run()
{
	if (state->kill->dutyCycle(servoIn_getPulse(KILL_CHAN3), 2) < 0.75)
	//if (!state->sw[2].readState)
	{
		state->sdcard->sync();

		// close the file
		state->sdcard->closeFile();

		emergency_t msg;
		msg.status = (int8_t)EMERGENCY_T_KILL;
		state->dLink->send(&msg);

		// disable flight mode LED
		MAP_GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED, 0);

		// waiting for unkill signal
		while(state->kill->dutyCycle(servoIn_getPulse(KILL_CHAN3), 2) < 0.75)
		{
		    //update switch configurations when killed
		    //do we want to this or force restart on config update
		    for(int i = 0; i < 3; ++i) state->sw[i].update();
		}

		resetYaw();

		/*
		while (!state->sw[2].readState)
		{
			switchesUpdate(state->sw);
		}
		*/

		// Start a new file
//		snprintf(fileName, sizeof(fileName), "log%u.txt", fileNumber++);
//		state->sdcard->createFile(fileName);
		state->sdcard->createFile();

		msg.status = (int8_t)EMERGENCY_T_NORMAL;
		state->dLink->send(&msg);

	}
}

float KillRunnable::getYawOffset()
{
    Imu imu; // create clean and temporary imu instance
    IMU_DONE = false;

    MicroStrainCmd cmd = imu.formatMeasCmd();

    imuUartSend(cmd.buf, cmd.length);

    while(!IMU_DONE); // busy loop OK here

    imu.parse(IMU_RAW_DATA);
    IMU_DONE = false;

    return imu.getYaw();
}

void KillRunnable::resetYaw()
{
    getYawOffset(); // throws away garbage reading
    float yoff = 0;

    for (int i = 0; i < 3; ++i)
    {
        yoff += 1.0 / 3.0 * getYawOffset();
    }

    state->imu->setRefYaw(yoff);
    Toggle_LED(BLUE_LED, SYSCLOCK / 2);
}
