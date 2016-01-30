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
	if ((servoIn_getPulse(KILL_CHAN3)) < 120000)
	//if (!state->sw[2].readState)
	{
		state->sdcard->sync();

		// close the file
		state->sdcard->closeFile();

		emergency_t msg;
		msg.status = (int8_t)EMERGENCY_T_KILL;
		state->dLink->send(&msg);

		// waiting for unkill signal
		while((servoIn_getPulse(KILL_CHAN3)) < 120000);

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
    Imu imu;
    imuDone = false;
    imuUartSend(&imuCmd, 1);
    while(!imuDone);
    imu.parse(imuRawFinal);
    imuDone = false;
    return imu.getYaw();
}

void KillRunnable::resetYaw()
{
    getYawOffset();
    float yoff = 0;
    for(int i = 0; i < 3; ++i)
    {
        yoff += 1.0/3.0 * getYawOffset();
    }
    state->imu->setRefYaw(yoff);
    Toggle_LED(BLUE_LED, SYSCLOCK / 2);
}
