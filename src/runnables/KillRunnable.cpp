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

KillRunnable::KillRunnable(ProgramState *pState) : state(pState)
{
//	fileNumber = 1;
}

void KillRunnable::run()
{
	if ((servoIn_getPulse(KILL_CHAN3)) < 120000)
	{
		state->sdcard->sync();

		// close the file
		state->sdcard->closeFile();

		// waiting for unkill signal
		while((servoIn_getPulse(KILL_CHAN3)) < 120000);

		// Start a new file
//		snprintf(fileName, sizeof(fileName), "log%u.txt", fileNumber++);
//		state->sdcard->createFile(fileName);
		state->sdcard->createFile();

	}
}
