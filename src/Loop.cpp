/*
 * loop.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: clark
 *      Modified: Zhengjie
 */

#include <cstdlib>
#include "Loop.hpp"
#include "time_util.h"

#include "PPM.h"
#include "time_util.h"
#include "rc.hpp"
#include "servoIn.hpp"
#include "inc/hw_nvic.h"
#include "EEPROM.h"

using namespace std;

Loop::Loop()
{
	for (int i = 0; i < NUM_EVENT; ++i)
	{
		_events[i].task = NULL;
		_events[i].lastTime = 0;
		_events[i].period = 0;
	}
}

void Loop::regEvent(Runnable* task, int32_t periodMs, uint32_t idx)
{
	if (idx >= NUM_EVENT) return;
	_events[idx].task = task;
	_events[idx].lastTime = 0;
	_events[idx].period = periodMs;
}

void Loop::run()
{
	int32_t loopTime;

	for (;;)
	{
		loopTime = millis();
		float arrayIn[24] = {1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0, 1.0, 2.0, 3.0, 4.0};
		float arrayOut[24] = {};
//		float array2 = 190;

		for (size_t i = 0; i < NUM_EVENT; ++i)
		{
			if ((loopTime - _events[i].lastTime) > _events[i].period)
			{
				_events[i].lastTime = loopTime;
				_events[i].task->run();
				Write_PID_EEPROM(arrayIn);
				Read_PID_EEPROM(arrayOut);
//				array2 = arrayOut[2];
			}
		}
	}
}
