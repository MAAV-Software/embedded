/*
 * loop.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: clark
 */

#include <cstdlib>
#include "Loop.hpp"
#include "time_util.h"

#include "PPM.h"
#include "time_util.h"
#include "rc.hpp"
#include "servoIn.hpp"
#include "inc/hw_nvic.h"

using namespace std;

Loop::Loop()
{
	_eventCnt = 0;
}

void Loop::regEvent(Runnable* task, int32_t periodMs) {
	_events[_eventCnt].task = task;
	_events[_eventCnt].lastTime = 0;
	_events[_eventCnt].period = periodMs;
	_eventCnt++;
}

void Loop::run()
{
	int32_t loopTime;

	for (;;)
	{
		loopTime = millis();
		for (size_t i = 0; i < NUM_EVENT; ++i)
		{
			if ((loopTime - _events[i].lastTime) > _events[i].period)
			{
				_events[i].lastTime = loopTime;
				_events[i].task->run();
			}
		}
	}
}
