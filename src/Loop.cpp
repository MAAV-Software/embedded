/*
 * loop.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: clark
 */

#include <cstdlib>
#include "Loop.hpp"
#include "time_util.h"

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
	if (idx < NUM_EVENT)
	{
		_events[idx].task = task;
		_events[idx].lastTime = 0;
		_events[idx].period = periodMs;
	}
}

void Loop::run()
{
	int32_t loopTime;

	for (;;)
	{
		loopTime = millis();
		for (size_t i = 0; i < NUM_EVENT; ++i)
		{
			if ((loopTime - _events[i].lastTime) >= _events[i].period)
			{
				_events[i].lastTime = loopTime;
				_events[i].task->run();
			}
		}
	}
}
