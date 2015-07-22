/*
 * loop.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: clark
 */

#include <stdlib.h>
#include "Loop.hpp"
#include "time_util.h"
#include <vector>

#include "PPM.h"
#include "time_util.h"
#include "rc.hpp"
#include "servoIn.hpp"

Loop::Loop() { }

void Loop::addEvent(Runnable* task, int32_t periodMs) {
	Event event;
	event.task = task;
	event.lastTime = 0;
	event.period = periodMs;
	_events.push_back(event);
}

void Loop::run(SdCard* sdcard)
{
	int32_t loopTime;

	for (;;)
	{
		loopTime = millis();
		for (size_t i = 0; i < _events.size(); ++i)
		{
			if ((servoIn_getPulse(KILL_CHAN3)) < (low_kill + SYSCLOCK * 5 / 1000))
			{
				// Kill
				sdcard->closeFile();
				sdcard->unmount();

			}

			//TODO: FIGURE OUT WHY THIS WORKS BUT NOT THE COMMENTED OUT PART!!!!
			Event tmp = _events[i];
			if ((loopTime - tmp.lastTime) > tmp.period)
			{
				tmp.lastTime = loopTime;
				tmp.task->run();
			}
			_events[i] = tmp;


//			if ((loopTime - _events[i].lastTime) > _events[i].period)
//			{
//				_events[i].lastTime = loopTime;
//				_events[i].task->run();
//			}

		}
	}
}
