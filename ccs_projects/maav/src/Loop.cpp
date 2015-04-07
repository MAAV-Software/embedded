/*
 * loop.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: clark
 */

#include "Loop.hpp"

#include "time_util.h"

#include <stdlib.h>


Loop::Loop() { }

void Loop::addEvent(Runnable* task, int32_t periodMs) {
	Event event;
	event.task = task;
	event.lastTime = 0;
	event.period = periodMs;
	_events.push_back(event);
}

void Loop::run() {
	int32_t loopTime;

	while (1) {
		loopTime = millis();
		for (size_t i = 0; i < _events.size(); ++i) {
			if ((loopTime - _events[i].lastTime) > _events[i].period) {
				_events[i].lastTime = loopTime;
				_events[i].task->run();
			}
		}
	}
}
