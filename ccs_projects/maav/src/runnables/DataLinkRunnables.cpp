#include "runnables/DataLinkRunnables.hpp"

#include "utility.h"

/*
 * DataLinkProcessDataRunnable.cpp
 *
 *  Created on: Mar 28, 2015
 *      Author: Sasawat
 */

void ProcessReceivedDataRunnable::run() {
	data_link_process_incoming();
}

void TargetMessageHandlerRunnable::run() {
	if(message->timestamp > last)
	{
		last = message->timestamp;
		targetMessageQuadCtrlChangesHandler(qc, message);
	}
}

TargetMessageHandlerRunnable::TargetMessageHandlerRunnable(target_t *target, quad_ctrl_t *quad)
{
	qc = quad;
	message = target;
	last = 0;
}


void TuningMessageHandlerRunnable::run() {
	if(message->timestamp > last)
	{
		last = message->timestamp;
		tuningMessageQuadCtrlChangesHandler(qc, message);
	}
}

TuningMessageHandlerRunnable::TuningMessageHandlerRunnable(tuning_t *tuning, quad_ctrl_t *quad)
{
	qc = quad;
	message = tuning;
	last = 0;
}
