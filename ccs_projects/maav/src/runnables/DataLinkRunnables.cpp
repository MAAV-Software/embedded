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

ProcessReceivedDataRunnable::~ProcessReceivedDataRunnable() {}

void TargetMessageHandlerRunnable::run() {
	if(_message->timestamp > _last)
	{
		_last = _message->timestamp;
		targetMessageQuadCtrlChangesHandler(_qc, _message);
	}
}

TargetMessageHandlerRunnable::TargetMessageHandlerRunnable(target_t *target, quad_ctrl_t *quad)
	: _qc(quad), _message(target), _last(0) { }

TargetMessageHandlerRunnable::~TargetMessageHandlerRunnable() {}


void TuningMessageHandlerRunnable::run() {
	if(_message->timestamp > _last)
	{
		_last = _message->timestamp;
		tuningMessageQuadCtrlChangesHandler(_qc, _message);
	}
}

TuningMessageHandlerRunnable::TuningMessageHandlerRunnable(tuning_t *tuning, quad_ctrl_t *quad)
	: _qc(quad), _message(tuning), _last(0) { }

TuningMessageHandlerRunnable::~TuningMessageHandlerRunnable() {}
