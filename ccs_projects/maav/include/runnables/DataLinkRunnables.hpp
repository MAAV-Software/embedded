/*
 * DataLinkProcessDataRunnable.hpp
 *
 *  Created on: Mar 28, 2015
 *      Author: Sasawat
 */

#ifndef DATALINKPROCESSDATARUNNABLE_HPP_
#define DATALINKPROCESSDATARUNNABLE_HPP_

#include "Runnable.hpp"
#include "messaging/data_link.h"
#include "quad_ctrl.h"

class ProcessReceivedDataRunnable : public Runnable {
public:
	void run();
};

class TargetMessageHandlerRunnable : public Runnable {
private:
	target_t *message;
	int32_t last;
	quad_ctrl_t *qc;
public:
	void run();
	
	TargetMessageHandlerRunnable(target_t *target, quad_ctrl_t *qc);
};

class TuningMessageHandlerRunnable : public Runnable {
private:
	tuning_t *message;
	int32_t last;
	quad_ctrl_t *qc;
public:
	void run();
	
	TuningMessageHandlerRunnable(tuning_t *tuning, quad_ctrl_t *qc);
};



#endif /* DATALINKPROCESSDATARUNNABLE_HPP_ */
