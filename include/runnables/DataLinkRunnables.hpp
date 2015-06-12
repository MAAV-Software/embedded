//TODO: Change QuadCtrl Interface
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
//#include "QuadCtrl.hpp"

class ProcessReceivedDataRunnable : public Runnable {
public:
	void run();

	~ProcessReceivedDataRunnable();
};

class TargetMessageHandlerRunnable : public Runnable {
private:
	target_t *_message;
	int32_t _last;
	//quad_ctrl_t *_qc;
public:
	void run();
	
	//TargetMessageHandlerRunnable(target_t *target, quad_ctrl_t *qc);

	~TargetMessageHandlerRunnable();
};

class TuningMessageHandlerRunnable : public Runnable {
private:
	tuning_t *_message;
	int32_t _last;
	//quad_ctrl_t *_qc;
public:
	void run();
	
	//TuningMessageHandlerRunnable(tuning_t *tuning, quad_ctrl_t *qc);

	~TuningMessageHandlerRunnable();
};



#endif /* DATALINKPROCESSDATARUNNABLE_HPP_ */
