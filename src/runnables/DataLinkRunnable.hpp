#ifndef DATALINKRUNNABLE_HPP_
#define DATALINKRUNNABLE_HPP_

#include <stdint.h>
#include "Runnable.hpp"
#include "ProgramState.hpp"
#include "messaging/emergency_t.h"
#include "messaging/feedback_t.h"
#include "messaging/gains_t.h"
#include "messaging/raw_pose_t.h"
#include "messaging/setpt_t.h"

class DataLinkRunnable : public Runnable
{
public:
	DataLinkRunnable(ProgramState *pState);
	void run();

private:
	ProgramState *ps;
	int32_t lastSetptTime;
	int64_t lastGainsTime;
	int64_t lastRawPoseTime;
	setpt_t setpt;
	gains_t gains;

	void updateVehicleGains();
	void updateVehicleSetpt();
};

#endif
