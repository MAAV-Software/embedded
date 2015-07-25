#include <cstdlib>
#include "runnables/CtrlRunnable.hpp"
#include "time_util.h"

using namespace std;

CtrlRunnable::CtrlRunnable(ProgramState* pState) : ps(pState) { }

void CtrlRunnable::run()
{
	float time = (float)millis() / 1000.0f; // grab current time

	// TODO add logic for determining when new camera measurement is ready, then call runFilter with
	// updated arguments for x, y, yawImu, and set withCam to true
	ps->vehicle->runFilter(0, 0, ps->lidar->getDist(),
						   ps->px4->getXFlow(), ps->px4->getYFlow(),
						   ps->imu->getRoll(), ps->imu->getPitch(),
						   ps->imu->getYaw(), 0, time, false, ps->mode);
	ps->vehicle->runCtrl(ps->mode);

	/* Call the following to generate log info:
		ps->vehicle->prepareLog(vlog, plogs)
		ps->vehicle->getDjiVals().pitch
		ps->vehicle->getDjiVals().roll
		ps->vehicle->getDjiVals().yawRate
		ps->vehicle->getDjiVals().thrust
	*/
}

