#include <cstdlib>
#include <cstdio>
#include "runnables/CtrlRunnable.hpp"
#include "time_util.h"

using namespace std;

//static float lastTime = 0;
//static char buf[128];

CtrlRunnable::CtrlRunnable(ProgramState* pState) : ps(pState) {}

void CtrlRunnable::run()
{
	float time = ((float)millis()) / 1000.0f; // grab current time

	if (ps->mode == ASSISTED) // set setpts here from rc pilot ctrl in assisted mode
	{
		float setpt[NUM_DOFS][NUM_DOF_STATES];
		for (uint8_t d = 0; d < NUM_DOFS; ++d)
		{
			for (uint8_t s = 0; s < (NUM_DOF_STATES - 1); ++s)
			{
				setpt[d][s] = 0;
			}
			setpt[d][DOF_TIME] = time;
		}
		setpt[X_AXIS][DOF_RATE] = ms2XY_rate(pulse2ms(servoIn_getPulse(RC_CHAN1)));
		setpt[Y_AXIS][DOF_RATE] = ms2XY_rate(pulse2ms(servoIn_getPulse(RC_CHAN2)));
		setpt[Z_AXIS][DOF_VAL] = ms2height(pulse2ms(servoIn_getPulse(RC_CHAN3)));

		ps->vehicle->setSetpt(setpt, ASSISTED);
	}

	// TODO add logic for determining when new camera measurement is ready, then call runFilter with
	// updated arguments for x, y, yawImu, and set withCam to true
	/*
	ps->vehicle->runFilter(0, 0, ps->lidar->getDist(),
						   ps->px4->getXFlow(), ps->px4->getYFlow(),
						   ps->imu->getRoll(), ps->imu->getPitch(),
						   ps->imu->getYaw(), 0, time, false, ps->mode);
						   */

	/*
	float states[NUM_DOFS][NUM_DOF_STATES];
	states[Z_AXIS][DOF_VAL] = ps->lidar->getDist();
	states[Z_AXIS][DOF_RATE] = ps->lidar->getDist() / (time - lastTime);
	states[Z_AXIS][DOF_TIME] = time;
	lastTime = time;
	ps->vehicle->setDofStates(states);
	*/
/*
	ps->vehicle->runCtrl(ps->mode);
	ps->vehicle->prepareLog(vlog, plogs);

	uint32_t len = snprintf(buf, sizeof(buf), "\n\nZs: %f\tUz: %f\tZ: %f\n\n",
							plogs[Z_AXIS][0].setpt, vlog.zUval, vlog.zFilt);
	ps->sdcard->write(buf, len);
*/

	/* Call the following to generate log info:
		ps->vehicle->prepareLog(vlog, plogs)
		ps->vehicle->getDjiVals().pitch
		ps->vehicle->getDjiVals().roll
		ps->vehicle->getDjiVals().yawRate
		ps->vehicle->getDjiVals().thrust
	*/
}

