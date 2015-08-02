#include <cstdlib>
#include <cstdio>
#include "runnables/CtrlRunnable.hpp"
#include "time_util.h"
#include "ImuDefines.hpp"
#include "servoIn.hpp"
#include "rc.hpp"

using namespace std;

CtrlRunnable::CtrlRunnable(ProgramState* pState) : ps(pState) {}

void CtrlRunnable::run()
{
	float time = ((float)millis()) / 1000.0f; // grab current time
/*
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
*/
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
*/
/*
	ps->vehicle->prepareLog(vlog, plogs);

    ps->feedback->utime = time;
    ps->feedback->roll  = vlog.rollFilt;
    ps->feedback->pitch = vlog.pitchFilt;
    ps->feedback->yaw   = vlog.yawFilt;
    ps->feedback->x[0]  = vlog.xFilt;
    ps->feedback->x[1]  = vlog.xdotFilt;
    ps->feedback->x[2]  = 0;
    ps->feedback->y[0]  = vlog.yFilt;
    ps->feedback->y[1]  = vlog.ydotFilt;
    ps->feedback->y[2]  = 0;
    ps->feedback->z[0]  = vlog.zFilt;
    ps->feedback->z[1]  = vlog.zdotFilt;
    ps->feedback->z[2]  = 0;
    ps->feedback->flags = ps->dLink->getSetptMsg().flags;
    ps->dLink->send(ps->feedback);
*/

/* Log msg structure
 * Time: 		t
 * Mode: 		fm
 * Imu_Accel: 	ax ay za
 * Imu_Gyro: 	gx gy gz
 * Imu_Mag: 	mx my mz
 * Imu_Rot:		r0 .. r8
 * Px4: 		xdot ydot qual
 * Lidar:		dist
 * camera:		x y yaw t
 * FilterState:	x y z dx dy dz roll pitch yaw
 * valGains:	Px Ix Dx Py Iy Dy Pz Iz Dz Pyaw Iyaw Dyaw
 * rateGains: 	Pdx Idx Ddx Pdy Idy Ddy Pdz Idz Ddz Pdyaw Idyaw Ddyaw
 * setpts:		x y z yaw dx dy dz
 * PIDout:		ux uy uz dyaw
 * Flags:		xval yval zval xrate yrate zrate yawrate
 * atomflag:
 * battery:		vols
 * DJI:			roll pitch dyaw Fz
 */

	float RotMat[NUM_M_VAL];
	ps->imu->getRotMat(RotMat);

	char msg[1024];
	// time, IMU, px4, lidar, camera
	uint32_t len = snprintf(msg, sizeof(msg),
			"%f\t"
			"%u\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t"
			"%f\t%f\t%f\t%f\t"
			"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t"
			"%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
			"%f\t%f\t%f\t%f\t"
			"%u\t%u\t%u\t%u\t%u\t%u\t%u\t"
			"%f\t"
			"%f\t%f\t%f\t%f\n",
			time,
			ps->mode,
			ps->imu->getAccX(), ps->imu->getAccY(), ps->imu->getAccZ(),
			ps->imu->getAngRateX(), ps->imu->getAngRateY(), ps->imu->getAngRateZ(),
			ps->imu->getMagX(), ps->imu->getMagY(), ps->imu->getMagZ(),
			RotMat[0], RotMat[1], RotMat[2], RotMat[3], RotMat[4], RotMat[5], RotMat[6], RotMat[7], RotMat[8],
			ps->px4->getXFlow(), ps->px4->getYFlow(), ps->px4->getQual(),
			ps->lidar->getDist(),
			1.0f, 2.0f, 3.0f, 4.0f,
			vlog.xFilt, vlog.yFilt, vlog.zFilt, vlog.xdotFilt, vlog.ydotFilt, vlog.zdotFilt, vlog.rollFilt, vlog.pitchFilt, vlog.yawFilt,
			plogs[X_AXIS][DOF_VAL].kp, plogs[X_AXIS][DOF_VAL].ki, plogs[X_AXIS][DOF_VAL].kd,
			plogs[Y_AXIS][DOF_VAL].kp, plogs[Y_AXIS][DOF_VAL].ki, plogs[Y_AXIS][DOF_VAL].kd,
			plogs[Z_AXIS][DOF_VAL].kp, plogs[Z_AXIS][DOF_VAL].ki, plogs[Z_AXIS][DOF_VAL].kd,
			plogs[YAW][DOF_VAL].kp, plogs[YAW][DOF_VAL].ki, plogs[YAW][DOF_VAL].kd,
			plogs[X_AXIS][DOF_RATE].kd, plogs[X_AXIS][DOF_RATE].ki, plogs[X_AXIS][DOF_RATE].kd,
			plogs[Y_AXIS][DOF_RATE].kd, plogs[Y_AXIS][DOF_RATE].ki, plogs[Y_AXIS][DOF_RATE].kd,
			plogs[Z_AXIS][DOF_RATE].kd, plogs[Z_AXIS][DOF_RATE].ki, plogs[Z_AXIS][DOF_RATE].kd,
			plogs[YAW][DOF_RATE].kd, plogs[YAW][DOF_RATE].ki, plogs[YAW][DOF_RATE].kd,
			plogs[X_AXIS][DOF_VAL].setpt, plogs[Y_AXIS][DOF_VAL].setpt, plogs[Z_AXIS][DOF_VAL].setpt, plogs[YAW][DOF_VAL].setpt,
			plogs[X_AXIS][DOF_RATE].setpt, plogs[Y_AXIS][DOF_RATE].setpt, plogs[Z_AXIS][DOF_RATE].setpt,
			vlog.xUval, vlog.yUval, vlog.zUval, plogs[YAW][DOF_RATE].setpt,
			plogs[X_AXIS][DOF_VAL].flags, plogs[Y_AXIS][DOF_VAL].flags, plogs[Z_AXIS][DOF_VAL].flags,
			plogs[X_AXIS][DOF_RATE].flags, plogs[Y_AXIS][DOF_RATE].flags, plogs[Z_AXIS][DOF_RATE].flags, plogs[YAW][DOF_RATE].flags,
			ps->battery->getVolts(),
			servoIn_getPulse(RC_CHAN2),
			servoIn_getPulse(RC_CHAN1),
			servoIn_getPulse(RC_CHAN4),
			servoIn_getPulse(RC_CHAN3));
	ps->sdcard->write(msg, len);
}

