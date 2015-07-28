#include <cstdlib>
#include "runnables/CtrlRunnable.hpp"
#include "time_util.h"
#include "ImuDefines.hpp"

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

	//Call the following to generate log info:
	ps->vehicle->prepareLog(vlog, plogs);
//	ps->vehicle->getDjiVals().pitch
//	ps->vehicle->getDjiVals().roll
//	ps->vehicle->getDjiVals().yawRate
//	ps->vehicle->getDjiVals().thrust

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

//	char msg[500];
//	float RotMat[NUM_M_VAL] = {};
//	ps->imu->getRotMat(RotMat);
//
//	char msg[1024];
//	// time, IMU, px4, lidar, camera
//	snprintf(msg, sizeof(msg),
//			"%f\t"
//			"%u\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t"
//			"%f\t%f\t%f\t%f\t",
//			time,
//			ps->mode,
//			ps->imu->getAccX(), ps->imu->getAccY(), ps->imu->getAccZ(),
//			ps->imu->getAngRateX(), ps->imu->getAngRateY(), ps->imu->getAngRateZ(),
//			ps->imu->getMagX(), ps->imu->getMagY(), ps->imu->getMagZ(),
//			RotMat[0], RotMat[1], RotMat[2], RotMat[3], RotMat[4], RotMat[5], RotMat[6], RotMat[7], RotMat[8],
//			ps->px4->getXFlow(), ps->px4->getYFlow(), ps->px4->getQual(),
//			ps->lidar->getDist(),
//			1.0f, 1.0f, 1.0f, 1.0f);
//	ps->sdcard->write(msg, (uint32_t)strlen(msg));
//
//	// filter & PID
//	snprintf(msg, sizeof(msg),
//			"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t%f\t%f\t%f\t%f\t"
//			"%f\t%f\t%f\t%f\t"
//			"%u\t%u\t%u\t%u\t%u\t%u\t%u\t",
//			vlog.xFilt, vlog.yFilt, vlog.zFilt, vlog.xdotFilt, vlog.ydotFilt, vlog.zdotFilt, vlog.rollFilt, vlog.pitchFilt, vlog.yawFilt,
//			plogs[X_AXIS][DOF_VAL].kp, plogs[X_AXIS][DOF_VAL].ki, plogs[X_AXIS][DOF_VAL].kd,
//			plogs[Y_AXIS][DOF_VAL].kp, plogs[Y_AXIS][DOF_VAL].ki, plogs[Y_AXIS][DOF_VAL].kd,
//			plogs[Z_AXIS][DOF_VAL].kp, plogs[Z_AXIS][DOF_VAL].ki, plogs[Z_AXIS][DOF_VAL].kd,
//			plogs[YAW][DOF_VAL].kp, plogs[YAW][DOF_VAL].ki, plogs[YAW][DOF_VAL].kd,
//			plogs[X_AXIS][DOF_RATE].kd, plogs[X_AXIS][DOF_RATE].ki, plogs[X_AXIS][DOF_RATE].kd,
//			plogs[Y_AXIS][DOF_RATE].kd, plogs[Y_AXIS][DOF_RATE].ki, plogs[Y_AXIS][DOF_RATE].kd,
//			plogs[Z_AXIS][DOF_RATE].kd, plogs[Z_AXIS][DOF_RATE].ki, plogs[Z_AXIS][DOF_RATE].kd,
//			plogs[YAW][DOF_RATE].kd, plogs[YAW][DOF_RATE].ki, plogs[YAW][DOF_RATE].kd,
//			plogs[X_AXIS][DOF_VAL].setpt, plogs[Y_AXIS][DOF_VAL].setpt, plogs[Z_AXIS][DOF_VAL].setpt, plogs[YAW][DOF_VAL].setpt,
//			plogs[X_AXIS][DOF_RATE].setpt, plogs[Y_AXIS][DOF_RATE].setpt, plogs[Z_AXIS][DOF_RATE].setpt,
//			vlog.xUval, vlog.yUval, vlog.zUval, plogs[YAW][DOF_RATE].setpt,
//			plogs[X_AXIS][DOF_VAL].flags, plogs[Y_AXIS][DOF_VAL].flags, plogs[Z_AXIS][DOF_VAL].flags,
//			plogs[X_AXIS][DOF_RATE].flags, plogs[Y_AXIS][DOF_RATE].flags, plogs[Z_AXIS][DOF_RATE].flags, plogs[YAW][DOF_RATE].flags);
//	ps->sdcard->write(msg, (uint32_t)strlen(msg));
//
//	// Battery and DJI
//	snprintf(msg, sizeof(msg),
//			"%f\t"
//			"%f\t%f\t%f\t%f\n\n",
//			ps->battery->getVolts(),
//			ps->vehicle->getDjiVals().roll,
//			ps->vehicle->getDjiVals().pitch,
//			ps->vehicle->getDjiVals().yawRate,
//			ps->vehicle->getDjiVals().thrust);
//	ps->sdcard->write(msg, (uint32_t)strlen(msg));

}

