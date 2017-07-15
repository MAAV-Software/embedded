#include <cstdlib>
#include <cstdio>
#include "runnables/CtrlRunnable.hpp"
#include "time_util.h"
#include "ImuDefines.hpp"
#include "servoIn.hpp"
#include "rc.hpp"
#include "arm_math.h"

using namespace std;

CtrlRunnable::CtrlRunnable(ProgramState* pState)
{
	ps = pState;
	lastPoseTime = 0;
	lastTime = 0;
}

void CtrlRunnable::run()
{
	uint32_t msTime = millis();
	float time = ((float)msTime) / 1000.0f; // grab current time
	float poseTimestamp = (float)millis() / 1000.0f;

	if ((ps->mode == ASSISTED) || (ps->mode == MANUAL)) // set setpts here from rc pilot ctrl in assisted mode
	{
		// TODO: store these in something other than an array indexed by DOF related stuff
		// treat the pilot input like input from the Atom
		// Maybe just call Vehicle setControlInput ???
		// fx, fy, yaw, fz
//		ps->spArr[X_AXIS][DOF_RATE] = ms2XY_rate(ps->pilot->pulse((servoIn_getPulse(RC_CHAN1)), 0));
//		ps->spArr[Y_AXIS][DOF_RATE] = ms2XY_rate(ps->pilot->pulse((servoIn_getPulse(RC_CHAN2)), 1));
//		ps->spArr[Z_AXIS][DOF_VAL]  = ms2height(ps->pilot->pulse((servoIn_getPulse(RC_CHAN3)), 2)); // don't negate this here

		//ps->vehicle->setSetpt(setpt, ASSISTED, false);
	}
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

	Dji dji = ps->vehicle->getDjiVals();

//	uint32_t throttle = (uint32_t)map(dji.thrust, 0, 46.6956, 114000, 124000);
//	if (throttle < 75700)
//		throttle = 75700;
//	else if (throttle > 153300)
//		throttle = 153300;

	uint32_t throttle = (uint32_t)map(dji.thrust, 0, 46.6956, ps->djiout->dutyCycle(0.09, 2), ps->djiout->dutyCycle(1.00, 2));
	if (throttle < ps->djiout->dutyCycle(0.09, 2)) throttle = ps->djiout->dutyCycle(0.09, 2);
	else if (throttle > ps->djiout->dutyCycle(1.00, 2)) throttle = ps->djiout->dutyCycle(1.00, 2);

/*
	uint32_t ppmYawRate = (uint32_t)map(dji.yawRate, -1, 1, 113000, 120400);
	uint32_t ppmPitch = (uint32_t)map(dji.pitch, -0.5, 0.5, 105600, 135000);
	uint32_t ppmRoll  = (uint32_t)map(dji.roll, -0.5, 0.5, 104200, 135000);
*/

	uint32_t ppmYawRate = (uint32_t)map(dji.yawRate, -1, 1, ps->djiout->dutyCycle(0.1, 3), ps->djiout->dutyCycle(0.9, 3));
	if (ppmYawRate < ps->djiout->dutyCycle(0.1, 3)) ppmYawRate = ps->djiout->dutyCycle(0.1, 3);
	else if (ppmYawRate > ps->djiout->dutyCycle(0.9, 3)) ppmYawRate = ps->djiout->dutyCycle(0.9, 3);

	//uint32_t ppmPitch = (uint32_t)map(dji.pitch, -0.5, 0.5, 105600, 135000);
	uint32_t ppmPitch = (uint32_t)map(dji.pitch, -0.5, 0.5, ps->djiout->dutyCycle(0.00, 0), ps->djiout->dutyCycle(1.00, 0));
	if (ppmPitch < ps->djiout->dutyCycle(0.00, 0)) ppmPitch = ps->djiout->dutyCycle(0.00, 0);
	else if (ppmPitch > ps->djiout->dutyCycle(1.00, 0)) ppmPitch = ps->djiout->dutyCycle(1.00, 0);


	//uint32_t ppmRoll  = (uint32_t)map(dji.roll, -0.5, 0.5, 104200, 135000);
	uint32_t ppmRoll  = (uint32_t)map(dji.roll, -0.5, 0.5, ps->djiout->dutyCycle(0.00, 1), ps->djiout->dutyCycle(1.00, 1));
	if (ppmRoll < ps->djiout->dutyCycle(0.00, 1)) ppmRoll = ps->djiout->dutyCycle(0.00, 1);
	else if (ppmRoll > ps->djiout->dutyCycle(1.00, 1)) ppmRoll = ps->djiout->dutyCycle(1.00, 1);


//	char msg[1024];
//	// time, IMU, px4, lidar, camera
//	uint32_t len = snprintf(msg, sizeof(msg),
//			"%f\t"//Time
//			"%u\t"//Mode
//			"%f\t%f\t%f\t"//Acc X,Y,Z
//			"%f\t%f\t%f\t"//AngRate X,Y,Z
//			"%f\t%f\t%f\t"//Mag X,Y,Z
//			"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"//RotMat (9 elements)
//			"%f\t%f\t%f\t"//Px4Flow X,Y,Quality
//			"%f\t"//Dist
//			"%f\t%f\t%f\t%lu\t"//Pose message
//			"%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t"//Filtered states, X,Y,Z,Xd,Yd,Zd Imu X,Y,Z
//			"%f\t%f\t%f\t"//KPID Start
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"
//			"%f\t%f\t%f\t"//KPID End
//			"%f\t%f\t%f\t%f\t%f\t%f\t%f\t"//VAL SETPT X,Y,Z,YAW RATE SETPT X,Y,Z
//			"%f\t%f\t%f\t%f\t"//UVALS X,Y,Z RATE SETP YAW
//			"%u\t%u\t%u\t%u\t%u\t%u\t%u\t"//Flags
//			"%f\t"//Battery Volts
//			"%u\t%u\t%u\t%u\t"//RC Pilot Duty Cycles
//			"%f\t"//Setpt Message Flags
//			"%f\t%f\t%f\t%f\t"//DJI RPFzY
//			"%u\t%u\t%u\t%u\t"//PPM RPFzY
//			"%u\t"//msTime
//			"%f\t%f\t%f\t%f\t%f\t%u\n",//
//			time,
//			ps->mode,
//			ps->imu->getAccX(), ps->imu->getAccY(), ps->imu->getAccZ(),
//			ps->imu->getAngRateX(), ps->imu->getAngRateY(), ps->imu->getAngRateZ(),
//			ps->imu->getMagX(), ps->imu->getMagY(), ps->imu->getMagZ(),
//			RotMat[0], RotMat[1], RotMat[2], RotMat[3], RotMat[4], RotMat[5], RotMat[6], RotMat[7], RotMat[8],
////			ps->px4->getXFlow(), ps->px4->getYFlow(), ps->px4->getQual(),
//			ps->lidar->getDist(),
			//dji.thrust,
			//1.0f, 2.0f, 3.0f, 4.0f,
//			ps->dLink->getRawPoseMsg().x, ps->dLink->getRawPoseMsg().y, ps->dLink->getRawPoseMsg().yaw, ps->dLink->getRawPoseMsg().utime, // this last time value is our tiva time
//
//			//vlog.xFilt, vlog.yFilt, vlog.zFilt, vlog.xdotFilt, vlog.ydotFilt, vlog.zdotFilt, vlog.rollFilt, vlog.pitchFilt, vlog.yawFilt,
//			vlog.xFilt, vlog.yFilt, vlog.zFilt, vlog.xdotFilt, vlog.ydotFilt, vlog.zdotFilt, ps->imu->getRoll(), ps->imu->getPitch(), ps->imu->getYaw(),
//
//			plogs[X_AXIS][DOF_VAL].kp, plogs[X_AXIS][DOF_VAL].ki, plogs[X_AXIS][DOF_VAL].kd,
//			plogs[Y_AXIS][DOF_VAL].kp, plogs[Y_AXIS][DOF_VAL].ki, plogs[Y_AXIS][DOF_VAL].kd,
//			plogs[Z_AXIS][DOF_VAL].kp, plogs[Z_AXIS][DOF_VAL].ki, plogs[Z_AXIS][DOF_VAL].kd,
//			plogs[YAW][DOF_VAL].kp, plogs[YAW][DOF_VAL].ki, plogs[YAW][DOF_VAL].kd,
//			plogs[X_AXIS][DOF_RATE].kd, plogs[X_AXIS][DOF_RATE].ki, plogs[X_AXIS][DOF_RATE].kd,
//			plogs[Y_AXIS][DOF_RATE].kd, plogs[Y_AXIS][DOF_RATE].ki, plogs[Y_AXIS][DOF_RATE].kd,
//			plogs[Z_AXIS][DOF_RATE].kd, plogs[Z_AXIS][DOF_RATE].ki, plogs[Z_AXIS][DOF_RATE].kd,
//			plogs[YAW][DOF_RATE].kd, plogs[YAW][DOF_RATE].ki, plogs[YAW][DOF_RATE].kd,
//
//			//ps->dLink->getSetptMsg().x, ps->dLink->getSetptMsg().y, ps->dLink->getSetptMsg().z, ps->dLink->getSetptMsg().yaw,
//			plogs[X_AXIS][DOF_VAL].setpt, plogs[Y_AXIS][DOF_VAL].setpt, plogs[Z_AXIS][DOF_VAL].setpt, plogs[YAW][DOF_VAL].setpt,
//
//			plogs[X_AXIS][DOF_RATE].setpt, plogs[Y_AXIS][DOF_RATE].setpt, plogs[Z_AXIS][DOF_RATE].setpt,
//			vlog.xUval, vlog.yUval, vlog.zUval, plogs[YAW][DOF_RATE].setpt,
//			plogs[X_AXIS][DOF_VAL].flags, plogs[Y_AXIS][DOF_VAL].flags, plogs[Z_AXIS][DOF_VAL].flags,
//			plogs[X_AXIS][DOF_RATE].flags, plogs[Y_AXIS][DOF_RATE].flags, plogs[Z_AXIS][DOF_RATE].flags, plogs[YAW][DOF_RATE].flags,
//			ps->battery->getVolts(),
//			ps->pilot->dutyCycle(servoIn_getPulse(RC_CHAN2), 1),
//			ps->pilot->dutyCycle(servoIn_getPulse(RC_CHAN1), 0),
//			ps->pilot->dutyCycle(servoIn_getPulse(RC_CHAN4), 3),
//			ps->pilot->dutyCycle(servoIn_getPulse(RC_CHAN3), 2),
//			dji.pitch, dji.roll, dji.thrust, dji.yawRate,
//			ppmRoll, ppmPitch, throttle, ppmYawRate,
//			msTime,
//			ps->lidar->getTimestamp(),
//			ps->imu->getTimestamp(),
//			poseTimestamp,
//			ps->imu->getRefYaw(),
//			ps->vehicle->getRCInputError());
//	ps->sdcard->write(msg, len);
}

