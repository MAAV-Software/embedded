#include <stdint.h>
#include <cstdlib>
#include <cmath>
#include "Vehicle.hpp"
#include "Dof.hpp"
#include "Pid.hpp"
#include "FlightMode.hpp"
#include "MaavMath.hpp"
//#include "servoIn.hpp"
//#include "rc.hpp"

#ifdef LINUX
#include "cmeigen.hpp"
#else
#include "time_util.h"
#include "arm_math.h"
#endif

#ifdef BENCHTOP
#include "uartstdio.h"
#endif

//added
//#ifndef PI
//#define PI 3.14159265358979f
//#endif

using namespace std;
using MaavMath::mat_at;

// define pi and gravity
#ifndef PI
static const float PI = 3.14159265358979323846;
#endif
//static const float GRAVITY = 9.81;

Vehicle::Vehicle(const float valueGains[NUM_DOFS][NUM_PID_GAINS],
				 const float rateGains[NUM_DOFS][NUM_PID_GAINS])
	: lastPredictTime(0),
	lastLidarTime(0),
	lastPx4Time(0),
	lastCameraTime(0),
	first(true)
{
	mass 		 = 2.38f;
	dji.roll     = 0;
	dji.pitch    = 0;
	dji.yawRate  = 0;
	dji.thrust   = 0;

	// Ctrl Vals
	// state and setpt will always be initialized the to 0 and time
	float states[NUM_DOFS][NUM_DOF_STATES] = {
		{0, 0, 0, lastPredictTime},
		{0, 0, 0, lastPredictTime},
		{0, 0, 0, lastPredictTime},
		{0, 0, 0, lastPredictTime},
	};
	float setpts[NUM_DOFS][NUM_DOF_STATES] = {
		{0, 0, 0, lastPredictTime},
		{0, 0, 0, lastPredictTime},
		{0, 0, 0, lastPredictTime},
		{0, 0, 0, lastPredictTime},
	};
	uint8_t valueFlags[NUM_DOFS] = {
		DERR_DT_MASK,
		DERR_DT_MASK,
		DERR_DT_MASK,
		DERR_DT_MASK | DISC_DERIV_MASK | WRAP_AROUND_MASK,
	};
	uint8_t rateFlags[NUM_DOFS] = {
		DERR_DT_MASK | DISC_DERIV_MASK,
		DERR_DT_MASK | DISC_DERIV_MASK,
		DERR_DT_MASK | DISC_DERIV_MASK,
		DERR_DT_MASK | DISC_DERIV_MASK,
	};
	float stateBounds[4] = {0, 0, 0, MaavMath::Pi};
	float rateUpLims[4]  = {2, 2, 10, 1};
	float rateLwLims[4]  = {-2, -2, -10, -1};
	float accelUpLims[4] = {1, 1, 10, 1};
	float accelLwLims[4] = {-1, -1, -10, -1};

	// todo for now, lp will be set to 0 and disabled.
	float valueStateLpCoeffs[NUM_DOFS] = {0, 0, 0, 0};
	float valueErrorLpCoeffs[NUM_DOFS] = {0, 0, 0, 0};
	float rateStateLpCoeffs[NUM_DOFS]  = {0, 0, 0, 0};
	float rateErrorLpCoeffs[NUM_DOFS]  = {0, 0, 0, 0};

	//for (int i = 0; i < NUM_ANGLES; ++i) rpLimits[i] = PI / 4.0f;
	for (int i = 0; i < NUM_ANGLES; ++i) rpLimits[i] = 0.5;

	for (int i = 0; i < NUM_DOFS; ++i)
	{
		dofs[i] = Dof(states[i], setpts[i], valueGains[i], rateGains[i],
					  valueFlags[i], rateFlags[i], 2.5f, stateBounds[i],
					  rateUpLims[i], rateLwLims[i], accelUpLims[i],
					  accelLwLims[i], valueStateLpCoeffs[i],
					  valueErrorLpCoeffs[i], rateStateLpCoeffs[i],
					  rateErrorLpCoeffs[i]);
	}

	//initializing Q and R matrices
	//                x    xdot  y    ydot  z     zdot
	kalmanFilter.setQ(0.1, 0.01, 0.1, 0.01, 0.07, 0.9);
	//                      z        zdot
	kalmanFilter.setR_lidar(0.47236, 0.47236);
	//                    xdot ydot
	kalmanFilter.setR_Px4(0.1, 0.1);
	//camera not in use
	kalmanFilter.setR_camera(0.0, 0.0);



}

Vehicle::~Vehicle()
{

}

// Vehicle::Vehicle(const float states[NUM_DOFS][NUM_DOF_STATES],
// 				 const float setpts[NUM_DOFS][NUM_DOF_STATES],
// 				 const float valueGains[NUM_DOFS][NUM_PID_GAINS],
// 				 const float rateGains[NUM_DOFS][NUM_PID_GAINS],
// 				 const uint8_t valueFlags[NUM_DOFS],
// 				 const uint8_t rateFlags[NUM_DOFS],
// 				 const float inertias[NUM_DOFS],
// 				 const float stateBounds[NUM_DOFS],
// 				 const float rateUpLims[NUM_DOFS],
// 				 const float rateLwLims[NUM_DOFS],
// 				 const float accelUpLims[NUM_DOFS],
// 				 const float accelLwLims[NUM_DOFS],
// 				 const float valueStateLpCoeffs[NUM_DOFS],
// 				 const float valueErrorLpCoeffs[NUM_DOFS],
// 				 const float rateStateLpCoeffs[NUM_DOFS],
// 				 const float rateErrorLpCoeffs[NUM_DOFS],
// 				 const float totalMass,
// 				 const float initTime,
// 				 const float rpLims[NUM_ANGLES],
// {
// 	mass = totalMass;
// 	time = initTime;
// 	lastPredTime = initTime;
// 	lastCorrTime = initTime;
// 	dji.roll    = 0;
// 	dji.pitch   = 0; 
// 	dji.yawRate = 0;
// 	dji.thrust  = 0;
	
// 	for (int i = 0; i < NUM_ANGLES; ++i) rpLimits[i] = rpLims[i];

// 	for (int i = 0; i < NUM_DOFS; ++i)
// 	{
// 		dofs[i] = Dof(states[i], setpts[i], valueGains[i], rateGains[i],
// 					  valueFlags[i], rateFlags[i], inertias[i], stateBounds[i],
// 					  rateUpLims[i], rateLwLims[i], accelUpLims[i], 
// 					  accelLwLims[i], valueStateLpCoeffs[i], 
// 					  valueErrorLpCoeffs[i], rateStateLpCoeffs[i],
// 					  rateErrorLpCoeffs[i]);
// 	}
// }

// updates sensor inputs, runs the EKF, and updates the states in the DOFs
void Vehicle::runFilter(const float rotationMatrix[9], float yaw,
			float imuX, float imuY, float imuZ, float currTime,
			float lidar, float lidarTime,
			float px4X, float px4Y, float px4Time, 
			float cameraX, float cameraY, float cameraTime) 
{
	if(first)
	{
		first = false;
		lastPredictTime = currTime;
		return;
	}
#ifdef BENCHTOP
	UARTprintf("Running Filter at t = %3.2fs ", currTime);
#endif

	currYawSin = arm_sin_f32(yaw);
	currYawCos = arm_cos_f32(yaw);

	// new IMU measurement
	float imuArenaX, imuArenaY, imuArenaZ;
	MaavMath::applyTransRotMatrix(rotationMatrix, imuX, imuY, imuZ,
		imuArenaX, imuArenaY, imuArenaZ);

	if (!MaavMath::floatClose(currTime, lastPredictTime, 0.001))
	{
#ifdef BENCHTOP
	UARTprintf(" Predict ");
#endif
		kalmanFilter.predict(imuArenaX, imuArenaY, imuArenaZ + MaavMath::Gravity, currTime - lastPredictTime);
		lastPredictTime = currTime;
	}

	// new lidar measurement
	if (lidarTime != lastLidarTime) 
	{
#ifdef BENCHTOP
	UARTprintf(" Correct Lidar ");
#endif
		float lidarArenaX, lidarArenaY, lidarArenaZ;
		MaavMath::applyTransRotMatrix(rotationMatrix, 0, 0, -lidar,
			lidarArenaX, lidarArenaY, lidarArenaZ);

		kalmanFilter.correctLidar(lidarArenaZ, lidarArenaZ - lastLidarArenaZ);

		lastLidarArenaZ = lidarArenaZ;
		lastLidarTime = lidarTime;
	}

	// new px4 measurement
	if (px4Time != lastPx4Time) 
	{

#ifdef BENCHTOP
	UARTprintf(" Correct Px4 ");
#endif
		float px4ArenaX = currYawCos * px4X + currYawSin * px4Y;
		float px4ArenaY = -currYawSin * px4X + currYawCos * px4Y;

		kalmanFilter.correctPx4(px4ArenaX, px4ArenaY);

		lastPx4Time = px4Time;
	}

	// new camera measurement
	if (cameraTime != lastCameraTime) 
	{
#ifdef BENCHTOP
	UARTprintf(" Correct Camera ");
#endif
		kalmanFilter.correctCamera(cameraX, cameraY);
		lastCameraTime = cameraTime;
	}

#ifdef BENCHTOP
	UARTprintf(" Updating State\n");
#endif


	// // extract state and send to dofs
	const arm_matrix_instance_f32& filterState = kalmanFilter.getState();
	float state[NUM_DOFS][NUM_DOF_STATES];
	for (int i = 0; i < NUM_DOFS; ++i) state[i][DOF_TIME] = currTime;
	state[X_AXIS][DOF_VAL]   = filterState.pData[0];
	state[X_AXIS][DOF_RATE]  = filterState.pData[1];
	state[X_AXIS][DOF_ACCEL] = imuArenaX;
	state[Y_AXIS][DOF_VAL]   = filterState.pData[2];
	state[Y_AXIS][DOF_RATE]  = filterState.pData[3];
	state[Y_AXIS][DOF_ACCEL] = imuArenaY;
	state[Z_AXIS][DOF_VAL]   = filterState.pData[4];
	state[Z_AXIS][DOF_RATE]  = filterState.pData[5];
	state[Z_AXIS][DOF_ACCEL] = imuArenaZ;
	state[YAW][DOF_VAL]      = yaw;
	state[YAW][DOF_RATE]     = 0;
	state[YAW][DOF_ACCEL]    = 0;
	setDofStates(state);
}

// Executes all PID control for all DOFs based on flight mode and calcs DJI vals
void Vehicle::runCtrl(const FlightMode mode)
{
	switch (mode) // mode only affects x, y dofs
	{
		case AUTONOMOUS: 
			dofs[X_AXIS].run(false);
			dofs[Y_AXIS].run(false);
			break;
		case ASSISTED: 	 
			dofs[X_AXIS].run(true);
			dofs[Y_AXIS].run(true);
			break;
		case MANUAL:
			return;
		default:
			break;
	}
	
	// always do full cascade for z and yaw
	dofs[Z_AXIS].run(false);
	//dofs[Z_AXIS].runZeroVel();

	dofs[YAW].run(false);

	// finally get dji values
	calcDJIValues(mode);
}

// Assigns setpoints based on the controller mode
void Vehicle::setSetpt(const float setpt[NUM_DOFS][NUM_DOF_STATES], 
					   const FlightMode mode)
{
	switch (mode) // mode only affects x, y dofs
	{
		case AUTONOMOUS: 
			dofs[X_AXIS].setSetpt(setpt[X_AXIS], false);
			dofs[Y_AXIS].setSetpt(setpt[Y_AXIS], false);
			break;
		case ASSISTED: 	 
			dofs[X_AXIS].setSetpt(setpt[X_AXIS], true);
			dofs[Y_AXIS].setSetpt(setpt[Y_AXIS], true);
			break;
		case MANUAL:
		    return; // don't set any setpts since we're passing RC Pilot direclty to DJI
		default:
			break;
	}
	
	// always do Z and Yaw val setpt
	dofs[Z_AXIS].setSetpt(setpt[Z_AXIS], false);
	dofs[YAW].setSetpt(setpt[YAW], false);
}
	
// Assigns gains to the DOFs within this Vehicle
void Vehicle::setGains(const float valueGains[NUM_DOFS][NUM_PID_GAINS], 
					   const float rateGains[NUM_DOFS][NUM_PID_GAINS])
{
	for (int i = 0; i < NUM_DOFS; ++i) 
		dofs[i].setGains(valueGains[i], rateGains[i]);
}

void Vehicle::setQR(const float qx, const float qxd, 
		const float qy, const float qyd,
		const float qz, const float qzd,
		const float rlidarz, const float rlidarzd,
		const float rpx4xd, const float rpx4yd,
		const float rcamx, const float rcamy)
{
	kalmanFilter.setQ(qx, qxd, qy, qyd, qz, qzd);
	kalmanFilter.setR_lidar(rlidarz, rlidarzd);
	kalmanFilter.setR_Px4(rpx4xd, rpx4yd);
	kalmanFilter.setR_camera(rcamx, rcamy);
}

void Vehicle::setDofStates(const float state[NUM_DOFS][NUM_DOF_STATES])
{
	for (int i = 0; i < NUM_DOFS; ++i) dofs[i].setState(state[i]);
}


void Vehicle::calcDJIValues(const FlightMode mode)
{
	float forceVe[3];        // vehicle force in the earth frame (in R^3)
	float forceVy[3];        // vehicle force in the yaw frame (in R^3)
	float forceMag = 0;      // magnitude of the vehicle force
	float angle[NUM_ANGLES]; // roll and pitch setpoint for dji

	// get earth frame vehicle forces
	//for (int i = 0; i < 3; ++i) forceVe[i] = dofs[i].getUval();
	//forceVe[Z_AXIS] += mass * GRAVITY;

	for (int i = 0; i < 2; ++i) forceVe[i] = dofs[i].getUval();

	forceVe[Z_AXIS] = (dofs[Z_AXIS].getRate() * mass) + (mass * MaavMath::Gravity);

	// Convert earth frame forces to body frame
	forceVy[X_AXIS] =  (currYawCos * forceVe[X_AXIS]) + (currYawSin * forceVe[Y_AXIS]);
	forceVy[Y_AXIS] = -(currYawSin * forceVe[X_AXIS]) + (currYawCos * forceVe[Y_AXIS]);
	forceVy[Z_AXIS] = forceVe[Z_AXIS];

	// calculate ||F|| = sqrt(Fx^2 + Fy^2 + Fz^2) (L2-Norm)
	for (int i = 0; i < 3; ++i) forceMag += forceVy[i] * forceVy[i];
	float tmp = forceMag;
	arm_sqrt_f32(tmp, &forceMag);

	// Calculate roll and pitch
	angle[ROLL]  = -asin(forceVy[Y_AXIS] / forceMag);
	tmp = (forceMag * forceMag) - (forceVy[Y_AXIS] * forceVy[Y_AXIS]);
	float res = 0;
	arm_sqrt_f32(tmp, &res);
	angle[PITCH] =  asin(forceVy[X_AXIS] / res);

	// cap roll and pitch
	if (angle[ROLL]  >  rpLimits[ROLL])  angle[ROLL]  =  rpLimits[ROLL];
	if (angle[ROLL]  < -rpLimits[ROLL])  angle[ROLL]  = -rpLimits[ROLL];
	if (angle[PITCH] >  rpLimits[PITCH]) angle[PITCH] =  rpLimits[PITCH];
	if (angle[PITCH] < -rpLimits[PITCH]) angle[PITCH] = -rpLimits[PITCH];

	// assign DJI values
	dji.roll    = (mode == MANUAL) ? 0.0 : angle[ROLL];
	dji.pitch   = (mode == MANUAL) ? 0.0 : angle[PITCH];
	//dji.thrust = dofs[Z_AXIS].getUval();
	//dji.thrust = dofs[Z_AXIS].getRate();
	dji.thrust  = (mode == MANUAL) ? 0.0 : forceVe[Z_AXIS];

	dji.yawRate = (mode == MANUAL) ? 0.0 : dofs[YAW].getRate();
}


Dji Vehicle::getDjiVals() const
{
	return dji;
}

void Vehicle::prepareLog(VehicleLog &vlog, PidLog plogs[NUM_DOFS][2])
{	
	// grab dof and pid logs
	for (int i = 0; i < NUM_DOFS; ++i) dofs[i].prepareLog(plogs[i]);
	
	// grab ekf state
	arm_matrix_instance_f32 kfState = kalmanFilter.getState();
	
	// fill vehicle log
	vlog.xUval	   = dofs[X_AXIS].getUval();
	vlog.yUval     = dofs[Y_AXIS].getUval();
	vlog.zUval     = dofs[Z_AXIS].getRate() * mass;
	vlog.xFilt     = mat_at(kfState, 0, 0);
	vlog.xdotFilt  = mat_at(kfState, 0, 1);
	vlog.yFilt     = mat_at(kfState, 0, 2);
	vlog.ydotFilt  = mat_at(kfState, 0, 3);
	vlog.zFilt     = mat_at(kfState, 0, 4);
	vlog.zdotFilt  = mat_at(kfState, 0, 5);
	vlog.rollFilt  = 0;
	vlog.pitchFilt = 0;
	vlog.yawFilt   = 0;
}

// End of File
