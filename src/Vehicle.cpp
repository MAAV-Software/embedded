#include <stdint.h>
#include <cstdlib>
#include <cmath>
#include "Vehicle.hpp"
#include "Dof.hpp"
#include "Pid.hpp"
#include "FlightMode.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/KalmanFunctions.hpp"
//#include "servoIn.hpp"
//#include "rc.hpp"

#ifdef LINUX
#include "cmeigen.hpp"
#else
#include "time_util.h"
#include "arm_math.h"
#endif

//added
//#ifndef PI
//#define PI 3.14159265358979f
//#endif

using namespace std;

// define pi and gravity only if not yet defined
#ifndef PI
static const float PI = 3.14159265358979323846;
#endif //PI
static const float GRAVITY = 9.81;

Vehicle::Vehicle(const float valueGains[NUM_DOFS][NUM_PID_GAINS],
				 const float rateGains[NUM_DOFS][NUM_PID_GAINS])
{
	#ifdef LINUX
		time = 0.0f;
	#else
		time 		 = (float)millis() / 1000.0f; // grab current time for initialization
	#endif
	mass 		 = 2.38f;
	lastPredTime = time;
	lastCorrTime = time;
	dji.roll     = 0;
	dji.pitch    = 0;
	dji.yawRate  = 0;
	dji.thrust   = 0;

	// Ctrl Vals
	// state and setpt will always be initialized the to 0 and time
	float states[NUM_DOFS][NUM_DOF_STATES] = {
		{0, 0, 0, time},
		{0, 0, 0, time},
		{0, 0, 0, time},
		{0, 0, 0, time},
	};
	float setpts[NUM_DOFS][NUM_DOF_STATES] = {
		{0, 0, 0, time},
		{0, 0, 0, time},
		{0, 0, 0, time},
		{0, 0, 0, time},
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
	float stateBounds[4] = {0, 0, 0, PI};
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

	// EKF values
	float ekfInitState[6] = {0, 0, 0, 0, 0, 0};
	float ekfInitP[36] = {
		0.1, 0, 0, 0, 0, 0,
		0, 0.1, 0, 0, 0, 0,
		0, 0, 0.1, 0, 0, 0,
		0, 0, 0, 0.1, 0, 0,
		0, 0, 0, 0, 0.1, 0,
		0, 0, 0, 0, 0, 0.1
	};
	float ekfQ[36] = {
		0.1, 0, 0, 0, 0, 0,
		0, 0.1, 0, 0, 0, 0,
		0, 0, 0.9, 0, 0, 0,
		0, 0, 0, 0.01, 0, 0,
		0, 0, 0, 0, 0.01, 0,
		0, 0, 0, 0, 0, 0.01
	};
	float ekfNoCamR[9] = {
		0.02, 0, 0,
		0, 0.1, 0,
		0, 0, 0.1
	};
	float ekfWithCamR[25] = {
		0.1, 0, 0, 0, 0,
		0, 0.1, 0, 0, 0,
		0, 0, 0.02, 0, 0,
		0, 0, 0, 0.1, 0,
		0, 0, 0, 0, 0.1
	};

	// ekfInitP is the ekfInitErrorCov
	ekf = new ExtendedKalmanFilter(6, ekfInitState, ekfInitP);

	arm_matrix_instance_f32 Q;
	arm_mat_init_f32(&Q, 6, 6, ekfQ);
	ekf->setPredictFunc(4, systemDeltaState, systemGetJacobian, &Q);

	arm_matrix_instance_f32 RNoCam, RWithCam;
	arm_mat_init_f32(&RNoCam, 3, 3, ekfNoCamR);
	arm_mat_init_f32(&RWithCam, 5, 5, ekfWithCamR);
	ekf->setUpdateFunc(0, 3, sensorPredict, sensorGetJacobian, &RNoCam);
	ekf->setUpdateFunc(1, 5, sensorPredictWithCam, sensorGetJacobianWithCam, &RWithCam);

	float controlInput[4] = {0, 0, 0, 0};
	arm_mat_init_f32(&controlInputMat, 4, 1, controlInput);

	float sensorMeasurement[3] = {0, 0, 0};
	arm_mat_init_f32(&sensorMeasurementMat, 3, 1, sensorMeasurement);

	float sensorMeasurementWithCam[5] = {0, 0, 0, 0, 0};
	arm_mat_init_f32(&sensorMeasurementMatWithCam, 5, 1, sensorMeasurementWithCam);
}

/*
Vehicle::Vehicle()
{
	for (int i = 0; i < NUM_DOFS; ++i) dofs[i] = Dof();
	for (int i = 0; i < NUM_ANGLES; ++i) rpLimits[i] = PI / 4.0f;
	
	dji.roll    = 0;
	dji.pitch   = 0; 
	dji.yawRate = 0;
	dji.thrust  = 0;
	mass        = 1;
	time        = 0;
	lastPredTime = 0;
	
	// allocate EKF
	float initialState[6] = {0, 0, 0, 0, 0, 0};
	float initialErrorCov[36]; memset(initialErrorCov, 0, sizeof(float) * 36);
	ekf = new ExtendedKalmanFilter(6, initialState, initialErrorCov);

	float controlInput[4] = {0, 0, 0, 0};
	arm_mat_init_f32(&controlInputMat, 4, 1, controlInput);

	float sensorMeasurement[3] = {0, 0, 0};
	arm_mat_init_f32(&sensorMeasurementMat, 3, 1, sensorMeasurement);

	float sensorMeasurementWithCam[5] = {0, 0, 0, 0, 0};
	arm_mat_init_f32(&sensorMeasurementMatWithCam, 5, 1, sensorMeasurementWithCam);
}
*/

Vehicle::~Vehicle()
{
	delete ekf;
}

Vehicle::Vehicle(const float states[NUM_DOFS][NUM_DOF_STATES],
				 const float setpts[NUM_DOFS][NUM_DOF_STATES],
				 const float valueGains[NUM_DOFS][NUM_PID_GAINS],
				 const float rateGains[NUM_DOFS][NUM_PID_GAINS],
				 const uint8_t valueFlags[NUM_DOFS],
				 const uint8_t rateFlags[NUM_DOFS],
				 const float inertias[NUM_DOFS],
				 const float stateBounds[NUM_DOFS],
				 const float rateUpLims[NUM_DOFS],
				 const float rateLwLims[NUM_DOFS],
				 const float accelUpLims[NUM_DOFS],
				 const float accelLwLims[NUM_DOFS],
				 const float valueStateLpCoeffs[NUM_DOFS],
				 const float valueErrorLpCoeffs[NUM_DOFS],
				 const float rateStateLpCoeffs[NUM_DOFS],
				 const float rateErrorLpCoeffs[NUM_DOFS],
				 const float totalMass,
				 const float initTime,
				 const float rpLims[NUM_ANGLES],
				 const float ekfInitState[6],
				 const float ekfInitP[36],
				 float ekfQ[36],
				 float ekfNoCamR[3],
				 float ekfWithCamR[5])
{
	mass = totalMass;
	time = initTime;
	lastPredTime = initTime;
	lastCorrTime = initTime;
	dji.roll    = 0;
	dji.pitch   = 0; 
	dji.yawRate = 0;
	dji.thrust  = 0;
	
	for (int i = 0; i < NUM_ANGLES; ++i) rpLimits[i] = rpLims[i];

	for (int i = 0; i < NUM_DOFS; ++i)
	{
		dofs[i] = Dof(states[i], setpts[i], valueGains[i], rateGains[i],
					  valueFlags[i], rateFlags[i], inertias[i], stateBounds[i],
					  rateUpLims[i], rateLwLims[i], accelUpLims[i], 
					  accelLwLims[i], valueStateLpCoeffs[i], 
					  valueErrorLpCoeffs[i], rateStateLpCoeffs[i],
					  rateErrorLpCoeffs[i]);
	}

	// intial P = ekfInitErrorCov
	ekf = new ExtendedKalmanFilter(6, ekfInitState, ekfInitP);

	arm_matrix_instance_f32 Q;
	arm_mat_init_f32(&Q, 6, 6, ekfQ);
	ekf->setPredictFunc(4, systemDeltaState, systemGetJacobian, &Q);

	arm_matrix_instance_f32 RNoCam, RWithCam;
	arm_mat_init_f32(&RNoCam, 3, 3, ekfNoCamR);
	arm_mat_init_f32(&RWithCam, 5, 5, ekfWithCamR);
	ekf->setUpdateFunc(0, 3, sensorPredict, sensorGetJacobian, &RNoCam);
	ekf->setUpdateFunc(1, 5, sensorPredictWithCam, sensorGetJacobianWithCam, &RWithCam);


	float controlInput[4] = {0, 0, 0, 0};
	arm_mat_init_f32(&controlInputMat, 4, 1, controlInput);

	float sensorMeasurement[3] = {0, 0, 0};
	arm_mat_init_f32(&sensorMeasurementMat, 3, 1, sensorMeasurement);

	float sensorMeasurementWithCam[5] = {0, 0, 0, 0, 0};
	arm_mat_init_f32(&sensorMeasurementMatWithCam, 5, 1, sensorMeasurementWithCam);
}

// updates sensor inputs, runs the EKF, and updates the states in the DOFs
void Vehicle::runFilter(const float x, const float y, const float z,
				  	    const float xdot, const float ydot, const float roll,
						const float pitch, const float yawImu, const float yawCam,
						const float timestamp, const bool withCam, const FlightMode mode,
						const bool usePredict)
{
	// record time and calc filter dt
	lastPredTime = time;
	time = timestamp;
	float dt = time - lastPredTime;


	// get sin/cos of Euler angles
	preYawCos = arm_cos_f32(yawImu);
	preYawSin = arm_sin_f32(yawImu);
	float rollCos = arm_cos_f32(roll);
	float rollSin = arm_sin_f32(roll);
	float pitchCos = arm_cos_f32(pitch);
	float pitchSin = arm_sin_f32(pitch);

	// assign input to control vector
	//controlInputMat.pData[0] = (mode == MANUAL) ? map(servoIn_getPulse(RC_CHAN3), 114000, 124000, 0, 46.6956) : dji.thrust;
    controlInputMat.pData[0] = (mode == MANUAL) ? 0.0 : dji.thrust;
    controlInputMat.pData[1] = (mode == MANUAL) ? 0.0 : dji.roll;
	controlInputMat.pData[2] = (mode == MANUAL) ? 0.0 : dji.pitch;
	controlInputMat.pData[3] = (mode == MANUAL) ? 0.0 : dji.yawRate;

	// run filter
	//if ((time - lastCorrTime) < 0.02) // predict if corrDt < 20 ms (rate of Lidar)
	//if (dt < 0.015)
	if (usePredict)
	{
		ekf->predict(dt, mass, rollSin, rollCos, pitchSin, pitchCos,
					 preYawSin, preYawCos, &controlInputMat);
	}
	else // run correction bcz we have Lidar
	{
		lastCorrTime = time;
		if ((mode == AUTONOMOUS) && withCam)
		{
			sensorMeasurementMatWithCam.pData[0] = x;
			sensorMeasurementMatWithCam.pData[1] = y;
			sensorMeasurementMatWithCam.pData[2] = z * pitchCos * rollCos;
			sensorMeasurementMatWithCam.pData[3] = (xdot * preYawCos) + (ydot * preYawSin);
			sensorMeasurementMatWithCam.pData[4] = -(xdot * preYawSin) + (ydot * preYawCos);

			ekf->update(dt, 1, &sensorMeasurementMatWithCam); // update with camera vals
		}
		else
		{
			// assign input to sensor vector
			sensorMeasurementMat.pData[0] = z * pitchCos * rollCos;
			sensorMeasurementMat.pData[1] = (xdot * preYawCos) + (ydot * preYawSin);
			sensorMeasurementMat.pData[2] = -(xdot * preYawSin) + (ydot * preYawCos);

			ekf->update(dt, 0, &sensorMeasurementMat); // update without camera vals
		}
	}
	// extract state and send to dofs
	arm_matrix_instance_f32 ekfState = ekf->getState();
	float state[NUM_DOFS][NUM_DOF_STATES];
	for (int i = 0; i < NUM_DOFS; ++i) state[i][DOF_TIME] = time;
	state[X_AXIS][DOF_VAL]   = ekfState.pData[0];
	state[X_AXIS][DOF_RATE]  = ekfState.pData[3];
	state[X_AXIS][DOF_ACCEL] = 0;
	state[Y_AXIS][DOF_VAL]   = ekfState.pData[1];
	state[Y_AXIS][DOF_RATE]  = ekfState.pData[4];
	state[Y_AXIS][DOF_ACCEL] = 0;
	state[Z_AXIS][DOF_VAL]   = -ekfState.pData[2];
	state[Z_AXIS][DOF_RATE]  = -ekfState.pData[2] / dt;
	state[Z_AXIS][DOF_ACCEL] = 0;
	state[YAW][DOF_VAL]      = yawImu;
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

	forceVe[Z_AXIS] = (dofs[Z_AXIS].getRate() * mass) + (mass * GRAVITY);

	// Convert earth frame forces to body frame
	forceVy[X_AXIS] =  (preYawCos * forceVe[X_AXIS]) + (preYawSin * forceVe[Y_AXIS]);
	forceVy[Y_AXIS] = -(preYawSin * forceVe[X_AXIS]) + (preYawCos * forceVe[Y_AXIS]);
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
	arm_matrix_instance_f32 ekfState = ekf->getState();
	
	// fill vehicle log
	vlog.xUval	   = dofs[X_AXIS].getUval();
	vlog.yUval     = dofs[Y_AXIS].getUval();
	vlog.zUval     = dofs[Z_AXIS].getRate() * mass;
	vlog.xFilt     = ekfState.pData[0];
	vlog.yFilt     = ekfState.pData[1];
	vlog.zFilt     = ekfState.pData[2];
	vlog.xdotFilt  = ekfState.pData[3];
	vlog.ydotFilt  = ekfState.pData[4];
	vlog.zdotFilt  = ekfState.pData[5];
	vlog.rollFilt  = 0;
	vlog.pitchFilt = 0;
	vlog.yawFilt   = 0;
}

// End of File
