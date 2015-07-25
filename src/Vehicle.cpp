#include <stdint.h>
#include <cstdlib>
#include <cmath>
#include "Vehicle.hpp"
#include "Dof.hpp"
#include "Pid.hpp"
#include "FlightMode.hpp"

using namespace std;

// define pi and gravity
static const float PI = 3.14159265358979323846;
static const float GRAVITY = 9.81;

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
	float initialState[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	float initialErrorCov[81]; memset(initialErrorCov, 0, sizeof(float) * 81);
	ekf = new ExtendedKalmanFilter(9, initialState, initialErrorCov);
	
	float controlInput[4] = {0, 0, 0, 0};
	arm_mat_init_f32(&controlInputMat, 4, 1, controlInput);

	float sensorMeasurement[6] = {0, 0, 0, 0, 0, 0};
	arm_mat_init_f32(&sensorMeasurementMat, 6, 1, sensorMeasurement);
}

Vehicle::~Vehicle()
{
	delete ekf;
}

Vehicle::Vehicle(const float states[NUM_DOFS][NUM_DOF_STATES],
				 const float setpts[NUM_DOFS][NUM_DOF_STATES],
				 const float valueGains[NUM_DOFS][NUM_PID_GAINS],
				 const float rateGains[NUM_DOFS][NUM_PID_GAINS],
				 const float valueFlags[NUM_DOFS],
				 const float rateFlags[NUM_DOFS],
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
				 const float rpLims[NUM_ANGLES])
{
	mass = totalMass;
	time = initTime;
	lastPredTime = 0;
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
	
	// allocate EKF
	float initialState[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	float initialErrorCov[81]; memset(initialErrorCov, 0, sizeof(float) * 81);
	ekf = new ExtendedKalmanFilter(9, initialState, initialErrorCov);
	
	float predictCov[81] = {
			1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1,
	};
	arm_matrix_instance_f32 predictCovMat;
	arm_mat_init_f32(&predictCovMat, 9, 9, predictCov);
	ekf->setPredictFunc(4, systemDeltaState, systemGetJacobian, &predictCovMat);

	float updateCov[36] = {
			1, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1,
	};
	arm_matrix_instance_f32 updateCovMat;
	arm_mat_init_f32(&updateCovMat, 6, 6, updateCov);
	ekf->setUpdateFunc(0, 6, sensorPredict, sensorGetJacobian, &updateCovMat);

	float controlInput[4] = {0, 0, 0, 0};
	arm_mat_init_f32(&controlInputMat, 4, 1, controlInput);

	float sensorMeasurement[6] = {0, 0, 0, 0, 0, 0};
	arm_mat_init_f32(&sensorMeasurementMat, 6, 1, sensorMeasurement);
}

// updates sensor inputs, runs the EKF, and updates the states in the DOFs
void Vehicle::runFilter(const float x, const float y, const float z,
				   const float xdot, const float ydot, const float roll, 
				   const float pitch, const float yaw, const float timestamp);
{
	// record time and calc filter dt
	lastPredTime = time;
	time = timestamp;
	float dt = time - lastPredTime;
	
	// assign input to sensor vector
	sensorMeasurementMat.pData[0] = roll;
	sensorMeasurementMat.pData[1] = pitch;
	sensorMeasurementMat.pData[2] = yaw;
	sensorMeasurementMat.pData[3] = xdot;
	sensorMeasurementMat.pData[4] = ydot;
	sensorMeasurementMat.pData[5] = z;
	
	// assign input to control vector
	controlInputMat.pData[0] = dji.thrust;
	controlInputMat.pData[1] = dji.roll;
	controlInputMat.pData[2] = dji.pitch;
	controlInputMat.pData[3] = dji.yawRate;

	// run filter
	filter.predict(dt, mass, &controlInputMat);
	filter.update(dt, 0, &sensorMeasurementMat);
	
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
	state[Z_AXIS][DOF_VAL]   = ekfState.pData[2];
	state[Z_AXIS][DOF_RATE]  = ekfState.pData[5];
	state[Z_AXIS][DOF_ACCEL] = 0;
	state[YAW][DOF_VAL]      = ekfState.pData[8];
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
		case MANUAL: break;
		default: 	 break;
	}
	
	// always do full cascade for z and yaw
	dofs[Z_AXIS].run(false);
	dofs[YAW].run(false);

	// finally get dji values
	calcDJIValues();
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
		case MANUAL: break;
		default: 	 break;
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


void Vehicle::calcDJIValues()
{
	float forceVe[3];        // vehicle force in the earth frame (in R^3)
	float forceVy[3];        // vehicle force in the yaw frame (in R^3)
	float forceMag = 0;      // magnitude of the vehicle force
	float angle[NUM_ANGLES]; // roll and pitch setpoint for dji

	// get earth frame vehicle forces
	for (int i = 0; i < 3; ++i) forceVe[i] = dofs[i].getUval();
	forceVe[Z_AXIS] += mass * GRAVITY;

	float preYawCos = 0, preYawSin = 0;
	// Convert earth frame forces to body frame, adding trim (which is already
	// in the body frame)
	forceVy[X_AXIS] =  (preYawCos * forceVe[X_AXIS]) + (preYawSin * forceVe[Y_AXIS]);
	forceVy[Y_AXIS] = -(preYawSin * forceVe[X_AXIS]) + (preYawCos * forceVe[Y_AXIS]);
	forceVy[Z_AXIS] = forceVe[Z_AXIS];

	// calculate ||F|| = sqrt(Fx^2 + Fy^2 + Fz^2) (L2-Norm)
	for (int i = 0; i < 3; ++i) forceMag += forceVy[i] * forceVy[i];
	forceMag = sqrt(forceMag);

	// Calculate roll and pitch
	angle[ROLL]  = -asin(forceVy[Y_AXIS] / forceMag);
	angle[PITCH] =  asin(forceVy[X_AXIS] / 
			sqrt((forceMag * forceMag) - (forceVy[Y_AXIS] * forceVy[Y_AXIS])));

	// cap roll and pitch
	if (angle[ROLL]  >  rpLimits[ROLL])  angle[ROLL]  =  rpLimits[ROLL];
	if (angle[ROLL]  < -rpLimits[ROLL])  angle[ROLL]  = -rpLimits[ROLL];
	if (angle[PITCH] >  rpLimits[PITCH]) angle[PITCH] =  rpLimits[PITCH];
	if (angle[PITCH] < -rpLimits[PITCH]) angle[PITCH] = -rpLimits[PITCH];

	// assign DJI values
	dji.roll    = angle[ROLL];
	dji.pitch   = angle[PITCH];
	dji.thrust  = dofs[Z_AXIS].getUval();
	dji.yawRate = dofs[YAW].getRate();
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
	vlog.zUval     = dofs[Z_AXIS].getUval();
	vlog.xFilt     = ekfState.pData[0];
	vlog.yFilt     = ekfState.pData[1];
	vlog.zFilt     = ekfState.pData[2];
	vlog.xdotFilt  = ekfState.pData[3];
	vlog.ydotFilt  = ekfState.pData[4];
	vlog.zdotFilt  = ekfState.pData[5];
	vlog.rollFilt  = ekfState.pData[6];
	vlog.pitchFilt = ekfState.pData[7];
	vlog.yawFilt   = ekfState.pData[8];
}

// End of File
