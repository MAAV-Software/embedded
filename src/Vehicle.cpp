#include <stdint.h>
#include <cstdlib>
#include <math.h>
#include "Vehicle.hpp"
#include "Dof.hpp"
#include "Pid.hpp"
#include "FlightMode.hpp"

// define pi and gravity
const float M_PI = 3.14159265358979323846;
const float GRAVITY = 9.81;

Vehicle::Vehicle()
{
	for (int i = 0; i < NUM_DOFS; ++i) dofs[i] = Dof();
	for (int i = 0; i < NUM_ANGLES; ++i) rpLimits[i] = M_PI / 4;
	for (int i = 0; i < ROTMAT_SIZE; ++i) rotMat[i] = 0;
	rotMat[0] = rotMat[4] = rotMat[8] = 1;
	
	dji.roll    = 0;
	dji.pitch   = 0; 
	dji.yawRate = 0;
	dji.thrust  = 0;
	mass        = 1;
	time        = 0;
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
				 const float rpLims[NUM_ANGLES],
				 const float initRotMat[ROTMAT_SIZE])
{
	mass = totalMass;
	time = initTime;
	dji.roll    = 0;
	dji.pitch   = 0; 
	dji.yawRate = 0;
	dji.thrust  = 0;
	
	for (int i = 0; i < ROTMAT_SIZE; ++i) rotMat[i] = initRotMat[i];
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

void Vehicle::prepareLog()
{	
	//TODO Implement this function
}

// End of File
