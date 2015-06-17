#include <stdint.h>
#include <cstdlib>
#include <math.h>
#include "Vehicle.hpp"
#include "Dof.hpp"
#include "Pid.hpp"
#include "FlightMode.hpp"

// define PI just in case
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Define Gravity
#ifndef GRAVITY
#define GRAVITY 9.81
#endif

Vehicle::Vehicle()
{
	for (int i = 0; i < NUM_DOFS; ++i) dofs[i] = Dof();
	for (int i = 0; i < NUM_ANGLES; ++i) rpLimits[i] = M_PI / 4;
	for (int i = 0; i < ROTMAT_SIZE; ++i) rotMat[i] = 0;
	rotMat[0] = rotMat[4] = rotMat[8] = 1;
	djiRoll   = 0;
	djiPitch  = 0; 
	djiYawDot = 0;
	djiThrust = 0;
	mass      = 1;
	time      = 0;
}
/*
Vehicle::Vehicle(


		)
{


	Dof(const float state[NUM_DOF_STATES], 
		const float setpt[NUM_DOF_STATES],
	 	const float valueGains[NUM_PID_GAINS],
		const float rateGains[NUM_PID_GAINS],
		const uint8_t valueFlags,
		const uint8_t rateFlags,
		const float Inertia,
		const float stateBound,
		const float rateUpLim,
		const float rateLwLim, 
		const float accelUpLim,
		const float accelLwLim,
		const float valueLpCoeff[NUM_PID_STATES - 1],
		const float rateLpCoeff[NUM_PID_STATES - 1]);
	


}
*/
/*
void Vehicle::calcDJIValues()
{
	float forceVe[3];      // vehicle force in the earth frame (in R^3)
	float forceVy[3];      // vehicle force in the yaw frame (in R^3)
	float forceMag = 0;    // magnitude of the vehicle force
	float angle[NUM_ANGLES]; // roll and pitch setpoint for dji

	// get earth frame vehicle forces
	for (int i = 0; i < 3; ++i) forceVe[i] = xyzh[i].getUval();
	forceVe[Z_AXIS] += mass * GRAVITY;

	// Convert earth frame forces to body frame, adding trim (which is already
	// in the body frame)
	forceVy[X_AXIS] =  (preYawCos * forceVe[X_AXIS]) + (preYawSin * forceVe[Y_AXIS]);
	forceVy[Y_AXIS] = -(preYawSin * forceVe[X_AXIS]) + (preYawCos * forceVe[Y_AXIS]);
	forceVy[Z_AXIS] = forceVe[Z_AXIS];

	// calculate ||F|| = sqrt(Fx^2 + Fy^2 + Fz^2)
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
	djiRoll   = angle[ROLL];
	djiPitch  = angle[PITCH];
	djiThrust = xyzh[Z_AXIS].getUval();
	djiYawDot = xyzh[YAW].getRate();
}
*/

float Vehicle::getDjiRoll() const
{
	return djiRoll;
}

float Vehicle::getDjiPitch() const
{
	return djiPitch;
}

float Vehicle::getDjiYawDot() const
{
	return djiYawDot;
}

float Vehicle::getDjiThrust() const
{
	return djiThrust;
}

void Vehicle::prepareLog()
{	
	//TODO Implement thif function
}

// End of File
