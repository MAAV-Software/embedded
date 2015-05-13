#include <stdint.h>
#include <stdlib.h>
#include <cmath>
#include "Vehicle.hpp"
#include "Dof.hpp"

// define PI just in case
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GRAVITY 9.81

Vehicle::Vehicle()
{
	// value and rate PID gains
	float valueGains[4][3] =
	{
		/* Kp,  Ki,    Kd 		value gains */
		{1.00, 0.00, 0.00,},	/* x */
		{1.00, 0.00, 0.00,},	/* y */
		{1.00, 0.00, 0.00,},	/* z */
		{1.00, 0.00, 0.00,},	/* yaw */
	};
	float rateGains[4][3] =
	{
		/* Kp,  Ki,    Kd		rate gains */
		{1.00, 0.00, 0.00,},	/* x */
		{1.00, 0.00, 0.00,},	/* y */
		{1.00, 0.00, 0.00,},	/* z */
		{1.00, 0.00, 0.00,},	/* yaw */
	};

	// flags for taking discrete derivatives of state variables
	uint8_t flags[4] =
	{
		DISC_ACCEL,	/* x */
		DISC_ACCEL,	/* y */
		DISC_ACCEL,	/* z */
		DISC_ACCEL | WRAP_AROUND,	/* yaw */
	};

	// index order used by the following: [X, Y, Z, Yaw]
	float stateBounds[4]	= {0, 0, 0, M_PI};
	float velCaps[4]		= {1.0, 1.0, 1.0, 1.0};
	float UvalPosThresh[4]	= {100.0, 100.0, 150.0, 1.0};
	float UvalNegThresh[4]	= {-100.0, -100.0, -150.0, -1.0};

	// Vehicle Members
	mode            = MANUAL;
	djiRoll         = 0.0;
	djiPitch        = 0.0;
	djiYawDot       = 0.0;
	djiForceZ       = 0.0;
	mass            = 1.0;
	rpLimits[ROLL]  = 1.0;
	rpLimits[PITCH] = 1.0;
	preYawSin       = 0.0;
	preYawCos       = 0.0;

	for (uint8_t i = 0; i < 4; ++i) // loop through and initialize dofs
	{
		xyzh[i] = Dof(0, 0, 0, 0, mass, stateBounds[i], velCaps[i],
					  UvalPosThresh[i], UvalNegThresh[i], flags[i]);
		xyzh[i].setGains(valueGains[i], rateGains[i]);
	}
}


Vehicle::Vehicle(float valueGains[4][3], float rateGains[4][3],
		  	  	 float stateBounds[4], float velCaps[4], float rpCaps[2],
				 float UvalPosThresh[4], float UvalNegThresh[4],
				 uint8_t flags[4], FlightMode modeInit, float massInit)
{
	mode 			= modeInit;
	mass 			= massInit;
	rpLimits[ROLL]  = rpCaps[ROLL];
	rpLimits[PITCH] = rpCaps[PITCH];
	preYawSin 		= 0.0;
	preYawCos 		= 1.0;
	djiRoll   		= 0.0;
	djiPitch  		= 0.0;
	djiYawDot 		= 0.0;
	djiForceZ 		= 0.0;

	for (uint8_t i = 0; i < 4; ++i) // loop through and initialize dofs
	{
		xyzh[i] = Dof(0, 0, 0, 0, mass, stateBounds[i], velCaps[i],
		     		  UvalPosThresh[i], UvalNegThresh[i], flags[i]);
		xyzh[i].setGains(valueGains[i], rateGains[i]);
	}
}

Vehicle::~Vehicle() {}

void Vehicle::setSetpt(float setpt[], float t)
{
	for (uint8_t i = 0; i < 4; ++i) xyzh[i].setSetpt(setpt[i], setpt[i+4], 0, t);
}


void Vehicle::setGains(float valueGains[4][3], float rateGains[4][3])
{
	for (uint8_t i = 0; i < 4; ++i)
	{
		xyzh[i].setGains(valueGains[i], rateGains[i]);
	}
}

void Vehicle::runPID()
{
	for (uint8_t i = 0; i < 4; ++i) xyzh[i].calcCtrlDt();
	// get change in time for all DOFs

	runValuePID(); // run position->velocity PID (accounts for ctrl mode)
	runRatePID(); // run velocity->force PID
}

void Vehicle::runValuePID()
{
	if (mode == AUTONOMOUS) // only do X, Y value PID if in auton mode
	{
		for (uint8_t i = 0; i < 2; ++i) // loop through x, y DOFs
		{
			xyzh[i].calcError(VAL, true);
			xyzh[i].valuePID(false);
		}
	}

	// always do Z and Yaw value PID
	for (uint8_t i = 2; i < 4; ++i)
	{
		xyzh[i].calcError(VAL, true);
		xyzh[i].valuePID(false);
	}
}

void Vehicle::runRatePID()
{
	for (uint8_t i = 0; i < 3; ++i) // only run through X, Y, Z rate PIDs
	{
		xyzh[i].calcError(RATE, true); // calculate rate and accel error
		xyzh[i].calcError(ACCEL, true);// (accel probably not needed)
		xyzh[i].ratePID(true); // run rate PID with d(rate_error)/dt
	}
}

void Vehicle::calcDJIValues()
{
	float forceVe[3];      // vehicle force in the earth frame
	float forceVy[3];      // vehicle force in the yaw frame
	float forceMag = 0;    // magnitude of the vehicle force
	float angle[2];       // roll and pitch setpoint

	// get earth frame vehicle forces
	for (uint8_t i = 0; i < 3; ++i) forceVe[i] = xyzh[i].Uval;
	forceVe[Z_AXIS] += mass * GRAVITY;

	// Convert earth frame forces to body frame, adding trim (which is already
	// in the body frame)
	forceVy[X_AXIS] =  (preYawCos * forceVe[X_AXIS]) + (preYawSin * forceVe[Y_AXIS]);
	forceVy[Y_AXIS] = -(preYawSin * forceVe[X_AXIS]) + (preYawCos * forceVe[Y_AXIS]);
	forceVy[Z_AXIS] = forceVe[Z_AXIS];

	// calculate ||F|| = sqrt(Fx^2 + Fy^2 + Fz^2)
	for (uint8_t i = 0; i < 3; ++i) forceMag += forceVy[i] * forceVy[i];
	forceMag = sqrtf(forceMag);

	// Calculate roll and pitch
	angle[ROLL]  = -asinf(forceVy[Y_AXIS] / forceMag);
	angle[PITCH] =  asinf(forceVy[X_AXIS] / sqrtf((forceMag * forceMag)
	            		   - (forceVy[Y_AXIS] * forceVy[Y_AXIS])));

	// cap roll and pitch
	if (angle[ROLL]  >  rpLimits[ROLL]) 	angle[ROLL]  =  rpLimits[ROLL];
	if (angle[ROLL]  < -rpLimits[ROLL]) 	angle[ROLL]  = -rpLimits[ROLL];
	if (angle[PITCH] >  rpLimits[PITCH]) 	angle[PITCH] =  rpLimits[PITCH];
	if (angle[PITCH] < -rpLimits[PITCH]) 	angle[PITCH] = -rpLimits[PITCH];

	// assign DJI values
	djiRoll   = angle[ROLL];
	djiPitch  = angle[PITCH];
	djiForceZ = xyzh[Z_AXIS].Uval;

	if (mode == AUTON_CTRL)	djiYawDot = xyzh[YAW].velocity;
	else 					djiYawDot = xyzh[YAW].setpt[RATE];
}

// End of File
