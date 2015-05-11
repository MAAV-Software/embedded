/*
 * quad_ctrl.c
 *
 * Executing PID outer-loop control of quadrotor with four
 * degrees of freedom to control (X, Y, Z, Yaw). This implements the functions
 * declared in quad_ctrl.h
 *
 *  Created on: Jul 17, 2014
 *      Author: Sajan Patel, Matt Karashin
 */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "Dof.hpp"
#include "QuadCtrl.hpp"

// define PI just in case
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define GRAVITY 9.81

QuadCtrl::QuadCtrl()
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

 	// Hardcode intertial properties, mode, and limits
	mode = RC_CTRL;
	mass = 1.0;
	rpLimits[ROLL]  = 10.0;
	rpLimits[PITCH] = 10.0;
	preYawSin = 0.0;
	preYawCos = 1.0;
	djiRoll   = 0.0;
	djiPitch  = 0.0;
	djiYawDot = 0.0;
	djiForceZ = 0.0;

	for (uint8_t i = 0; i < 4; ++i) // loop through and initialize dofs
	{
		xyzh[i] = Dof(0, 0, 0, 0, mass, stateBounds[i], velCaps[i],
					  UvalPosThresh[i], UvalNegThresh[i], flags[i]);
		xyzh[i].setGains(valueGains[i], rateGains[i]);
	}
}

QuadCtrl::QuadCtrl(float valueGains[4][3], float rateGains[4][3],
		  	  	  	 float stateBounds[4], float velCaps[4], float rpCaps[2],
					 float UvalPosThresh[4], float UvalNegThresh[4],
					 uint8_t flags[4], ctrlMode modeInit, float massInit)
{
	mode = modeInit;
	mass = massInit;
	rpLimits[ROLL]  = rpCaps[ROLL];
	rpLimits[PITCH] = rpCaps[PITCH];
	preYawSin = 0.0;
	preYawCos = 1.0;
	djiRoll   = 0.0;
	djiPitch  = 0.0;
	djiYawDot = 0.0;
	djiForceZ = 0.0;

	for (uint8_t i = 0; i < 4; ++i) // loop through and initialize dofs
	{
		xyzh[i] = Dof(0, 0, 0, 0, mass, stateBounds[i], velCaps[i],
		     		  UvalPosThresh[i], UvalNegThresh[i], flags[i]);
		xyzh[i].setGains(valueGains[i], rateGains[i]);
	}
}

/*
 * qc_setState
 *
 * Sets the state of the quadrotor based on feedback from the Kalman filter's
 * state estimator. This also calculates the sine and cosine of the feedback
 * Yaw for future use in force balancing to calculate DJI values.
 *
 * \param qc	pointer to quad_ctrl_t struct
 * \param state	state feedback array. Index order:
 * 					[x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
 * \param t		timestamp
 *
 * \pre		qc_init must be called and state must be formatted as indicated
 * 			above. If data is not available for any of the state array elements,
 * 			enter a 0 in their place.
 * \post	PID control can be executed
 */
void QuadCtrl::setQuadState(float state[], float t)
{
	for (uint8_t i = 0; i < 4; ++i) xyzh[i].setState(state[i], state[i+4], 0, t);

	// calculate sin(yaw) and cos(yaw) for later
	preYawSin = sinf(state[YAW]);
	preYawCos = cosf(state[YAW]);
}

/*
 * qc_setSetpt
 *
 * Sets the controller setpoint in each DOF.
 *
 * \param qc	pointer to global quad_ctrl_t struct
 * \param setpt	array of setpoints for all of the DOFs. Index order:
 * 					[x, y, z, yaw, x_dot, y_dot, z_dot, yaw_dot]
 * \param t		timestamp
 *
 * \pre		Format the setpt array according to the order listed above. If any
 * 			of the above setpoints are not available, enter a 0 in its place in
 * 			the setpt array. Also, qc must be initialized.
 * 			*** If the control mode changes from the default which it is
 * 			initialized to (AUTON_CTRL), update qc->ctrl_mode the appropraite
 * 			mode. If switching between different modes, update qc->ctrl_mode
 * 			to switch.
 * 			*** If in RC_CTRL mode, enter 0s for x, y, z, and yaw entries in
 * 			the setpt array.
 * \post	Setpoints are ready to be acted upon in PID control.
 */
void QuadCtrl::setQuadSetpt(float setpt[], float t)
{
	for (uint8_t i = 0; i < 4; ++i) xyzh[i].setSetpt(setpt[i], setpt[i+4], 0, t);
}

/* qc_runPID
 *
 * Executes all PID control for all DOFs accoridng to the control mode.
 *
 * \param qc	pointer to quad_ctrl_t struct
 *
 * \pre		setpoints and state are valid and qc_init has been called
 * \post	qc_setDJIValues
 */
void QuadCtrl::runPID()
{
	for (uint8_t i = 0; i < 4; ++i) xyzh[i].calcCtrlDt();
	// get change in time for all DOFs

	runValuePID(); // run position->velocity PID (accounts for ctrl mode)
	runRatePID(); // run velocity->force PID
}


/* qc_runValuePID
 *
 * Executes positon->rate PIDs for all DOFs in qc. This calculates the rate
 * setpoints to prep for qc_runRatePID as well as sets the DOF velocity member
 * varaible with the velocity output (intended for Yaw).
 *
 * \param qc	pointer to global quad_ctrl_t struct
 *
 * \pre		qc_setState and qc_setSetpt have been called to set valid setpoints
 * 			in each of qc's DOF structs.
 * \post	Call qc_runRatePID or access the velocity member of each DOF within
 * 			qc.
 */
void QuadCtrl::runValuePID()
{
	if (mode == AUTON_CTRL) // only do X, Y value PID if in auton mode
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

/*
 * qc_runRatePID
 *
 * Executes rate->force PID control for all DOFs within qc. This calculates the
 * Uval for each dof.
 *
 * \param qc	pointer to global quad_ctrl_t struct
 *
 * \pre		DOF rate setpoints have been set either manually or by calling
 * 			qc_runValuePID
 * \post	Uvals are ready for each DOF
 */
void QuadCtrl::runRatePID()
{
	for (uint8_t i = 0; i < 3; ++i) // only run through X, Y, Z rate PIDs
	{
		xyzh[i].calcError(RATE, true); // calculate rate and accel error
		xyzh[i].calcError(ACCEL, true);// (accel probably not needed)
		xyzh[i].ratePID(true); // run rate PID with d(rate_error)/dt
	}
}

/*
 * qc_setDJIValues
 *
 * Calculates the roll, pitch, thrust force, and yaw rate commands to send to
 * the DJI Naza-M Lite (inner loop controller). This is done by balancing the
 * force outputs (Uval) of the x, y, and z DOFs to calculate roll and
 * pitch, passing through the force output of the z DOF (Uval) for z force
 * (aka thrust), and passing through the velocity output of the yaw DOF for
 * yaw rate.
 *
 * \param qc	pointer to global quad_ctrl_t struct
 *
 * \pre		qc_runValuePID for yaw rate and x, y, z rate setpts has been called
 * 			followed by qc_runRatePID for generating x, y, z Uvals
 * \post	dji_roll, dji_pitch, dji_forceZ, and dji_yawDot from the qc struct
 * 			are valid and can be used
 */
void QuadCtrl::calcDJIValues()
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
