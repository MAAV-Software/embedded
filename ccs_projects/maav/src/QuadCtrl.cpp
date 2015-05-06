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
	// 0 for all lowpass coeffs turns off lowpass filter in DOF
	float lowpass_coeff[4] = {0.0, 0.0, 0.0};

	// value and rate PID gains
	float value_gains[4][3] =
	{
		/* Kp,  Ki,    Kd 		value gains */
		{1.00, 0.00, 0.00,},	/* x */
		{1.00, 0.00, 0.00,},	/* y */
		{1.00, 0.00, 0.00,},	/* z */
		{1.00, 0.00, 0.00,},	/* yaw */
	};

	float rate_gains[4][3] =
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
	_rpLimits[ROLL] = 10.0;
	_rpLimits[PITCH] = 10.0;
	_preYawSin = 0.0;
	_preYawCos = 1.0;

	djiRoll = 0.0;
	djiPitch = 0.0;
	djiYawDot = 0.0;
	djiForceZ = 0.0;

	for (uint8_t i = 0; i < 4; ++i) // loop through and initialize dofs
	{
		//TODO: FIGURE THIS OUT
		_xyzh[i].Dof(); // does this work here ?????????
		// call dof init funcitons
		/*
		dof_init_struct(&(qc->xyzh[i]), 0, 0, 0, 0, qc->mass);
		dof_set_gains(&(qc->xyzh[i]), value_gains[i], rate_gains[i]);
		dof_init_limits(&(qc->xyzh[i]), state_bounds[i], vel_caps[i],
						Uval_posThresh[i], Uval_negThresh[i]);
		dof_init_flags(&(qc->xyzh[i]), wraparounds[i], take_disc_deriv[i]);
		dof_init_lowpass(&(qc->xyzh[i]), lowpass_coeff);
		*/
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
void qc_setState(quad_ctrl_t *qc, float state[], float t) {
	uint8_t i;
	for (i = 0; i < 4; ++i)
		dof_set_state(&(qc->xyzh[i]), state[i], state[i + 4], 0, t);

	// calculate sin(yaw) and cos(yaw) for later (yaw is state[3])
//	qc->pre_yaw_sin = sinf(state[3]);
//	qc->pre_yaw_cos = cosf(state[3]);	//TODO add these back (ask Sajan why)
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
void qc_setSetpt(quad_ctrl_t *qc, float setpt[], float t)
{
	uint8_t i;
	for (i = 0; i < 4; ++i)
		dof_set_setpt(&(qc->xyzh[i]), setpt[i], setpt[i + 4], 0, t);
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
void qc_runPID(quad_ctrl_t *qc)
{
	uint8_t i;

	for (i = 0; i < 4; ++i) dof_calc_ctrl_dt(&(qc->xyzh[i]));
	// get change in time for all DOFs

	qc_runValuePID(qc); // run position->velocity PID (accounts for ctrl mode)
	qc_runRatePID(qc); // run velocity->force PID
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
void qc_runValuePID(quad_ctrl_t *qc)
{
	uint8_t i;

	if (qc->ctrl_mode == AUTON_CTRL) // only do X, Y value PID if in auton mode
	{
		for (i = 0; i < 2; ++i) // loop through x, y DOFs
		{
			dof_calc_error(&(qc->xyzh[i]), 0, 1); // calculate errors
			dof_value_PID(&(qc->xyzh[i]), 0);
		}
	}

	// always do Z and Yaw value PID
//	for (i = 2; i < 4; ++i) { // loop through Z and Yaw DOFs //TODO Add yaw back.  Removed (no yaw value PID needed for comp)
		dof_calc_error(&(qc->xyzh[Z_AXIS]), 0, 1); // calculate errors
		dof_value_PID(&(qc->xyzh[Z_AXIS]), 0);
//	}
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
void qc_runRatePID(quad_ctrl_t *qc)
{
	uint8_t i;

	for (i = 0; i < 3; ++i) // only run through X, Y, Z rate PIDs
	{
		dof_calc_error(&(qc->xyzh[i]), 1, 1); // calculate rate and accel error
		dof_calc_error(&(qc->xyzh[i]), 2, 1); // (accel probably not needed)
		dof_rate_PID(&(qc->xyzh[i]), 1); // run rate PID with d(rate_error)/dt
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
void qc_setDJIValues(quad_ctrl_t *qc)
{
	float force_ve[3];      // vehicle force in the earth frame
	float force_vy[3];      // vehicle force in the yaw frame
	float force_mag = 0;    // magnitude of the vehicle force
	float angle[2];       // roll and pitch setpoint
	uint8_t i;

	// get earth frame vehicle forces
	for (i = 0; i < 3; ++i) force_ve[i] = qc->xyzh[i].Uval;

	force_ve[Z_AXIS] += qc->mass * GRAVITY;

	// Convert earth frame forces to body frame, adding trim (which is already
	// in the body frame)
	force_vy[X_AXIS] = (qc->pre_yaw_cos * force_ve[X_AXIS])
					   + (qc->pre_yaw_sin * force_ve[Y_AXIS]);
	force_vy[Y_AXIS] = -(qc->pre_yaw_sin * force_ve[X_AXIS])
					   + (qc->pre_yaw_cos * force_ve[Y_AXIS]);
	force_vy[Z_AXIS] = force_ve[Z_AXIS];

	// calculate ||F|| = sqrt(Fx^2 + Fy^2 + Fz^2)
	for (i = 0; i < 3; ++i) force_mag += force_vy[i] * force_vy[i];

	force_mag = sqrtf(force_mag);

	// Calculate roll and pitch
	angle[ROLL]  = -asinf(force_vy[Y_AXIS] / force_mag);
	angle[PITCH] = asinf(force_vy[X_AXIS] / sqrtf((force_mag * force_mag)
	            		 - (force_vy[Y_AXIS] * force_vy[Y_AXIS])));

	// cap roll and pitch
	if (angle[ROLL] > qc->rp_cap[ROLL]) angle[ROLL] = qc->rp_cap[ROLL];
	if (angle[ROLL] < -qc->rp_cap[ROLL]) angle[ROLL] = -qc->rp_cap[ROLL];
	if (angle[PITCH] > qc->rp_cap[PITCH]) angle[PITCH] = qc->rp_cap[PITCH];
	if (angle[PITCH] < -qc->rp_cap[PITCH]) angle[PITCH] = -qc->rp_cap[PITCH];

	// assign DJI values
	qc->dji_roll = angle[ROLL];
	qc->dji_pitch = angle[PITCH];
	qc->dji_forceZ = qc->xyzh[Z_AXIS].Uval;

	if (qc->ctrl_mode == AUTON_CTRL) qc->dji_yawDot = qc->xyzh[YAW].velocity;
	else qc->dji_yawDot = qc->xyzh[YAW].setpt[1];
}

// End of File
