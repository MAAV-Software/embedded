/** \file dof.c
    \brief Implimentation of the DOF structure support functions

    Implements the functionality and support of the DOF structure allowing the
    structure to be used for implementing a controller for the quadrotor.

    \author Mathew Karashin, Sajan Patel
    \version 3.0
    \date July 2014

    \see dof.h
*/

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "dof.h"

/** Sets up the internal state and other data in the struct so that first loop
    runs properly by using the initial state of the vehicle to seed the state
    vector.

    \param dof     	pointer to calling structure
    \param x       	initial state
    \param dx_dt   	initial rate
    \param d2x_dt2 	initial accel
    \param t       	initial time
    \param inertia	mass or momemnt of intertia for this dof

    \pre None, call immediately after declaring structure
    \post Dynamic values in structre zeroed for first loop

    \ingroup dof_t_methods
*/
void dof_init_struct(dof_t *dof, const float x, const float dx_dt,
					 const float d2x_dt2, const float t, const float inertia)
{
	uint8_t i;

	for (i = 0; i < 4; ++i) // init state vals to 0
	{
		dof->state[i] = 0.0;
		dof->setpt[i] = 0.0;
		dof->ctrl_prev_err[i] = 0.0;
		dof->ctrl_error[i] = 0.0;
	}

	for (i = 0; i < 3; ++i)	// init error deriv and error integral to 0
	{
		dof->ctrl_derr_dt[i] = 0.0;
		dof->ctrl_integral[i] = 0.0;
	}

	// assign user-defined intial state
	dof->state[0] = x;
	dof->state[1] = dx_dt;
	dof->state[2] = d2x_dt2;
	dof->state[3] = t;
	dof->ctrl_error[3] = dof->state[3];
	dof->inertia = inertia;

	dof->velocity = 0.0; // init veloctiy to 0
	dof->Uval = 0.0;	// init force to 0
}

/** Assigns the PID gains in the structure so that it can properly control
    the degree of freedom. Also initializes the inertia term for the dof which
    will be either a mass or a moment of inertia. Resets integrals to eliminate
    jumps caused by changing the integral gain.

    \param dof pointer to calling structure
    \param inertia mass or moment of inertia term for the degree of freedom
    \param value_gains [Kp, Ki, Kd] for the value -> desired rate PID
    \param rate_gains [Kp, Ki, Kd] for the rate -> U value PID

    \pre None, call after declaring structre
    \post Gains stored in structure for proper control

    \ingroup dof_t_methods
*/
void dof_set_gains(dof_t *dof, float value_gains[], float rate_gains[])
{
	uint8_t i;

	// Reset integral if integral gain changes
	if (value_gains[1] != dof->value_gains[1])
	{
		if (value_gains[1] != 0.0)
			dof->ctrl_integral[0] *= dof->value_gains[1] / value_gains[1];
		else
			dof->ctrl_integral[0] = 0.0;
	}

	// Do the same for rate integral
	if (rate_gains[1] != dof->rate_gains[1])
	{
		if (rate_gains[1] != 0.0)
			dof->ctrl_integral[1] *= dof->rate_gains[1] / rate_gains[1];
		else
			dof->ctrl_integral[1] = 0.0;
	}

	for (i = 0; i < 3; ++i) // assign new gains
	{
		dof->value_gains[i] = value_gains[i];
		dof->rate_gains[i] = rate_gains[i];
	}
}

/** Defintes the limits on state, desired velocity, and force for the structure
    to ensure the PID output does not exceed physical or operation limits of the
    vehicle.

    \param dof pointer to calling structure
    \param state_bound positive limit on x, 0 for none, used for euler angles
    \param rate_set_limit positive limit on rate setpoint, 0 for none
    \param Uval_up_limit upper limit on U value, no limit if = to lw
    \param Uval_lw_limit lower limit on U value, no limit if = to up

    \pre None, call after declaring structure
    \post Limits stored in structure for proper control

    \ingroup dof_t_methods
*/
void dof_init_limits(dof_t *dof, const float state_bound,
					 const float rate_set_limit, const float Uval_up_limit,
					 const float Uval_lw_limit)
{
	dof->state_bound = state_bound;
	dof->rate_set_limit = rate_set_limit;
	dof->Uval_up_limit = Uval_up_limit;
	dof->Uval_lw_limit = Uval_lw_limit;
}

/** Determines internal function of the control loop by enabling or disabling
    wraparound tracking, Kalman filtering.

    \param dof          pointer to calling structure
    \param wraparound   toggles wraparound tracking, for yaw only
    \param disc_deriv   (size 2) toggles discrete derivatives for [rate, accel]

    \pre None, call after declaring structure
    \post Operation flags are stored in structure for proper control

    \ingroup dof_t_methods
*/
void dof_init_flags(dof_t *dof, const uint8_t wraparound,
					const uint8_t disc_deriv[])
{
	dof->wraparound = wraparound;
	dof->disc_deriv[0] = disc_deriv[0];
	dof->disc_deriv[1] = disc_deriv[1];
}


/**	Initializes the lowpass coefficients for filtering discrete derivative
 	calculations.

 	\param dof				pointer to calling structure
 	\param lowpass_coeff[]	array of lowpass coefficients [x, x_dot, x_ddot]

 */
void dof_init_lowpass(dof_t* dof, const float lowpass_coeff[])
{
    unsigned int i;
    for(i = 0; i < 3; ++i) dof->lowpass_coeff[i] = lowpass_coeff[i];
}

/** Stores the input into the structure's internal state. If there are limits on
    the state, this will bound it to the proper values (wraparound style not
    saturation). If using discrete derivatives, they will be calculated in here.

    \param dof      pointer to calling structure
    \param x        primary state value
    \param dx_dt    state deriative
    \param d2x_dt2  state second derivative
    \param t        timestamp when state was valid

    \pre Struct is initialized
    \post New state is stored in dof

    \ingroup dof_t_methods
*/
void dof_set_state(dof_t *dof, const float x, const float dx_dt,
				   const float d2x_dt2, const float t)
{
	uint8_t i;

	for (i = 0; i < 4; ++i) dof->prev_state[i] = dof->state[i];

	// set new state
	dof->state[0] = x;
	dof->state[1] = dx_dt;
	dof->state[2] = d2x_dt2;
	dof->state[3] = t;

	if (dof->state_bound > 0.000001) // small tolerance for float precision
	{
		while (dof->state[0] > dof->state_bound)
			dof->state[0] -= 2 * dof->state_bound;
		while (dof->state[0] <= -dof->state_bound)
			dof->state[0] += 2 * dof->state_bound;
		assert(dof->state[0] <=  dof->state_bound);
		assert(dof->state[0] >= -dof->state_bound);
	}

	// calculate discrete derivatives for state if necessary
	if (dof->disc_deriv[0])
		dof_discrete_dxdt(dof);
	if (dof->disc_deriv[1])
		dof_discrete_d2xdt2(dof);
}

/** Stores the input into the structure's internal setpoint buffer that is not
    currently being used by the controller. This should be used at all times to
    set the setpoint rather than doing it manually. In general, the derivative
    and second derivative setpoints should be zero because the value PID will
    set the rate setpoint and the accel setpoint should always be zero. If the
    setpoint is outside of the state limit, this will bound it to the proper
    values (wraparound style not saturation).

    \param dof      pointer to calling structure
    \param x        primary state value setpoint
    \param dx_dt    state deriative setpoint (set by value PID)
    \param d2x_dt2  state second derivative setpoint
    \param t        timestamp when setpoint was valid

    \pre Struct is initialized, dof->setpt_updated == 0, control thread is not
         in the process of swapping
    \post New setpoint is stored in inactive buffer

    \ingroup dof_t_methods
*/
void dof_set_setpt(dof_t *dof, const float x, const float dx_dt,
				   const float d2x_dt2, const float t)
{
	dof->setpt[0] = x;
	dof->setpt[1] = dx_dt;
	dof->setpt[2] = d2x_dt2;
	dof->setpt[3] = t;

	if (dof->state_bound > 0.000001) // small tolerance for float precision
	{
		while (dof->setpt[0] > dof->state_bound)
			dof->setpt[0] -= 2 * dof->state_bound;
		while (dof->setpt[0] < -dof->state_bound)
			dof->setpt[0] += 2 * dof->state_bound;
		assert(dof->setpt[0] <=  dof->state_bound);
		assert(dof->setpt[0] >= -dof->state_bound);
	}
}

/** Uses the current and previous state value with the time step to calculate
    the discrete first derivative and store it in the state array.

    \param dof pointer to calling structure
    \return 0 if not updated, 1 if updated
.
    \post Stores the discrete derivative of state[0] into state[1]

    \ingroup dof_t_methods
*/
void dof_discrete_dxdt(dof_t *dof)
{
	static float filter_state = 0.0;

	float *stateval = dof->state;
	float dt = stateval[3] - dof->prev_state[3];

	if (dt <= 0.001) // don't calculate since time step is too small
	{
		return;
	}

	float diff = stateval[0] - dof->prev_state[0];

	// wraparound correct
	if (dof->state_bound > 0.000001)
	{
		if (diff > dof->state_bound)
			diff -= 2 * dof->state_bound;
		if (diff < -dof->state_bound)
			diff += 2 * dof->state_bound;
	}

	// Calculate discrete derivative
	float raw_deriv = diff / dt;

	// Use lowpass filter on derivative if enabled
	if ((dof->lowpass_coeff[1] > 0.0) && (dof->lowpass_coeff[1] < 1.0))
	{
		dof->state[1] = dof->lowpass_coeff[1] * raw_deriv
						+ (1.0 - dof->lowpass_coeff[1]) * filter_state;
	}
	else dof->state[1] = raw_deriv;

	filter_state = dof->state[1];
}

/** Uses the current and previous state value derivatives with the time step to
    calculate the discrete second derivative and store it in the state array.

    \param dof pointer to calling structure
    \return 0 if not updated, 1 if updated

    \pre set_state has been called this iteration of the control loop to
         properly set the prev_state and state values. The second and last
         elements in both of those arrays must be valid.
    \post Stores the discrete derivative of state[1] into state[2]

    \ingroup dof_t_methods
*/
void dof_discrete_d2xdt2(dof_t *dof)
{
	static float filter_state = 0.0;

	float *stateval = dof->state;

	float dt = stateval[3] - dof->prev_state[3];

	if (dt <= 0.001)	// don't calculate if dt is too small
	{
		return;
	}

	// Use lowpass filter on derivative if enabled
	float raw_deriv = (stateval[1] - dof->prev_state[1]) / dt;

	if ((dof->lowpass_coeff[2] > 0.0) && (dof->lowpass_coeff[2] < 1.0))
	{
		dof->state[2] = dof->lowpass_coeff[2] * raw_deriv
				        + (1.0 - dof->lowpass_coeff[2]) * filter_state;
	}
	else dof->state[2] = raw_deriv;

	filter_state = dof->state[2];
}

/** Calculates the time step since the last operation of the control loop by
    the timestamp on the prev_err (which it updates) and the highest timestamp
    from either the new setpoint or new state. Stores the value for use in
    calc_error and kalman predict functions.

    \param dof pointer to calling structure

    \pre state and setpt are valid. dof must have either gotten a state
         update, setpt update, or received a dof_continue command this loop.
    \post ctrl_dt valid, ctrl_error[3] and ctrl_prev_err[3] are valid

    \ingroup dof_t_methods
*/
void dof_calc_ctrl_dt(dof_t *dof)
{
	dof->ctrl_prev_err[3] = dof->ctrl_error[3];

	if (dof->state[3] >= dof->setpt[3])
		dof->ctrl_error[3] = dof->state[3];
	else
		dof->ctrl_error[3] = dof->setpt[3];

	dof->ctrl_dt = dof->ctrl_error[3] - dof->ctrl_prev_err[3];
}

/** Calculates the control error by taking the difference between the setpoint
    and the actual state value. Additionally, takes care of the prev_error,
    error derivative, and integral terms. After calling this function, the
    struct is ready to run the PID algorithms. Uses the filtered state values
    if the filter flag is set. Also uses wraparound error tracking if flag is
    set. Also takes the discrete derivative if flags are set.

    \param dof pointer to calling structure
    \param i index of the state vector to calculate the error, i < 3
    \param integrate flag to turn off integration while ctrl loop isn't running

    \pre state_swp has already been called and all values of the state
         vector and prev_state vector are valid in addition to setpoints for
         given index. calc_ctrl_dt must also have been called to have a valid
         dt. If the dt is <= 0, the function will simply return without doing
         anything. If kalman filtering is being used, kalman extract has been
         called after predicting/correcting this loop.
    \post ctrl_(error, prev_err, derr_dt, integral)[i] all valid

    \ingroup dof_t_methods
*/
void dof_calc_error(dof_t *dof, const uint8_t i, const uint8_t integrate)
{
	assert(i < 3);

	// dt is too small
	if (dof->ctrl_dt <= 0.001)
	{
		return;
	}

	float *stateval = dof->state;	// grab current state

	// set previous and current controller errors
	dof->ctrl_prev_err[i] = dof->ctrl_error[i];
	dof->ctrl_error[i] = dof->setpt[i] - stateval[i];

	// wraparound error for the main value
	if ((i == 0) && (dof->wraparound) && (dof->state_bound > 0.000001))
	{
		if (dof->ctrl_error[i] > dof->state_bound)
			dof->ctrl_error[i] -= 2 * dof->state_bound;
		if (dof->ctrl_error[i] < -dof->state_bound)
			dof->ctrl_error[i] += 2 * dof->state_bound;
	}

	// Calculate discrete derivative
	float raw_deriv = (dof->ctrl_error[i] - dof->ctrl_prev_err[i])
					  / dof->ctrl_dt;

	// lowpass filter if enabled; otherwise, use raw discrete derivative
	if ((dof->lowpass_coeff[i] > 0.0) && (dof->lowpass_coeff[i] < 1.0))
	{
		dof->ctrl_derr_dt[i] = dof->lowpass_coeff[i] * raw_deriv
		                       + (1.0 - dof->lowpass_coeff[i])
					 	 	   * dof->ctrl_derr_dt[i];
	}
	else dof->ctrl_derr_dt[i] = raw_deriv;

	if (integrate)
	{
		dof->ctrl_integral[i] += 0.5 * dof->ctrl_dt * (dof->ctrl_error[i]
		                         + dof->ctrl_prev_err[i]);
	}
}

/** Calculates the derivative setpoint of the dof by using a standard PID.
    Uses the internal gains to carry out a standard PID calculation of the form:

    \code
    rate_setpoint = Kp * error[0] + Ki * int_error[0] + Kd * derror_dt[0]
    \endcode


    \param dof pointer to calling structure
    \param use_derr_dt enable or disable using the derr_dt in PID

    \pre PID gains and rate setpoint limits initialized, calc_error(0) has been
         called this loop.
    \post Sets the internal setpoint[1] to the output of the PID calculation.
          Limits the output if rate_set_limit is set (saturation, not wrap)

    \ingroup dof_t_methods
*/
void dof_value_PID(dof_t *dof, const uint8_t use_derr_dt)
{
	dof->setpt[1] = (dof->value_gains[0] * dof->ctrl_error[0])
	                + (dof->value_gains[1] * dof->ctrl_integral[0]);

	float *stateval = dof->state;

	if (use_derr_dt == 1) // use discrete deriv of value error
		dof->setpt[1] += dof->value_gains[2] * dof->ctrl_derr_dt[0];
	else // use kalman filter velocity
		dof->setpt[1] += dof->value_gains[2] * -stateval[1];

	if (dof->rate_set_limit > 0.000001)
	{
		if (dof->setpt[1] > dof->rate_set_limit)
			dof->setpt[1] = dof->rate_set_limit;
		if (dof->setpt[1] < -dof->rate_set_limit)
			dof->setpt[1] = -dof->rate_set_limit;
	}

	dof->velocity = dof->setpt[1];
}

/** Calculates the Uval of the dof by using a standard PID on the derivative.
    Uses the internal gains to carry out a standard PID calculation of the form:

    \code
    Uval = inertia * (Kp * error[1] + Ki * int_error[1] + Kd * error[2])
    \endcode

    Uses the acceleration instead of the discrete derivative of the rate error
    because it is usually better data. This requires the accel setpoint to be
    zero to work properly.

    \todo decide if the D gain for the rate controller should affect the actual
          accel error or if it should affect the derivative of the rate error

    \param dof         pointer to calling structure
    \param use_derr_dt if set to 1, the derivative term will use the
                       differentiation of the error, otherwise it will use the
                       acceleration error (-accel)

    \pre PID gains and Uval limits initialized, calc_error(1) and calc_error(2)
         have been called this loop (after a valid rate setpoint is calculated
         by value_PID).
    \post Sets the internal Uval to the output of the PID calculation.
          Limits the output if Uval limits are set (saturation, not wrap)

    \ingroup dof_t_methods
*/
void dof_rate_PID(dof_t *dof, const uint8_t use_derr_dt)
{
	dof->Uval = (dof->inertia * ((dof->rate_gains[0] * dof->ctrl_error[1])
				 + (dof->rate_gains[1] * dof->ctrl_integral[1])));

	float *stateval = dof->state;

	if (use_derr_dt == 1) // use derivative of velocity error
		dof->Uval += dof->inertia * (dof->rate_gains[2] * dof->ctrl_derr_dt[1]);
	else // use accel error based on kalman filter feedback into state
		dof->Uval += dof->inertia * (dof->rate_gains[2] * -stateval[2]);

	if (dof->Uval_up_limit > dof->Uval_lw_limit + 0.000001)
	{
		if (dof->Uval > dof->Uval_up_limit)
			dof->Uval = dof->Uval_up_limit;
		if (dof->Uval < dof->Uval_lw_limit)
			dof->Uval = dof->Uval_lw_limit;
	}
}

// End of File
