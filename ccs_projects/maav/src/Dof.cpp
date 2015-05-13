/** \file dof.c
    \brief Implimentation of the DOF structure support functions

    Implements the functionality and support of the DOF structure allowing the
    structure to be used for implementing a controller for the quadrotor.

    \author Mathew Karashin, Sajan Patel
    \version 3.0
    \date July 2014

    \see dof.h
*/
#include <stdint.h>
#include <math.h>
#include <Dof.hpp>

/** Constructor for Dof class ***/
Dof::Dof(const float x, const float DxDt, const float D2xDt2,
		  const float t, const float inertia, const float stateBound,
		  const float rateSetLimit, const float UvalUpLimit,
		  const float UvalLwLimit, const uint8_t flags)
{
	initState(x, DxDt, D2xDt2, t, inertia);
	initLimits(stateBound, rateSetLimit, UvalUpLimit, UvalLwLimit);
	initFlags(flags);
}

Dof::Dof()
{
	inertia = 0;
	Uval = 0;
	velocity = 0;
	stateBound = 0;
	rateSetLimit = 0;   	///< positive limit on rate setpoint, 0 for none
	UvalUpLimit = 0;		///< upper limit on U value, no limit if = to lw
	UvalLwLimit = 0;  		///< lower limit on U value, no limit if = to up
	flags = 0;
	ctrlDt = 0;         		///< time since controller last ran
	for (uint8_t i = 0; i < 3; ++i)
	{
		valueGains[i] = 0;  		///< [Kp, Ki, Kd] for the value -> desired rate PID
		rateGains[i] = 0;   		///< [Kp, Ki, Kd] for the rate -> U value PID
		ctrlDerrDt[i] = 0;  		///< discrete derivative of the error [x, dx, d2x]
		ctrlIntegral[i] = 0;		///< integral of setpoint - state [x, dx, d2x]
	}
	for (uint8_t i = 0; i < 4; ++i)
	{
		ctrlError[i] = 0;   		///< setpoint - state, with a timestamp
		ctrlPrevErr[i] = 0;		///< previous error values
	}
}

Dof::~Dof() {}

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
void Dof::initState(const float x, const float DxDt, const float D2xDt2,
  	   	 	 	 	const float t, const float mass)
{
	for (uint8_t i = 0; i < 4; ++i) // init state vals to 0
	{
		state[i]       = 0.0;
		setpt[i]       = 0.0;
		ctrlPrevErr[i] = 0.0;
		ctrlError[i]   = 0.0;
	}

	for (uint8_t i = 0; i < 3; ++i)	// init error deriv and error integral to 0
	{
		ctrlDerrDt[i]   = 0.0;
		ctrlIntegral[i] = 0.0;
	}

	// assign user-defined intial state
	state[VAL]      = x;
	state[RATE]     = DxDt;
	state[ACCEL]    = D2xDt2;
	state[TIME]     = t;
	ctrlError[TIME] = t;
	inertia         = mass;
	velocity        = 0.0; // init veloctiy to 0
	Uval            = 0.0;	// init force to 0
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
void Dof::setGains(const float _valueGains[3], const float _rateGains[3])
{
	// Reset integral if integral gain changes
	if (valueGains[KI] != _valueGains[KI])
	{
		if (_valueGains[KI] != 0.0)	ctrlIntegral[VAL] *= valueGains[KI] / _valueGains[KI];
		else						ctrlIntegral[VAL]  = 0.0;
	}

	// Do the same for rate integral
	if (rateGains[KI] != _rateGains[KI])
	{
		if (_rateGains[KI] != 0.0) 	ctrlIntegral[RATE] *= rateGains[KI] / _rateGains[KI];
		else						ctrlIntegral[RATE]  = 0.0;
	}

	for (uint8_t i = 0; i < 3; ++i) // assign new gains
	{
		valueGains[i] = _valueGains[i];
		rateGains[i]  = _rateGains[i];
	}
}

/** Defintes the limits on state, desired velocity, and force for the structure
    to ensure the PID output does not exceed physical or operation limits of the
    vehicle.

    \param state_bound positive limit on x, 0 for none, used for euler angles
    \param rate_set_limit positive limit on rate setpoint, 0 for none
    \param Uval_up_limit upper limit on U value, no limit if = to lw
    \param Uval_lw_limit lower limit on U value, no limit if = to up

    \pre None, call after declaring structure
    \post Limits stored in structure for proper control

    \ingroup dof_t_methods
*/
void Dof::initLimits(const float _stateBound, const float _rateSetLimit,
		  	  	  	 const float _UvalUpLimit, const float _UvalLwLimit)
{
	stateBound   = _stateBound;
	rateSetLimit = _rateSetLimit;
	UvalUpLimit  = _UvalUpLimit;
	UvalLwLimit  = _UvalLwLimit;
}

/** Determines internal function of the control loop by enabling or disabling
    wraparound tracking, Kalman filtering.

    \pre None, call after declaring structure
    \post Operation flags are stored in structure for proper control

    \ingroup dof_t_methods
*/
void Dof::initFlags(const uint8_t _flags)
{
	flags = _flags;
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
void Dof::setState(const float x, const float DxDt, const float D2xDt2, const float t)
{
	for (uint8_t i = 0; i < 4; ++i) prevState[i] = state[i];

	// set new state
	state[VAL]   = x;
	state[RATE]  = DxDt;
	state[ACCEL] = D2xDt2;
	state[TIME]  = t;

	if (stateBound > 0.000001) // small tolerance for float precision
	{
		while (state[VAL] >   stateBound) state[VAL] -= 2 * stateBound;
		while (state[VAL] <= -stateBound) state[VAL] += 2 * stateBound;
	}

	// calculate discrete derivatives for state if necessary
	if (flags & DISC_RATE)  discreteDxDt();
	if (flags & DISC_ACCEL) discreteD2xDt2();
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
void Dof::setSetpt(const float x, const float DxDt, const float D2xDt2, const float t)
{
	setpt[VAL]   = x;
	setpt[RATE]  = DxDt;
	setpt[ACCEL] = D2xDt2;
	setpt[TIME]  = t;

	if (stateBound > 0.000001) // small tolerance for float precision
	{
		while (setpt[VAL] >  stateBound) setpt[VAL] -= 2 * stateBound;
		while (setpt[VAL] < -stateBound) setpt[VAL] += 2 * stateBound;
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
void Dof::discreteDxDt()
{
	float dt = state[TIME] - prevState[TIME];
	if (dt <= 0.001) // don't calculate since time step is too small
	{
		return;
	}

	float diff = state[VAL] - prevState[VAL];
	if (stateBound > 0.000001) // wraparound correct
	{
		if (diff >  stateBound) diff -= 2 * stateBound;
		if (diff < -stateBound) diff += 2 * stateBound;
	}

	state[RATE] = diff / dt; 	// Calculate discrete derivative
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
void Dof::discreteD2xDt2()
{
	float dt = state[TIME] - prevState[TIME];
	if (dt <= 0.001)	// don't calculate if dt is too small
	{
		return;
	}
	state[ACCEL] = (state[RATE] - prevState[RATE]) / dt; // calc disc accel
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
void Dof::calcCtrlDt()
{
	ctrlPrevErr[TIME] = ctrlError[TIME];

	if (state[TIME] >= setpt[TIME]) ctrlError[TIME] = state[TIME];
	else 							ctrlError[TIME] = setpt[TIME];

	ctrlDt = ctrlError[TIME] - ctrlPrevErr[TIME];
}

/** Calculates the control error by taking the difference between the setpoint
    and the actual state value. Additionally, takes care of the prev_error,
    error derivative, and integral terms. After calling this function, the
    struct is ready to run the PID algorithms. Uses the filtered state values
    if the filter flag is set. Also uses wraparound error tracking if flag is
    set. Also takes the discrete derivative if flags are set.

    \param idx index of the state vector to calculate the error, i < 3
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
void Dof::calcError(const uint8_t idx, const bool integrate)
{
	// dt is too small
	if (ctrlDt <= 0.001)
	{
		return;
	}

	// set previous and current controller errors
	ctrlPrevErr[idx] = ctrlError[idx];
	ctrlError[idx] = setpt[idx] - state[idx];

	// wraparound error for the main value
	if ((idx == 0) && (flags & WRAP_AROUND) && (stateBound > 0.000001))
	{
		if (ctrlError[idx] >  stateBound) ctrlError[idx] -= 2 * stateBound;
		if (ctrlError[idx] < -stateBound) ctrlError[idx] += 2 * stateBound;
	}

	// Calculate discrete derivative
	ctrlDerrDt[idx] = (ctrlError[idx] - ctrlPrevErr[idx]) / ctrlDt;

	if (integrate)
		ctrlIntegral[idx] += 0.5 * ctrlDt * (ctrlError[idx] + ctrlPrevErr[idx]);
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
void Dof::valuePID(const bool useDerrDt)
{
	// calculate PI output of value PI and assign it as the rate setpoint
	setpt[RATE] = (valueGains[KP] * ctrlError[VAL]) + (valueGains[KI] * ctrlIntegral[VAL]);

	// Calculate the D of PID output and add it to the rate setpoint. This is
	// done in 1 of 2 ways: 1) by discrete derivative if useDerrDt is 1, or
	// 2) using the filtered rate
	if (useDerrDt)	setpt[RATE] += valueGains[KD] * ctrlDerrDt[VAL];
	else 			setpt[RATE] += valueGains[KD] * -state[RATE];

	if (rateSetLimit > 0.000001) // saturate at limits
	{
		if (setpt[RATE] >  rateSetLimit) setpt[RATE] =  rateSetLimit;
		if (setpt[RATE] < -rateSetLimit) setpt[RATE] = -rateSetLimit;
	}

	velocity = setpt[RATE]; // update output velocity member var
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
void Dof::ratePID(const bool useDerrDt)
{
	// Calculate the PI of PID output for the Uval
	Uval = inertia * ((rateGains[KP] * ctrlError[RATE]) + (rateGains[KI] * ctrlIntegral[RATE]));

	// Calculate the D of PID output and add it to the rate setpoint. This is
	// done in 1 of 2 ways: 1) by discrete derivative if useDerrDt is 1, or
	// 2) using the filtered acceleration
	if (useDerrDt)	Uval += inertia * (rateGains[KD] * ctrlDerrDt[RATE]);
	else			Uval += inertia * (rateGains[KD] * - state[ACCEL]);

	if (UvalUpLimit > (UvalLwLimit + 0.000001)) // saturate at limits
	{
		if (Uval > UvalUpLimit) Uval = UvalUpLimit;
		if (Uval < UvalLwLimit) Uval = UvalLwLimit;
	}
}

// End of File
