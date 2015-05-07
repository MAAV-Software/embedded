/** \file dof.c
    \brief Implimentation of the DOF structure support functions

    Implements the functionality and support of the DOF structure allowing the
    structure to be used for implementing a controller for the quadrotor.

    \author Mathew Karashin, Sajan Patel
    \version 3.0
    \date July 2014

    \see dof.h
*/

#include <Dof.hpp>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <stdint.h>
#include <stdbool.h>


/** Constructor for Dof class ***/
Dof::Dof(const float x, const float DxDt, const float D2xDt2,
		  const float t, const float inertia, const float stateBound,
		  const float rateSetLimit, const float UvalUpLimit,
		  const float UvalLwLimit, const uint8_t flags,
		  const float lowpassCoeff[3])
{
	initState(x, DxDt, D2xDt2, t, inertia);
	initLimits(stateBound, rateSetLimit, UvalUpLimit, UvalLwLimit);
	initFlags(flags);
	initLowpass(lowpassCoeff);
}

Dof::Dof()
{
	inertia = 0;
	Uval = 0;
	velocity = 0;
	_stateBound = 0;
	_rateSetLimit = 0;   	///< positive limit on rate setpoint, 0 for none
	_UvalUpLimit = 0;		///< upper limit on U value, no limit if = to lw
	_UvalLwLimit = 0;  		///< lower limit on U value, no limit if = to up
	_flags = 0; ///< bit 0 for discrete rate, bit 1 for discrete accel, bit 2 for wraparound
	/// \}

	_ctrlDt = 0;         		///< time since controller last ran
    /// \name controller values
    /// \{
	for (uint8_t i = 0; i < 3; ++i)
	{
		_valueGains[i] = 0;  		///< [Kp, Ki, Kd] for the value -> desired rate PID
		_rateGains[i] = 0;   		///< [Kp, Ki, Kd] for the rate -> U value PID
		_ctrlDerrDt[i] = 0;  		///< discrete derivative of the error [x, dx, d2x]
		_ctrlIntegral[i] = 0;		///< integral of setpoint - state [x, dx, d2x]
		_lowpassCoeff[i] = 0; 	///< low pass filter coeffiicients on error deriv
	}
	for (uint8_t i = 0; i < 4; ++i)
	{
		_ctrlError[i] = 0;   		///< setpoint - state, with a timestamp
		_ctrlPrevErr[i] = 0;		///< previous error values
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
		state[i] = 0.0;
		setpt[i] = 0.0;
		_ctrlPrevErr[i] = 0.0;
		_ctrlError[i] = 0.0;
	}

	for (uint8_t i = 0; i < 3; ++i)	// init error deriv and error integral to 0
	{
		_ctrlDerrDt[i] = 0.0;
		_ctrlIntegral[i] = 0.0;
	}

	// assign user-defined intial state
	state[val] = x;
	state[rate] = DxDt;
	state[accel] = D2xDt2;
	state[time] = t;
	_ctrlError[time] = t;
	inertia = mass;
	velocity = 0.0; // init veloctiy to 0
	Uval = 0.0;	// init force to 0
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
void Dof::setGains(float valueGains[3], float rateGains[3])
{
	// Reset integral if integral gain changes
	if (_valueGains[Ki] != valueGains[Ki])
	{
		if (valueGains[Ki] != 0.0)	_ctrlIntegral[val] *= _valueGains[Ki] / valueGains[Ki];
		else						_ctrlIntegral[val] = 0.0;
	}

	// Do the same for rate integral
	if (_rateGains[Ki] != rateGains[Ki])
	{
		if (rateGains[Ki] != 0.0) 	_ctrlIntegral[rate] *= _rateGains[Ki] / rateGains[Ki];
		else						_ctrlIntegral[rate] = 0.0;
	}

	for (uint8_t i = 0; i < 3; ++i) // assign new gains
	{
		_valueGains[i] = valueGains[i];
		_rateGains[i] = rateGains[i];
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
void Dof::initLimits(const float stateBound, const float rateSetLimit,
		  	  	  	   const float UvalUpLimit, const float UvalLwLimit)
{
	_stateBound = stateBound;
	_rateSetLimit = rateSetLimit;
	_UvalUpLimit = UvalUpLimit;
	_UvalLwLimit = UvalLwLimit;
}

/** Determines internal function of the control loop by enabling or disabling
    wraparound tracking, Kalman filtering.

    \pre None, call after declaring structure
    \post Operation flags are stored in structure for proper control

    \ingroup dof_t_methods
*/
void Dof::initFlags(const uint8_t flags)
{
	_flags = flags;
}


/**	Initializes the lowpass coefficients for filtering discrete derivative
 	calculations.

 	\param dof				pointer to calling structure
 	\param lowpass_coeff[]	array of lowpass coefficients [x, x_dot, x_ddot]

 */
void Dof::initLowpass(const float lowpassCoeff[3])
{
    for (uint8_t i = 0; i < 3; ++i) _lowpassCoeff[i] = lowpassCoeff[i];
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
void Dof::setState(const float x, const float DxDt, const float D2xDt2,
					 const float t)
{
	for (uint8_t i = 0; i < 4; ++i) prevState[i] = state[i];

	// set new state
	state[val] = x;
	state[rate] = DxDt;
	state[accel] = D2xDt2;
	state[time] = t;

	if (_stateBound > 0.000001) // small tolerance for float precision
	{
		while (state[val] >   _stateBound) state[val] -= 2 * _stateBound;
		while (state[val] <= -_stateBound) state[val] += 2 * _stateBound;
		//assert(state[0] <=  _stateBound);
		//assert(state[0] >= -_stateBound);
	}

	// calculate discrete derivatives for state if necessary
	if (_flags & DISC_RATE)  discreteDxDt();
	if (_flags & DISC_ACCEL) discreteD2xDt2();
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
void Dof::setSetpt(const float x, const float DxDt, const float D2xDt2,
					 const float t)
{
	setpt[val] = x;
	setpt[rate] = DxDt;
	setpt[accel] = D2xDt2;
	setpt[time] = t;

	if (_stateBound > 0.000001) // small tolerance for float precision
	{
		while (setpt[val] >  _stateBound) setpt[val] -= 2 * _stateBound;
		while (setpt[val] < -_stateBound) setpt[val] += 2 * _stateBound;
		//assert(setpt[0] <=  _stateBound);
		//assert(setpt[0] >= -_stateBound);
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
	static float filterState = 0.0;
	float dt = state[time] - prevState[time];
	if (dt <= 0.001) // don't calculate since time step is too small
	{
		return;
	}

	float diff = state[val] - prevState[val];
	if (_stateBound > 0.000001) // wraparound correct
	{
		if (diff >  _stateBound) diff -= 2 * _stateBound;
		if (diff < -_stateBound) diff += 2 * _stateBound;
	}

	// Calculate discrete derivative
	float rawDeriv = diff / dt;
	// Use lowpass filter on derivative if enabled
	if ((_lowpassCoeff[1] > 0.0) && (_lowpassCoeff[1] < 1.0))
	{
		state[rate] = _lowpassCoeff[1] * rawDeriv
					+ (1.0 - _lowpassCoeff[1]) * filterState;
	}
	else state[rate] = rawDeriv;

	filterState = state[rate];
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
	static float filterState = 0.0;

	float dt = state[time] - prevState[time];
	if (dt <= 0.001)	// don't calculate if dt is too small
	{
		return;
	}

	// Use lowpass filter on derivative if enabled
	float rawDeriv = (state[rate] - prevState[rate]) / dt;
	if ((_lowpassCoeff[2] > 0.0) && (_lowpassCoeff[2] < 1.0))
	{
		state[accel] = _lowpassCoeff[2] * rawDeriv
				        + (1.0 - _lowpassCoeff[2]) * filterState;
	}
	else state[accel] = rawDeriv;

	filterState = state[accel];
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
	_ctrlPrevErr[time] = _ctrlError[time];

	if (state[time] >= setpt[time]) _ctrlError[time] = state[time];
	else _ctrlError[time] = setpt[time];

	_ctrlDt = _ctrlError[time] - _ctrlPrevErr[time];
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
void Dof::calcError(const uint8_t idx, const uint8_t integrate)
{
	//assert(i < 3);

	// dt is too small
	if (_ctrlDt <= 0.001)
	{
		return;
	}

	// set previous and current controller errors
	_ctrlPrevErr[idx] = _ctrlError[idx];
	_ctrlError[idx] = setpt[idx] - state[idx];

	// wraparound error for the main value
	if ((idx == 0) && (_flags & WRAP_AROUND) && (_stateBound > 0.000001))
	{
		if (_ctrlError[idx] >  _stateBound) _ctrlError[idx] -= 2 * _stateBound;
		if (_ctrlError[idx] < -_stateBound) _ctrlError[idx] += 2 * _stateBound;
	}

	// Calculate discrete derivative
	float rawDeriv = (_ctrlError[idx] - _ctrlPrevErr[idx]) / _ctrlDt;
	// lowpass filter if enabled; otherwise, use raw discrete derivative
	if ((_lowpassCoeff[idx] > 0.0) && (_lowpassCoeff[idx] < 1.0))
	{
		_ctrlDerrDt[idx] = _lowpassCoeff[idx] * rawDeriv
							+ (1.0 - _lowpassCoeff[idx]) * _ctrlDerrDt[idx];
	}
	else _ctrlDerrDt[idx] = rawDeriv;

	if (integrate)
	{
		_ctrlIntegral[idx] += 0.5 * _ctrlDt
								* (_ctrlError[idx] + _ctrlPrevErr[idx]);
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
void Dof::valuePID(const uint8_t useDerrDt)
{
	// calculate PI output of value PI and assign it as the rate setpoint
	setpt[rate] = (_valueGains[Kp] * _ctrlError[val])
					+ (_valueGains[Ki] * _ctrlIntegral[val]);

	// Calculate the D of PID output and add it to the rate setpoint. This is
	// done in 1 of 2 ways: 1) by discrete derivative if useDerrDt is 1, or
	// 2) using the filtered rate
	if (useDerrDt)	setpt[rate] += _valueGains[Kd] * _ctrlDerrDt[val];
	else 			setpt[rate] += _valueGains[Kd] * -state[rate];

	if (_rateSetLimit > 0.000001) // saturate at limits
	{
		if (setpt[rate] >  _rateSetLimit) setpt[rate] =  _rateSetLimit;
		if (setpt[rate] < -_rateSetLimit) setpt[rate] = -_rateSetLimit;
	}

	velocity = setpt[rate]; // update output velocity member var
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
void Dof::ratePID(const uint8_t useDerrDt)
{
	// Calculate the PI of PID output for the Uval
	Uval = inertia * ((_rateGains[Kp] * _ctrlError[rate])
				 	 	 + (_rateGains[Ki] * _ctrlIntegral[rate]));

	// Calculate the D of PID output and add it to the rate setpoint. This is
	// done in 1 of 2 ways: 1) by discrete derivative if useDerrDt is 1, or
	// 2) using the filtered acceleration
	if (useDerrDt)	Uval += inertia * (_rateGains[Kd] * _ctrlDerrDt[rate]);
	else			Uval += inertia * (_rateGains[Kd] * - state[accel]);

	if (_UvalUpLimit > (_UvalLwLimit + 0.000001)) // saturate at limits
	{
		if (Uval > _UvalUpLimit) Uval = _UvalUpLimit;
		if (Uval < _UvalLwLimit) Uval = _UvalLwLimit;
	}
}

// End of File
