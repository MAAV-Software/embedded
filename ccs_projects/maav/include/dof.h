#ifndef DOF_H
#define DOF_H

/** \file dof.h
    \brief Declaration of the DOF structure and supporting functions
    
    Defines the DOF structure and declares the support functions allowing the 
    structure to be used for implementing a controller for the quadrotor.

    \author Sajan Patel
    \version 3.0
    \date July 2014

    \see dof.c
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/// Struct containing all necessary ctrl vales for a single degree of freedom
/**
    Structure contains state, setpoints, filter matrices, controller gains, 
    internal state value errors, derivatives, and integrals, and output 
    U value. Used with supporting functions detailed below to implement a 
    portable stacked PID controller on a signal degree of freedom with the form:
       
        \code
        error in value -> value PID -> desired rate
        error in rate -> rate PID -> U value
        \endcode

    This structure is set up for a parallel sensor update and control loop 
    process. It can also be used in a serial process if both threads shown below
    are stacked serially with the update thread placed before the control 
    thread. To implement the dof structure with PID control, use the code shown 
    below.
    
    \code
    dof_init_struct(dof, ...);
    dof_init_gains(dof, ...);
    dof_init_limits(dof, ...);
    dof_init_flags(dof, ...);
	\endcode

	Once the initialization is done. Run the following code in the main control
	loop.

    \code
    dof_calc_ctrl_dt(dof);

    dof_calc_error(dof, 0);
    dof_value_PID(dof);
    // insert nonlinear adjustment to velocity

    dof_calc_error(dof, 1);
    dof_calc_error(dof, 2);
    dof_rate_PID(dof);
    // insert nonlinear adjustment to force
    \endcode

    \see \link dof_t_methods dof_t supporting functions \endlink
*/
typedef struct _dof_t
{

    /// \name dof state values (all with timestamps)
    /// \{ 
    float state[4];         ///< [x, dx/dt, d^2x/dt^2, t] for this DOF
    float setpt[4];         ///< [x, dx/dt, d^2x/dt^2, t] desired for this DOF
    float prev_state[4];    ///< [x, dx/dt, d^2x/dt^2, t] of last time step
    float Uval;           ///< U value output for the DOF
    float inertia;        ///< Mass or moment of inertia for the DOF
    float velocity;		  ///< Velocity output of value PID for this DOF
    /// \}

    /// \name controller values
    /// \{ 
    float value_gains[3];   ///< [Kp, Ki, Kd] for the value -> desired rate PID
    float rate_gains[3];    ///< [Kp, Ki, Kd] for the rate -> U value PID
    float ctrl_dt;          ///< time since controller last ran
    float ctrl_error[4];    ///< setpoint - state, with a timestamp
    float ctrl_prev_err[4]; ///< previous error values
    float ctrl_derr_dt[3];  ///< discrete derivative of the error
    float ctrl_integral[3]; ///< integral of setpoint - state
    float lowpass_coeff[3]; ///< low pass filter coeffiicients on error deriv
    /// \}

    /// \name flags, options, and limits
    /// \{ 
    float state_bound;      ///< positive limit on x, 0 for none, for euler ang
    float rate_set_limit;   ///< positive limit on rate setpoint, 0 for none
    float Uval_up_limit;    ///< upper limit on U value, no limit if = to lw
    float Uval_lw_limit;    ///< lower limit on U value, no limit if = to up
    uint8_t wraparound;	///< if 1 triggers wraparound tracking, for yaw
    uint8_t disc_deriv[2];  ///< use discrete derivatives for [rate, accel]
    /// \}
} dof_t;

/// \defgroup dof_t_methods dof_t supporting functions
///
/// These functions act like member methods for dof structures
/// the first argument is always a pointer (may be const or not) to the dof
/// struct that is performing the action (this functions like *this).
///
/// \see \link _dof_t \endlink
///
/// \{ 

/// \name Initialization and Updates
/// \{ 
/// Initializes dynamic values given the first state
void dof_init_struct(dof_t* dof, const float x, const float dx_dt,
					 const float d2x_dt2, const float t, const float inertia);

/// Sets the PID gains
void dof_set_gains(dof_t* dof, float value_gains[], float rate_gains[]);

/// Initializes the PID output limits
void dof_init_limits(dof_t* dof, const float state_bound,
					 const float rate_set_limit, const float Uval_up_limit,
					 const float Uval_lw_limit);

/// Initializes the operational flags
void dof_init_flags(dof_t* dof, const uint8_t wraparound,
					const uint8_t disc_deriv[]);

/// Initializes the lowpass coefficients
void dof_init_lowpass(dof_t* dof, const float lowpass_coeff[]);
/// \}

/// \name State, setpoint, and error calculation
/// \{ 
/// Sets the internal state buffer
void dof_set_state(dof_t* dof, const float x, const float dx_dt,
				   const float d2x_dt2, const float t);

/// Sets the internal setpt buffer
void dof_set_setpt(dof_t* dof, const float x, const float dx_dt,
				   const float d2x_dt2, const float t);

/// Calculates the discrete derivative of the state value (to get rate)
void dof_discrete_dxdt(dof_t* dof);

/// Calculates the discrete derivative of the state rate (to get accel)
void dof_discrete_d2xdt2(dof_t* dof);

/// Sets the internal dt from time of last control loop, stores in ctrl_dt
void dof_calc_ctrl_dt(dof_t* dof);

/// Sets the internal ctrl_error, prev_error, ctrl_derr_dt, and ctrl_integral
void dof_calc_error(dof_t* dof, const uint8_t i, const uint8_t integrate);
/// \}

/// \name Execute controls
/// \{
/// Calculates the rate setpoint from the value PID
void dof_value_PID(dof_t* dof, const uint8_t use_derr_dt);

/// Calculates the U value from the rate PID
void dof_rate_PID(dof_t* dof, const uint8_t use_derr_dt);
/// \}

#ifdef __cplusplus
}
#endif

#endif // DOF_H
