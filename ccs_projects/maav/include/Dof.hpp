#ifndef DOF_HPP
#define DOF_HPP

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


// Defines for bitfields
#define DISC_RATE 		0x1
#define DISC_ACCEL	 	0x2
#define WRAP_AROUND		0x4

// Enum for gain indeces in data structures
enum PIDGainsEnum {Kp, Ki, Kd};
enum DofStateEnum {val, rate, accel, time};

/// Class containing all necessary ctrl vales for a single degree of freedom
/**
   Class contains state, setpoints, filter matrices, controller gains,
    internal state value errors, derivatives, and integrals, and output
    U value. Used with supporting functions detailed below to implement a
    portable stacked PID controller on a signal degree of freedom with the form:

        \code
        error in value -> value PID -> desired rate
        error in rate -> rate PID -> U value
        \endcode

    This class is set up for a parallel sensor update and control loop
    process. It can also be used in a serial process if both threads shown below
    are stacked serially with the update thread placed before the control
    thread. To implement the dof class with PID control, use the code shown
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
class Dof
{
private:
	/// \name flags, options, and limits
	/// \{
	float 	_stateBound;      	///< positive limit on value state, 0 for none
	float 	_rateSetLimit;   	///< positive limit on rate setpoint, 0 for none
	float 	_UvalUpLimit;		///< upper limit on U value, no limit if = to lw
	float 	_UvalLwLimit;  		///< lower limit on U value, no limit if = to up
	uint8_t _flags; ///< bit 0 for discrete rate, bit 1 for discrete accel, bit 2 for wraparound

	/// \}

    /// \name controller values
    /// \{
    float _valueGains[3];  		///< [Kp, Ki, Kd] for the value -> desired rate PID
    float _rateGains[3];   		///< [Kp, Ki, Kd] for the rate -> U value PID
    float _ctrlDt;         		///< time since controller last ran
    float _ctrlError[4];   		///< setpoint - state, with a timestamp
    float _ctrlPrevErr[4];		///< previous error values
    float _ctrlDerrDt[3];  		///< discrete derivative of the error [x, dx, d2x]
    float _ctrlIntegral[3];		///< integral of setpoint - state [x, dx, d2x]
    float _lowpassCoeff[3]; 	///< low pass filter coeffiicients on error deriv
    /// \}

    /// Calculates the discrete derivative of the state value (to get rate)
    void discreteDxDt();

    /// Calculates the discrete derivative of the state rate (to get accel)
    void discreteD2xDt2();

    /// Sets the internal dt from time of last control loop, stores in ctrl_dt
    void calcCtrlDt();

    /// Sets the internal ctrl_error, prev_error, ctrl_derr_dt, and ctrl_integral
    void calcError(const uint8_t idx, const uint8_t integrate);

public:
    /// \name dof state values (all with timestamps)
    /// \{
    float state[4];         ///< [x, dx/dt, d^2x/dt^2, t] for this DOF
    float setpt[4];         ///< [x, dx/dt, d^2x/dt^2, t] desired for this DOF
    float prevState[4];    ///< [x, dx/dt, d^2x/dt^2, t] of last time step
    float Uval;           ///< U value output for the DOF
    float inertia;        ///< Mass or moment of inertia for the DOF
    float velocity;		  ///< Velocity output of value PID for this DOF
    /// \}

    // Constructor
    Dof(const float x, const float DxDt, const float D2xDt2,
    	const float t, const float inertia, const float stateBound,
		const float rateSetLimit, const float UvalUpLimit,
		const float UvalLwLimit, const uint8_t flags,
		const float lowpassCoeff[3]);

    // Destructor
    ~Dof();

    /// \name Initialization and Updates
    /// \{
    /// Initializes dynamic values given the first state
    void initState(const float x, const float DxDt, const float D2xDt2,
    		   	   	 const float t, const float mass);

    /// Sets the PID gains
    void setGains(float valueGains[3], float rateGains[3]);

    /// Initializes the PID output limits
    void initLimits(const float stateBound, const float rateSetLimit,
    				  const float UvalUpLimit, const float UvalLwLimit);

    /// Initializes the operational flags
    void initFlags(const uint8_t flags);

    /// Initializes the lowpass coefficients
    void initLowpass(const float lowpassCoeff[3]);
    /// \}

    /// \name State, setpoint, and error calculation
    /// \{
    /// Sets the internal state buffer
    void setState(const float x, const float DxDt, const float D2xDt2,
    				const float t);

    /// Sets the internal setpt buffer
    void setSetpt(const float x, const float DxDt, const float D2xDt2,
    				const float t);
    /// \}

    /// \name Execute controls
    /// \{
    /// Calculates the rate setpoint from the value PID
    void valuePID(const uint8_t useDerrDt);

    /// Calculates the U value from the rate PID
    void ratePID(const uint8_t useDerrDt);
    /// \}
};

#endif // DOF_HPP
