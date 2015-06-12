#ifndef DOF_HPP_
#define DOF_HPP_

/** \file dof.h
    \brief Declaration of the DOF structure and supporting functions

    Defines the DOF structure and declares the support functions allowing the
    structure to be used for implementing a controller for the quadrotor.

    \author Sajan Patel
    \version 3.0
    \date July 2014

    \see dof.c
*/
#include <stdint.h>

// Global constants for bitfields
#define DISC_RATE	0x01
#define DISC_ACCEL	0x02
#define WRAP_AROUND	0x04


// Global constants for max lengths of arrays in this class
// (I don't want vectors in these classes becuase of memory overhead)
#define NUM_GAINS		3
#define NUM_STATES		4
#define NUM_DERIVATIVES	3 // always NUM_STATES - 1

// Enum for gain indeces in data structures
enum PIDGainsEnum {KP, KI, KD};
enum DofStateEnum {VAL, RATE, ACCEL, TIME};

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
class Dof {
public:
    /// \name dof state values (all with timestamps)
    /// \{
    float state[NUM_STATES];         ///< [x, dx/dt, d^2x/dt^2, t] for this DOF
    float setpt[NUM_STATES];         ///< [x, dx/dt, d^2x/dt^2, t] desired for this DOF
    /// \}

	/// \name flags, options, and limits
	/// \{
	uint8_t flags; 				///< bit 0 for discrete rate, bit 1 for discrete accel, bit 2 for wraparound
	/// \}

    /// \name controller values
    /// \{
    float valueGains[NUM_GAINS];  		///< [Kp, Ki, Kd] for the value -> desired rate PID
    float rateGains[NUM_GAINS];   		///< [Kp, Ki, Kd] for the rate -> U value PID
    /// \}
	
	// Constructors
    Dof();
    Dof(const float x, const float DxDt, const float D2xDt2,
    	const float t, const float inertia, const float stateBound,
		const float rateSetLimit, const float UvalUpLimit,
		const float UvalLwLimit, const uint8_t flags);

    // Destructor
    ~Dof();

    /// \name Updates
    /// \name State, setpoint, and error calculation
    /// \{
    /// Sets the internal state buffer
    void updateState(const float x, const float DxDt, const float D2xDt2,
    			  const float t);

    /// Sets the i		dof_calc_ctrl_dt(&(qc->xyzh[i]));nternal setpt buffer
    void updateSetpt(const float x, const float DxDt, const float D2xDt2,
    			  const float t);
    
	/// Sets the PID gains
    void updateGains(const float valueGains[3], const float rateGains[3]);
	/// \}

    /// \name Execute controls
    /// \{
    /// Calculates the rate setpoint from the value PID
    void valuePID(const bool useDerrDt);

    /// Calculates the U value from the rate PID
    void ratePID(const bool useDerrDt);

    /// Sets the internal dt from time of last control loop, stores in ctrl_dt
    void calcCtrlDt();

    /// Sets the internal ctrl_error, prev_error, ctrl_derr_dt, and ctrl_integral
    void calcError(const unsigned int idx, const bool integrate);
    /// \}

    // The following functions return private member values
    float getUval() const; // return Uval

    float getVelocity() const; // returns setpt[RATE] since that's the desired velocity output

    //const float* getState(bool useCurr) const;
    //const float* getSetpt() const;

    float getInertia() const;
    float getBounds(bool useVal) const; 
    float getUvalLimit(bool useUpper) const;

    //uint8_t getFlags() const;
    //const float* getGains(bool useVal) const;

    float getCtrlDt() const;
    const float* getCtrlError(bool useCurr) const;
    const float* getCtrlDerrDt() const;
    const float* getCtrlIntegral() const;

private:
    /// \name dof state values (all with timestamps)
    /// \{
    //float state[NUM_STATES];         ///< [x, dx/dt, d^2x/dt^2, t] for this DOF
    float prevState[NUM_STATES];    ///< [x, dx/dt, d^2x/dt^2, t] of last time step
    //float setpt[NUM_STATES];         ///< [x, dx/dt, d^2x/dt^2, t] desired for this DOF
    float inertia;        ///< Mass or moment of inertia for the DOF
    float Uval;           ///< U value output for the DOF
    /// \}

	/// \name flags, options, and limits
	/// \{
	float stateBound;      	///< positive limit on value state, 0 for none
	float rateSetLimit;   	///< positive limit on rate setpoint, 0 for none
	float UvalUpLimit;		///< upper limit on U value, no limit if = to lw
	float UvalLwLimit;  		///< lower limit on U value, no limit if = to up
	//uint8_t flags; 				///< bit 0 for discrete rate, bit 1 for discrete accel, bit 2 for wraparound
	/// \}

    /// \name controller values
    /// \{
    //float valueGains[NUM_GAINS];  		///< [Kp, Ki, Kd] for the value -> desired rate PID
    //float rateGains[NUM_GAINS];   		///< [Kp, Ki, Kd] for the rate -> U value PID
    float ctrlDt;         		///< time since controller last ran
    float ctrlError[NUM_STATES];   		///< setpoint - state, with a timestamp
    float ctrlPrevError[NUM_STATES];		///< previous error values
    float ctrlDerrDt[NUM_DERIVATIVES];  		///< discrete derivative of the error [x, dx, d2x]
    float ctrlIntegral[NUM_DERIVATIVES];		///< integral of setpoint - state [x, dx, d2x]
    /// \}

    /// Calculates the discrete derivative of the state value (to get rate)
    void discreteDxDt();

    /// Calculates the discrete derivative of the state rate (to get accel)
    void discreteD2xDt2();
    

	/*** these were originally public, but I may not need them to be now.
	  * While I'm figuring this out, keep them here since they're used in the
	  * constructors.
	  */
	/// Sets the PID gains
    void initState(const float x, const float DxDt, const float D2xDt2,
    		       const float t, const float mass);

    void setInertia(const float _inertia);

    /// Initializes the PID output limits
	void setLimits(const float stateBound, const float rateSetLimit,
    			   const float UvalUpLimit, const float UvalLwLimit);

    /// Initializes the operational flags
    void setFlags(const uint8_t flags);
};

#endif // DOF_HPP