/**
 * PID class implmentation
 */
#include<stdint.h>
#include "PID.hpp"


PID::PID()
{
	for (unsigned i = 0; i < NUM_STATES; ++i)
	{
		state[i]     = 0;
		prevState[i] = 0;
		setpt[i]     = 0;
		error[i]     = 0;
		prevError[i] = 0;
	}
	
	for (unsigned i = 0; i < (NUM_STATES - 1); ++i)
	{
		dErrDt[i]       = 0;
		errIntegral     = 0;
		lowPassCoef[i]  = 0;
		lowPassState[i] = 0;
		stateUpLim[i]   = 0;
		stateLwLim[i]   = 0;
	}

	for (unsigned i = 0; i < NUM_GAINS; ++i) gains[i] = 0;

	flags = 0;	
	ctrlDt = 0;
	ctrlOutput = 0;
}
	
	// Custom constructor that initializes the state, setpt, flags, gains, limits, and lowpass coefficients
PID::PID(const float initState[NUM_STATES], 
	     const float initSetpt[NUM_STATES],
		 const uint8_t initFlags,
		 const float initGains[NUM_GAINS], 
	     const float upperLimits[NUM_STATES - 1], 
		 const float lowerLimits[NUM_STATES - 1],
		 const float lpCoeff[NUM_STATES - 1])
{







}
	
	// Sets the state (feedback) for the PID algorithm
	void setState(const float val, const float deriv, const float time);
	
	// Sets the setpoint for the PID algorithm
	void setSetpt(const float val, const float deriv, const float time);
	
	// Sets the gains for the PID algorithm
	void setGains(const float kp, const float ki, const float kd);

	// Calculates the time difference for the PID algorithm
	void calcDt(); 

	// Calculates the error between setpoint and state for the PID algorithm
	void calcError();
	
	// runs the PID algorithm
	void runPID();

	// return the output of the PID
	float getOutput() const;

	// function for preparing the log data for this PID class
	void prepareLog(); 
	
	// Calculates a discrete derivative of the state value, overwriting the 
    // state[DERIV] position with it.	
	void discreteDeriv();

