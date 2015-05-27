#ifndef PID_HPP_
#define PID_HPP_

/**
 * PID class that executes discrete PID control on a continuous-time model.
 * 
 */
#include<stdint.h>

// Global constants for bitfields in the flags varaible
#define USE_DISC_DERIV	0x01 // use a discrete derivate of state value for state deriv
#define WRAP_AROUND		0x02 // keep track of wraping arround the state bounds at the limits
#define USE_INTEGRAL	0x04 // enables use of I gain output and error integration
#define USE_DERR_DT		0x08 // enable use of error derivative for D gain output
#define CALC_DERIV_ERR	0x10 // enables error calculation on the state's derivative
							 // (useful when running PID on velocities as your states)
// Global constants for max lengths of arrays in this class
#define NUM_GAINS	3	// 3 PID gains
#define NUM_STATES	3	// States are state value, derivative, and time

// Enums for gains and states
enum PIDGainsEnum {KP, KI, KD}; 
enum PIDStateEnum {VAL, DERIV, TIME};

class PID
{
public:
	uint8_t flags; // flags for different options to enable on the controller

	// Default constructor
	PID();
	
	// Custom constructor that initializes the state, setpt, flags, gains, limits, and lowpass coefficients
	PID(const float stateVal, const float stateDeriv, const float stateTime, 
	    const float setptVal, const float setptDeriv, const float setptTime,
		const uint8_t flags, const float kp, const float ki, const float kd, 
const float upperLimits[2], const float lowerLimits[2],
		const float lpCoeff[2]);
	
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
	
private:
	float state[NUM_STATES];			// [val, deriv, time] for the current continuous state variable
	float prevState[NUM_STATES];		// previous [val, deriv, time]
	
	float setpt[NUM_STATES];			// [val, deriv, time] setpoint for the continuous state
	
	float error[NUM_STATES];			// setpt - state, the error for the PID algorithm
	float prevError[NUM_STATES];		// previous error
	
	float ctrlDt;						// time difference between current and last PID algorithm executions
	float dErrDt[NUM_STATES - 1]; 		// derivative of the error (doesn't need time, so 1 less array item)
	float errIntegral[NUM_STATES - 1];	// integral of the error (doesn't need time, so 1 less array item)
	
	float ctrlOutput;					// final output of the PID controller on this state, which is 
										// viewed as a derivative of the state that is being controlled
	
	float gains[NUM_GAINS];				// [KP, KI, KD] array of PID gains
	
	float lowPassCoef[NUM_STATES - 1];  // lowpass filter coeffs don't need time
	float lowPassState[NUM_STATES - 1]; // lowpass filter state
	
	float stateUpLim[NUM_STATES - 1];	// limits of the state and derivate values
	float stateLwLim[NUM_STATES - 1];

	// Calculates a discrete derivative of the state value, overwriting the 
    // state[DERIV] position with it.	
	void discreteDeriv();
};

#endif /* PID_HPP_ */
