#ifndef PID_HPP_
#define PID_HPP_

/**
 * PID class that executes discrete PID control on a continuous-time model.
 * 
 */
#include<stdint.h>

// Global constants for bitfields in the flags varaible
#define DISC_DERIV_MASK		0x01 // use a discrete derivate of state value for state deriv
#define WRAP_AROUND_MASK	0x02 // keep track of wraping arround the state bounds at the limits
#define DERR_DT_MASK		0x04 // enable use of error derivative for D gain output

// Global constants for max lengths of arrays in this class
#define NUM_GAINS	3	// 3 PID gains
#define NUM_STATES	3	// States are state value, derivative, and time

// Enums for gains and states
enum PIDGainsEnum {KP, KI, KD}; 
typedef enum _PIDStateEnum {VAL, DERIV, TIME} PIDStateEnum;

class Pid
{
public:
	uint8_t flags; // flags for different options to enable on the controller

	// Default constructor
	Pid();
	
	// Custom constructor that initializes the state, setpt, flags, gains, limits, and lowpass coefficients
	Pid(const float initState[NUM_STATES], const float initSetpt[NUM_STATES],
		const uint8_t initFlags, const float initGains[NUM_GAINS], 
	    const float initStateBound, const float initOutUpLim,
		const float initOutLwLim, const float lpCoeff[NUM_STATES - 1]);
	
	// Sets the state (feedback) for the PID algorithm
	void setState(const float val, const float deriv, const float time);
	
	// Sets the setpoint for the PID algorithm
	void setSetpt(const float val, const float time);
	
	// Sets the gains for the PID algorithm
	void setGains(const float kp, const float ki, const float kd);
	
	// runs the PID algorithm
	void run();

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
	
	float stateBound; 					// Bound on the state VALUE (mainly for rotational quantities)
	float outUpLim;						// Limits on the output for saturation
	float outLwLim;

	// Calculates a discrete derivative of the state value, overwriting the 
    // state[DERIV] position with it.	
	void discreteDeriv();
	
	// Calculates the time difference for the PID algorithm
	void calcDt(); 

	// Calculates the error between setpoint and state for the PID algorithm
	void calcError(PIDStateEnum idx);
};

#endif /* PID_HPP_ */
