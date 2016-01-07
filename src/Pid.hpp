#ifndef PID_HPP_
#define PID_HPP_

/**
 * PID class that executes discrete PID control on a continuous-time model.
 */
#include <stdint.h>
#include "CtrlLogs.hpp"
#include "LowPass.hpp"

// Global constants for max lengths of arrays in this class
#define NUM_PID_GAINS	3	// 3 PID gains
#define NUM_PID_STATES	3	// States are state value, derivative, and time

// Global constants for bitfields in the flags varaible
extern const uint8_t DISC_DERIV_MASK; // use a discrete derivate of state value for state deriv
extern const uint8_t WRAP_AROUND_MASK; // keep track of wraping arround the state bounds at the limits
extern const uint8_t DERR_DT_MASK; // enable use of error derivative for D gain output
extern const uint8_t STATE_LOWPASS_MASK; // enable use of lowpass filter on discrete state derivatives
extern const uint8_t DERR_LOWPASS_MASK; // enable use of lowpass filter on dError/dt

// Enums for gains and states
enum PIDGainsEnum {KP, KI, KD}; 
enum PIDStateEnum {VAL, DERIV, TIME};

class Pid
{
public:
	// Default constructor
	Pid();
	
	// Custom constructor that initializes the state, setpt, flags, gains, limits, and lowpass coefficients
	Pid(const float initState[NUM_PID_STATES], 
		const float initSetpt[NUM_PID_STATES],
		const uint8_t initFlags, 
		const float initGains[NUM_PID_GAINS], 
	    const float initStateBound, 
		const float initOutUpLim,
		const float initOutLwLim, 
		const float stateLpCoeff,
		const float errorLpCoeff);
	
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
	void prepareLog(PidLog &plog); 
	
private:
	uint8_t flags;						// flags for PID ctrl options

	float state[NUM_PID_STATES];		// [val, deriv, time] for the current continuous state variable
	float prevState[NUM_PID_STATES];	// previous [val, deriv, time]
										
	float setpt[NUM_PID_STATES];		// [val, deriv, time] setpoint for the continuous state
										
	float error;						// setpt[val] - state[val], the error for the PID algorithm
	float prevError;					// previous state value error
	float errorTime;					// timestamp for error
	float prevErrorTime;				// timestamp for previous error									
	float ctrlDt;						// time difference between current and last PID algorithm executions
	float dErrDt; 						// derivative of the state value error
	float errIntegral;					// integral of the state value error
										
	float ctrlOutput;					// final output of the PID controller on this state, which is 
										// viewed as a derivative of the state that is being controlled
										
	float gains[NUM_PID_GAINS];			// [KP, KI, KD] array of PID gains
										
	float stateBound; 					// Bound on the state VALUE (mainly for rotational quantities)
	float outUpLim;						// Upper limit on the output for saturation
	float outLwLim;						// Lower limit on the output for saturation
										
	LowPass stateLp; 					// LowPass filter for state derivative calculations
	LowPass errorLp; 					// LowPass filter for error derivative calculations

	// Calculates a discrete derivative of the state value, overwriting the 
    // state[DERIV] position with it.	
	void discreteDeriv();
	
	// Calculates the time difference for the PID algorithm
	float calcDt(); 

	// Calculates the error between setpoint and state for the PID algorithm
	void calcError();
};

#endif /* PID_HPP_ */
