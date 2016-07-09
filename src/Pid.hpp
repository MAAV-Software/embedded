/**
 * @brief Header for PID class.
 * @author Sajan Patel (sajanptl@umich.edu)
 */

#ifndef PID_HPP_
#define PID_HPP_

#include <stdint.h>
#include "CtrlLogs.hpp"
#include "LowPass.hpp"

/**
 * @brief Global constants for max lengths of arrays in this class
 */
#define NUM_PID_GAINS	3	///< 3 PID gains
#define NUM_PID_STATES	3	///< States are state value, derivative, and time

/**
 * @brief Global constants for bitfields in the flags varaible
 */
extern const uint8_t DISC_DERIV_MASK; ///< use a discrete derivate of state value for state deriv
extern const uint8_t WRAP_AROUND_MASK; ///< keep track of wraping arround the state bounds at the limits
extern const uint8_t DERR_DT_MASK; ///< enable use of error derivative for D gain output
extern const uint8_t STATE_LOWPASS_MASK; ///< enable use of lowpass filter on discrete state derivatives
extern const uint8_t DERR_LOWPASS_MASK; ///< enable use of lowpass filter on dError/dt

/**
 * @brief Enum for indexing gains arrays
 */
enum PIDGainsEnum {KP, KI, KD}; 

/**
 * @brief Enum for indexing state arrays
 */
enum PIDStateEnum {VAL, DERIV, TIME};

/**
 * @brief state-space PID class 
 * @detail PID class that executes discrete PID control on a continuous-time state-space model.
 */
class Pid
{
public:
	/**
     * @brief  Default constructor
     */
	Pid();
	
	/**
	 * @brief Custom constructor that initializes the state, setpt, flags, gains, limits, and lowpass coefficients
	 * @param initState     initial state [val, deriv, time]
     * @param intiSetpt     initial setpoint [val, unused, time]
     * @param initFlags     flags (obtained by ORing various masks) for configuring this PID instance
     * @param initGains     initial gains [Kp, Ki, Kd]
     * @param initStateBound    saturation bound on the state value
     * @param initOutUpLim      saturation upper bound on the output
     * @param initOutLwLim      staturation lower bound on the output
     * @param stateLpCoeff      state derivative low-pass filter coefficient
     * @param errorLpCoeff      error derivative low-pass filter coefficient
     */
	Pid(const float initState[NUM_PID_STATES], 
		const float initSetpt[NUM_PID_STATES],
		const uint8_t initFlags, 
		const float initGains[NUM_PID_GAINS], 
	    const float initStateBound, 
		const float initOutUpLim,
		const float initOutLwLim, 
		const float stateLpCoeff,
		const float errorLpCoeff);
	
	/**
	 * @brief Sets the state (feedback) for the PID algorithm
     * @param val state value (in units for the state this PID is used for)
     * @param time timestamp (in seconds) for this state feedback
	 */
	void setState(const float val, const float deriv, const float time);
	
	/**
	 * @breif Sets the setpoint for the PID algorithm
     * @param val setpoint value (in units for the state this PID is used for)
     * @param time timestamp (in seconds) of the setpoint
	 */
	void setSetpt(const float val, const float time);
	
	/**
	 * @brief Sets the gains for the PID algorithm
     * @param kp Kp (proportional) gain
     * @param ki Ki (integral) gain
     * @param kd Kd (derivative) gain
	 */
	void setGains(const float kp, const float ki, const float kd);
	
	/**
	 * @breif runs the PID algorithm
	 */
	void run();

	/**
	 * @brief return the output of the PID
	 */
	float getOutput() const;

	/**
	 * @brief function for preparing the log data for this PID class
     * @param plog PidLog struct to fill with the log data
	 */
	void prepareLog(PidLog &plog); 
	
private:
	uint8_t flags;						///< flags for PID ctrl options

	float state[NUM_PID_STATES];		///< [val, deriv, time] for the current continuous state variable
	float prevState[NUM_PID_STATES];	///< previous [val, deriv, time]
										
	float setpt[NUM_PID_STATES];		///< [val, deriv, time] setpoint for the continuous state
										
	float error;						///< setpt[val] - state[val], the error for the PID algorithm
	float prevError;					///< previous state value error
	float errorTime;					///< timestamp for error
	float prevErrorTime;				///< timestamp for previous error									
	float ctrlDt;						///< time difference between current and last PID algorithm executions
	float dErrDt; 						///< derivative of the state value error
	float errIntegral;					///< integral of the state value error
										
	float ctrlOutput;					///< final output of the PID controller on this state, which is 
										///< viewed as a derivative of the state that is being controlled
										
	float gains[NUM_PID_GAINS];			///< [KP, KI, KD] array of PID gains
										
	float stateBound; 					///< Bound on the state VALUE (mainly for rotational quantities)
	float outUpLim;						///< Upper limit on the output for saturation
	float outLwLim;						///< Lower limit on the output for saturation
										
	LowPass stateLp; 					///< LowPass filter for state derivative calculations
	LowPass errorLp; 					///< LowPass filter for error derivative calculations

	// Calculates a discrete derivative of the state value, overwriting the 
    // state[DERIV] position with it.	
	void discreteDeriv();
	
	// Calculates the time difference for the PID algorithm
	float calcDt(); 

	// Calculates the error between setpoint and state for the PID algorithm
	void calcError();
};

#endif /* PID_HPP_ */
