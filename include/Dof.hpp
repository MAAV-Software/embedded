#ifndef DOF_HPP_
#define DOF_HPP_

/** \file dof.h
*/
#include <stdint.h>
#include "Pid.hpp"

#define NUM_DOF_STATES 4 // val, rate, accel, and time

typedef enum _DofStatesEnum {DOF_VAL, DOF_RATE, DOF_ACCEL, DOF_TIME} DofStatesEnum;

class Dof
{
public:
	// Constructors
	Dof();	
	
	Dof(const float state[NUM_DOF_STATES], 
		const float setpt[NUM_DOF_STATES],
	 	const float valueGains[NUM_PID_GAINS],
		const float rateGains[NUM_PID_GAINS],
		const uint8_t valueFlags,
		const uint8_t rateFlags,
		const float Inertia,
		const float stateBound,
		const float rateUpLim,
		const float rateLwLim, 
		const float accelUpLim,
		const float accelLwLim,
		const float valueLpCoeff[NUM_PID_STATES - 1],
		const float rateLpCoeff[NUM_PID_STATES - 1]);
	
	// Sets the state (feedback) for the PID algorithm
	void setState(const float state[NUM_DOF_STATES]);
	
	// Sets the setpoint for the PID algorithm
	void setSetpt(const float setpt[NUM_DOF_STATES], bool rateOnly);
	
	// Sets the gains for the PID algorithm
	void setGains(const float valueGains[NUM_PID_GAINS], 
				  const float rateGains[NUM_PID_GAINS]);
	
	// runs the PID algorithm
	void run(bool rateOnly);

	// return the velocity output of the value PID
	float getRate() const;

	// return the force output of the rate PID
	float getUval() const;
	
	// function for preparing the log data for this PID class
	void prepareLog(); 
	
private:
	Pid valuePid;	// value to rate PID for this degree of freedom
	Pid ratePid;	// rate to accel PID for this degree of freedom
	float Uval;		// force/moment controller output for this degree of freedom
	float rate;		// rate controller output for this degree of freedom
	float inertia;	// mass/moment of inertia influencing this degree of freedom
	float setptTime; // time of Dof Setpt stored for ratePid calculations
};

#endif // DOF_HPP
