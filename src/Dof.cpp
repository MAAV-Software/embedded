/** \file dof.c
    \brief Implimentation of the DOF structure support functions

    Implements the functionality and support of the DOF structure allowing the
    structure to be used for implementing a controller for the quadrotor.

    \author Mathew Karashin, Sajan Patel
    \version 3.0
    \date July 2014

    \see dof.h
*/
#include <stdint.h>
#include <math.h>
#include "Dof.hpp"
#include "Pid.hpp"

Dof::Dof()
{
	valuePid = Pid();
	ratePid = Pid();
	Uval = 0;
	rate = 0;
	inertia = 1;
	setptTime = 0;
}
	
Dof::Dof(const float state[NUM_DOF_STATES], 
		 const float setpt[NUM_DOF_STATES],
	 	 const float valueGains[NUM_GAINS],
		 const float rateGains[NUM_GAINS],
		 const uint8_t valueFlags,
		 const uint8_t rateFlags,
		 const float Inertia,
		 const float stateBound,
		 const float rateUpLim,
		 const float rateLwLim, 
		 const float accelUpLim,
		 const float accelLwLim,
		 const float valueLpCoeff[NUM_STATES - 1],
		 const float rateLpCoeff[NUM_STATES - 1])
{
	float valState[NUM_STATES]  = {state[DOF_VAL], state[DOF_RATE], 
								   state[DOF_TIME]};
	float rateState[NUM_STATES] = {state[DOF_RATE], state[DOF_ACCEL], 
								   state[DOF_TIME]};
	float valSetpt[NUM_STATES]  = {setpt[DOF_VAL], 0, setpt[DOF_TIME]};
	float rateSetpt[NUM_STATES] = {setpt[DOF_RATE], 0, setpt[DOF_TIME]};
	valuePid = Pid(valState, valSetpt, valueFlags, valueGains, stateBound, 
				   rateUpLim, rateLwLim, valueLpCoeff);
	ratePid  = Pid(rateState, rateSetpt, rateFlags, rateGains, 0, 
				   accelUpLim, accelLwLim, rateLpCoeff);
	Uval = 0;
	rate = 0;
	inertia = Inertia;
	setptTime = setpt[DOF_TIME];
}
	
// Sets the state (feedback) for the PID algorithm
void Dof::setState(const float state[NUM_DOF_STATES])
{
	valuePid.setState(state[DOF_VAL], state[DOF_RATE], state[DOF_TIME]);
	ratePid.setState(state[DOF_RATE], state[DOF_ACCEL], state[DOF_TIME]);
}
	
// Sets the setpoint for the PID algorithm
void Dof::setSetpt(const float setpt[NUM_SETPTS], bool isYaw, FlightMode_t mode)
{
	//TODO Need to make FlightMode_t and isYaw inputs consistent with actual things
	if ((mode == ASSISTED) && !isYaw)
		ratePid.setSetpt(setpt[DOF_RATE], setpt[DOF_TIME]);
	else
		valuePid.setSetpt(setpt[DOF_VAL], setpt[DOF_TIME]);

	setptTime = setpt[DOF_TIME];
}
	
// Sets the gains for the PID algorithm
void Dof::setGains(const float valueGains[NUM_GAINS], 
				   const float rateGains[NUM_GAINS])
{
	valuePid.setGains(valueGains[KP], valueGains[KI], valueGains[KD]);
	raetPid.setGains(rateGains[KP], rateGains[KI], rateGains[KD]);
}
	
// runs the PID algorithm
void Dof::run(bool isYaw, FlightMode_t mode)
{
	if ((mode == AUTONOMOUS) || ((mode == ASSISTED) && isYaw))
		valuePid.run();

	rate = valuePid.getOutput();
	ratePid.setSetpt(rate, setptTime);
	ratePid.run();
	Uval = inertia * ratePid.getOutput();
}

// return the velocity output of the value PID
float Dof::getRate() const
{
	return rate;
}

// return the force output of the rate PID
float Dof::getUval() const
{
	return Uval;
}
	
// function for preparing the log data for this PID class
void Dof::prepareLog()
{
//TODO Implement this function
}	


// End of File
