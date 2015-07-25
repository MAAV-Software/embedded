/** Dof.cpp
  Implementation of the DOF (degree of freedom) class.
*/
#include <stdint.h>
#include <cmath>
#include "Dof.hpp"
#include "Pid.hpp"
#include "CtrlLogs.hpp"

using namespace std;

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
		 const float valStateLpCoeff,
		 const float valErrorLpCoeff,
		 const float rateStateLpCoeff,
		 const float rateErrorLpCoeff)
{
	float valState[NUM_PID_STATES]  = {state[DOF_VAL], state[DOF_RATE], 
								       state[DOF_TIME]};
	float rateState[NUM_PID_STATES] = {state[DOF_RATE], state[DOF_ACCEL], 
								   	   state[DOF_TIME]};
	float valSetpt[NUM_PID_STATES]  = {setpt[DOF_VAL], 0, setpt[DOF_TIME]};
	float rateSetpt[NUM_PID_STATES] = {setpt[DOF_RATE], 0, setpt[DOF_TIME]};
	
	valuePid = Pid(valState, valSetpt, valueFlags, valueGains, stateBound, 
				   rateUpLim, rateLwLim, valStateLpCoeff, valErrorLpCoeff);
	ratePid  = Pid(rateState, rateSetpt, rateFlags, rateGains, 0, 
				   accelUpLim, accelLwLim, rateStateLpCoeff, rateErrorLpCoeff);
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
void Dof::setSetpt(const float setpt[NUM_DOF_STATES], bool rateOnly)
{
	// Only give rate setpts when using rate PIDs only
	if (rateOnly) 
	{
		ratePid.setSetpt(setpt[DOF_RATE], setpt[DOF_TIME]);
		rate = setpt[DOF_RATE]; // log rate for later use
	}
	else
		valuePid.setSetpt(setpt[DOF_VAL], setpt[DOF_TIME]);
	
	// log setpt time for later use	
	setptTime = setpt[DOF_TIME];
}
	
// Sets the gains for the PID algorithm
void Dof::setGains(const float valueGains[NUM_PID_GAINS], 
				   const float rateGains[NUM_PID_GAINS])
{
	valuePid.setGains(valueGains[KP], valueGains[KI], valueGains[KD]);
	ratePid.setGains(rateGains[KP], rateGains[KI], rateGains[KD]);
}
	
// runs the PID algorithm
void Dof::run(bool rateOnly)
{
	if (!rateOnly) // only run value PIDs when not solely running rate PIDs
	{
		valuePid.run();
		rate = valuePid.getOutput();
		ratePid.setSetpt(rate, setptTime);
	}
	
	// Always run Rate PID (setpt either got set above or in Dof::setSetpt())
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
	
// function for preparing the log data for this Dof class
void Dof::prepareLog(PidLog plogs[2])
{
	// grab pid logs
	valuePid.prepareLog(plogs[0]);
	ratePid.prepareLog(plogs[1]);
}	


// End of File
