/**
 * PID class implmentation
 */
#include <stdint.h>
#include <cmath>
#include "Pid.hpp"
#include "LowPass.hpp"
#include "CtrlLogs.hpp"

using namespace std;

// Global constants for bitfields in the flags varaible
const uint8_t DISC_DERIV_MASK    = 0x01;
const uint8_t WRAP_AROUND_MASK   = 0x02;
const uint8_t DERR_DT_MASK	     = 0x04;
const uint8_t STATE_LOWPASS_MASK = 0x08;
const uint8_t DERR_LOWPASS_MASK  = 0x10;

Pid::Pid()
{
	for (unsigned i = 0; i < NUM_PID_STATES; ++i)
	{
		state[i]     = 0;
		prevState[i] = 0;
		setpt[i]     = 0;
	}
		
	error         = 0;
	prevError     = 0;
	errorTime     = 0;
	prevErrorTime = 0;
	dErrDt        = 0;
	errIntegral   = 0;	

	for (unsigned i = 0; i < NUM_PID_GAINS; ++i) gains[i] = 0;
	
	stateLp = LowPass();
	errorLp = LowPass();

	outUpLim   =  (float)HUGE_VAL;
	outLwLim   = -(float)HUGE_VAL;
	stateBound = 0;
	flags      = DISC_DERIV_MASK;	
	ctrlDt     = 0;
	ctrlOutput = 0;
}
	
Pid::Pid(const float initState[NUM_PID_STATES], 
	     const float initSetpt[NUM_PID_STATES],
		 const uint8_t initFlags,
		 const float initGains[NUM_PID_GAINS], 
	     const float initStateBound,
		 const float initOutUpLim,
		 const float initOutLwLim,
		 const float stateLpCoeff,
		 const float errorLpCoeff)
{
	for (unsigned i = 0; i < NUM_PID_STATES; ++i)
	{
		state[i]     = initState[i];
		prevState[i] = 0;
		setpt[i]     = initSetpt[i];
	}
	
	setpt[DERIV]  = 0; // enforce 0 derivative setpt
	error         = 0;
	prevError     = 0;
	errorTime     = 0;
	prevErrorTime = 0;
	dErrDt        = 0;	
	errIntegral   = 0;

	for (unsigned i = 0; i < NUM_PID_GAINS; ++i) gains[i] = initGains[i];
	
	stateLp = LowPass(stateLpCoeff);
	errorLp = LowPass(errorLpCoeff);

	stateBound = initStateBound;
	outUpLim   = initOutUpLim;
	outLwLim   = initOutLwLim;
	flags      = initFlags;	
	ctrlDt     = 0;
	ctrlOutput = 0;
}
	
void Pid::setState(const float val, const float deriv, const float time)
{
	// assign state feedback
	state[VAL]   = val;
	state[DERIV] = deriv;
	state[TIME]  = time;
	
	// wrap around stateBound if necessary (small tolerance for float precision)
	if ((flags & WRAP_AROUND_MASK) && (stateBound > 0.000001))
	{
		while (state[VAL] >=  stateBound) state[VAL] -= 2.0f * stateBound;
		while (state[VAL] <  -stateBound) state[VAL] += 2.0f * stateBound;
	}

	// calculate discrete derivatives for state if necessary
	if (flags & DISC_DERIV_MASK) discreteDeriv();
}

void Pid::setSetpt(const float val, const float time)
{
	setpt[VAL]   = val;
	setpt[DERIV] = 0;
	setpt[TIME]  = time;
	
	// wrap around stateBound if necessary (small tolerance for float precision)
	if ((flags & WRAP_AROUND_MASK) && (stateBound > 0.000001))
	{
		while (setpt[VAL] >=  stateBound) setpt[VAL] -= 2.0f * stateBound;
		while (setpt[VAL] <  -stateBound) setpt[VAL] += 2.0f * stateBound;
	}
}

void Pid::setGains(const float kp, const float ki, const float kd)
{
	// Reset integral if integral gain changes
	if (gains[KI] != ki)
	{
		if (ki != 0.0) errIntegral *= gains[KI] / ki;
		else		   errIntegral  = 0.0;
	}
	// copy over gains
	gains[KP] = kp;
	gains[KI] = ki;
	gains[KD] = kd;
}

float Pid::calcDt()
{
	prevErrorTime = errorTime; // record previous error time
	
	// update new error time based on whether the state of setpt was the most
	// recent item to be updated
	if (state[TIME] >= setpt[TIME]) errorTime = state[TIME];
	else 							errorTime = setpt[TIME];
	
	return errorTime - prevErrorTime; // calc dt		
}	

void Pid::calcError()
{
	ctrlDt = calcDt(); // get the controller's dt
	
	// only calc if dt is NOT too small
	if (ctrlDt >= 0.001f)
	{
		// set previous and current controller errors
		prevError = error;
		error = setpt[VAL] - state[VAL];

		// wraparound error for the main value
		if ((flags & WRAP_AROUND_MASK) && (stateBound > 0.000001f))
		{
			while (error >=  stateBound) error -= 2.0f * stateBound;
			while (error < -stateBound) error += 2.0f * stateBound;
		}
		
		// Calculate discrete derivative of error
		dErrDt = (error - prevError) / ctrlDt;
		if (flags & DERR_LOWPASS_MASK) // lowpass the derivative if needed
		{
			errorLp.run(dErrDt);
			dErrDt = errorLp.getState();
		}
		
		// caclualte integral error
		errIntegral += 0.5f * ctrlDt * (error + prevError);
	}
}

void Pid::run()
{
	calcError(); // calculate controller error
	
	// finally calculate the output of the PID algorithm (PI gains first)
	ctrlOutput = (gains[KP] * error) + (gains[KI] * errIntegral);
	
	// choose state deriv or dErrDt as D gain based on flag
	if (flags & DERR_DT_MASK) ctrlOutput += gains[KD] * dErrDt;
	else	 				  ctrlOutput += gains[KD] * -state[DERIV];

	// saturate at output limits
	if (ctrlOutput > outUpLim) 		ctrlOutput = outUpLim;
	else if (ctrlOutput < outLwLim) ctrlOutput = outLwLim;
}

float Pid::getOutput() const
{
	return ctrlOutput;
}

void Pid::prepareLog(PidLog &plog)
{
	plog.setpt = setpt[VAL];
	plog.kp    = gains[KP];
	plog.ki    = gains[KI];
	plog.kd	   = gains[KD];
	plog.flags = flags;
}

void Pid::discreteDeriv()
{
	// only calculate if dt between state and prevState is not too small
	if ((state[TIME] - prevState[TIME]) > 0.001f)
	{
		float diff = state[VAL] - prevState[VAL];
		
		// wraparound if necessary
		if ((flags & WRAP_AROUND_MASK) && (stateBound > 0.000001))
		{
			while (diff >=  stateBound) diff -= 2.0f * stateBound;
			while (diff <  -stateBound) diff += 2.0f * stateBound;
		}
		
		// calculate derivative
		state[DERIV] = diff / (state[TIME] - prevState[TIME]);
		if (flags & STATE_LOWPASS_MASK) // use LowPass if needed
		{
			stateLp.run(state[DERIV]);
			state[DERIV] = stateLp.getState();
		}
	}
}

