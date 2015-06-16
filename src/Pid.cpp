/**
 * PID class implmentation
 */
#include <stdint.h>
#include <cmath>
#include "Pid.hpp"

Pid::Pid()
{
	for (unsigned i = 0; i < NUM_PID_STATES; ++i)
	{
		state[i]     = 0;
		prevState[i] = 0;
		setpt[i]     = 0;
		error[i]     = 0;
		prevError[i] = 0;
	}
	
	for (unsigned i = 0; i < (NUM_PID_STATES - 1); ++i)
	{
		dErrDt[i]       = 0;
		errIntegral[i]  = 0;
		lowPassCoef[i]  = 0;
		lowPassState[i] = 0;
	}

	for (unsigned i = 0; i < NUM_PID_GAINS; ++i) gains[i] = 0;

	outUpLim   =  (float)HUGE_VAL;
	outLwLim   = -(float)HUGE_VAL;
	stateBound = 0;
	flags      = 0;	
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
		 const float lpCoeff[NUM_PID_STATES - 1])
{
	for (unsigned i = 0; i < NUM_PID_STATES; ++i)
	{
		state[i]     = initState[i];
		prevState[i] = 0;
		setpt[i]     = initSetpt[i];
		error[i]     = 0;
		prevError[i] = 0;
	}
	
	for (unsigned i = 0; i < (NUM_PID_STATES - 1); ++i)
	{
		dErrDt[i]       = 0;
		errIntegral[i]  = 0;
		lowPassCoef[i]  = lpCoeff[i];
		lowPassState[i] = 0;
	}

	for (unsigned i = 0; i < NUM_PID_GAINS; ++i) gains[i] = initGains[i];
	
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
		while (state[VAL] >   stateBound) state[VAL] -= 2.0f * stateBound;
		while (state[VAL] <= -stateBound) state[VAL] += 2.0f * stateBound;
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
		while (setpt[VAL] >   stateBound) setpt[VAL] -= 2.0f * stateBound;
		while (setpt[VAL] <= -stateBound) setpt[VAL] += 2.0f * stateBound;
	}
}

void Pid::setGains(const float kp, const float ki, const float kd)
{
	// Reset integral if integral gain changes
	if (gains[KI] != ki)
	{
		if (ki != 0.0) errIntegral[VAL] *= gains[KI] / ki;
		else		   errIntegral[VAL]  = 0.0;
	}
	// copy over gains
	gains[KP] = kp;
	gains[KI] = ki;
	gains[KD] = kd;
}

void Pid::calcDt()
{
	prevError[TIME] = error[TIME]; // record previous error time
	
	// update new error time based on whether the state of setpt was the most
	// recent item to be updated
	if (state[TIME] >= setpt[TIME]) error[TIME] = state[TIME];
	else 							error[TIME] = setpt[TIME];

	ctrlDt = error[TIME] - prevError[TIME];	// calc dt		
}	

void Pid::calcError(PIDStateEnum idx)
{
	// only calc if dt is NOT too small
	if (ctrlDt >= 0.001)
	{
		// set previous and current controller errors
		prevError[idx] = error[idx];
		error[idx] = setpt[idx] - state[idx];

		// wraparound error for the main value
		if ((idx == VAL) && (flags & WRAP_AROUND_MASK) 
				&& (stateBound > 0.000001))
		{
			if (error[idx] >  stateBound) error[idx] -= 2.0f * stateBound;
			if (error[idx] < -stateBound) error[idx] += 2.0f * stateBound;
		}

		// Calculate discrete derivative of error
		dErrDt[idx] = (error[idx] - prevError[idx]) / ctrlDt;
		// TODO Determine if lowpass filter is necessary

		// caclualte integral error
		errIntegral[idx] += 0.5f * ctrlDt * (error[idx] + prevError[idx]);
	}
}

void Pid::run()
{
	// first get the ctrl dt
	calcDt();

	// next calc error based on which flags are set
	calcError(VAL);
	if (flags & DERR_DT_MASK) calcError(DERIV);

	// Finally calculate the output of the PID algorithm (PI gains first)
	ctrlOutput = (gains[KP] * error[VAL]) + (gains[KI] * errIntegral[VAL]);
	
	// choose state deriv or dErrDt as D gain based on flag
	if (flags & DERR_DT_MASK) ctrlOutput += gains[KD] * dErrDt[VAL];
	else	 				  ctrlOutput += gains[KD] * -state[DERIV];

	// saturate at output limits
	if (ctrlOutput > outUpLim) 		ctrlOutput = outUpLim;
	else if (ctrlOutput < outLwLim) ctrlOutput = outLwLim;
}

float Pid::getOutput() const
{
	return ctrlOutput;
}

void Pid::prepareLog()
{
//TODO Implement this function after we figure out what the log format will be.
}

void Pid::discreteDeriv()
{
	float dt = state[TIME] - prevState[TIME]; // dt for deriv calculation
	
	// only calculate if dt is NOT too small
	if (dt > 0.001)
	{
		float diff = state[VAL] - prevState[VAL];
		
		// wraparound if necessary
		if ((flags & WRAP_AROUND_MASK) && (stateBound > 0.000001))
		{
			while (diff >  stateBound) diff -= 2.0f * stateBound;
			while (diff < -stateBound) diff += 2.0f * stateBound;
		}
		
		state[DERIV] = diff / dt;
		// TODO Determine if lowpass filter is necessary
	}
}

