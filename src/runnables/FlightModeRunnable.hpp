#ifndef FLIGHTMODERUNNABLE_HPP_
#define FLIGHTMODERUNNABLE_HPP_

#include "ProgramState.hpp"
#include "runnables/Runnable.hpp"

class FlightModeRunnable : public Runnable 
{
public:
	FlightModeRunnable(ProgramState* pState);
	void run();

private:
	ProgramState *state;
};

#endif /* FLIGHTMODERUNNABLE_HPP_ */
