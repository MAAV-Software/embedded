#ifndef CTRLRUNNABLE_HPP_
#define CTRLRUNNABLE_HPP_

#include "../ProgramState.hpp"
#include "Runnable.hpp"

class CtrlRunnable : public Runnable
{
public:
	CtrlRunnable(ProgramState *pState);
	void run();

private:
	ProgramState *ps;
	VehicleLog vlog;
	PidLog plogs[NUM_DOFS][2];
};

#endif /* CtrlRunnable.hpp */
