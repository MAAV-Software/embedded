#ifndef SWITCHRUNNABLE_HPP_
#define SWITCHRUNNABLE_HPP_

#include "Runnable.hpp"
#include "switch.h"
#include "ProgramState.hpp"

class SwitchRunnable : public Runnable
{
public:
	SwitchRunnable(ProgramState *ps);
	void run();

private:
	SwitchData_t *_sw;
};

#endif
