#ifndef SWITCHRUNNABLE_HPP_
#define SWITCHRUNNABLE_HPP_

#include "Runnable.hpp"
#include "Switch.hpp"
#include "ProgramState.hpp"

class SwitchRunnable : public Runnable
{
public:
	SwitchRunnable(ProgramState *ps);
	void run();

private:
	ThreeSwitch *_sw;
};

#endif
