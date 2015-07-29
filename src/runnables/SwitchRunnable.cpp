#include "runnables/SwitchRunnable.hpp"
#include "switch.h"
#include "ProgramState.hpp"

void SwitchRunnable::run()
{
	switchesUpdate(_sw);
}

SwitchRunnable::SwitchRunnable(ProgramState *ps) : _sw(ps->sw)
{ 
	switchesInit(_sw);
}
