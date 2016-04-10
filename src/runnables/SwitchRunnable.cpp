#include "runnables/SwitchRunnable.hpp"
#include "Switch.hpp"
#include "ProgramState.hpp"

void SwitchRunnable::run()
{
	for(int i = 0; i < 3; ++i) _sw[i].update();
}

SwitchRunnable::SwitchRunnable(ProgramState *ps) : _sw(ps->sw) { }
