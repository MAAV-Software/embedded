#include "runnables/OptSwitchRunnables.hpp"

void SwitchUpdateRunnable::run(){
	switchesUpdate(_sw);
}

SwitchUpdateRunnable::SwitchUpdateRunnable(SwitchData_t *switches)
	: _sw(switches) { }

SwitchUpdateRunnable::~SwitchUpdateRunnable() {}
