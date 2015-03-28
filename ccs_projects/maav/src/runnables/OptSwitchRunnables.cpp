#include "runnables/OptSwitchRunnables.hpp"

SwitchUpdateRunnable::run(){
	switchesUpdate(sw);
}

SwitchUpdateRunnable::SwitchUpdateRunnable(SwitchData_t *switches)
{
	sw = switches;
}
