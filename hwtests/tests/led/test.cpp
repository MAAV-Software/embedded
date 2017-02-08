#include "TestFunction.hpp"
#include <stdint.h>
#include "driverlib/sysctl.h"
#include "LED.h"
#include "time_util.h"

void TestFunction::run()
{
	Config_LED();

	for(;;) {
		TurnOff_LED(BLUE_LED);                 //TurnOn_LED and TurnOff_LED do the opposite things
		SysCtlDelay(SYSCLOCK / 3 / 2);
		TurnOn_LED(BLUE_LED);
		SysCtlDelay(SYSCLOCK / 3 / 2);
	}
}
