#include "TestFunction.hpp"
#include "LED.h"
#include "time_util.h"

void TestFunction::run()
{
	Config_LED();
	for(;;)
	{
		Toggle_LED(RED_LED, SYSCLOCK / 3 / 2);
	}
}
