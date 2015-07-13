#include "runnables/FlightModeRunnable.hpp"
#include "FlightMode.hpp"
#include "FlightModeHw.hpp"
#include "LED.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"


FlightModeRunnable::FlightModeRunnable(ProgramState *pState) : state(pState) {}

void FlightModeRunnable::run() 
{
	state->mode = flightModeGet();
	switch(state->mode)
	{
		case ASSISTED:
			MAP_GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED, RED_LED);
			break;
		case AUTONOMOUS:
			MAP_GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED, BLUE_LED);
			break;
		case MANUAL:
			MAP_GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED | BLUE_LED, GREEN_LED);
			break;
		default:
			break;
	}
}
