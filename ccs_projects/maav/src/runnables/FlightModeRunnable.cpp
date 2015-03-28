#include "runnables/FlightModeRunnable.hpp"
#include "led.h"

FlightModeRunnable::FlightModeRunnable(VehicleState* state) :
	_state(state) {	}

FlightModeRunnable::~FlightModeRunnable() { }

void FlightModeRunnable::run() {
	_state->mode = flightModeGet();
	switch (_state->mode) {
		case 1:
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
					BLUE_LED, RED_LED);
			break;
		case 2:
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
					BLUE_LED, BLUE_LED);
			break;
		case 3:
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
					BLUE_LED, GREEN_LED);
			break;
	}
}
