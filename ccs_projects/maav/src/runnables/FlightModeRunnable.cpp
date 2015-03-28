#include "runnables/FlightModeRunnable.hpp"
#include "led.h"

FlightModeRunnable::FlightModeRunnable(VehicleState* state) :
	_state(state) {	}

FlightModeRunnable::~FlightModeRunnable() { }

void FlightModeRunnable::run() {
	_state->mode = flightModeGet();
	switch (_state->mode) {
		case ASSISTED:
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
					BLUE_LED, RED_LED);
			break;
		case AUTONOMOUS:
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
					BLUE_LED, BLUE_LED);
			break;
		case MANUAL:
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
					BLUE_LED, GREEN_LED);
			break;
		default:
			break;
	}
}
