#include "runnables/FlightModeRunnable.hpp"
#include "FlightMode.hpp"
#include "led.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"


FlightModeRunnable::FlightModeRunnable(Vehicle* v)
	: vehicle(v) {}

FlightModeRunnable::~FlightModeRunnable() { }

void FlightModeRunnable::run() {
	vehicle->setFlightMode(flightModeGet());
	switch (vehicle->getFlightMode()) {
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
