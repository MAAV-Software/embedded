#include "test_definitions.h"
#include "general.h"
#include "test_definitions.h"
#include "flight_mode.h"
#include "led.h"
#include "general.h"
#include "PPM.h"
#include "time_util.h"
#include "servoIn.h"
#include "VehicleState.hpp"
#include "runnables/FlightModeRunnable.hpp"
#include "runnables/DjiRunnable.hpp"
#include "Loop.hpp"

int runnable_test(void) {
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, 0);

	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);	// Chose any open timer
	PPM_init(SYSCTL_PERIPH_TIMER2, SYSCLOCK, TIMER2_BASE, INT_TIMER2A,		// Chose any open timer
			GPIO_PORTB_BASE, GPIO_PIN_6, 4);								// Chose any open port/pin
	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE); // Chose timer4 until encapsulated
	servoIn_attachPin();

	VehicleState state;
	FlightModeRunnable flightModeRunnable(&state);
	DjiRunnable djiRunnable(&state);
	Loop mainLoop;
	mainLoop.addEvent(&flightModeRunnable, 10);
	mainLoop.addEvent(&djiRunnable, 10);
	mainLoop.run();

	return 0;
}



