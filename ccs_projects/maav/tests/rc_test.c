#include "test_definitions.h"
#include "flight_mode.h"
#include "led.h"
#include "general.h"
#include "PPM.h"
#include "time_util.h"
#include "servoIn.h"

int rc_test(void) {
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


	FLIGHT_MODE mode;
	while (1) {
		mode = flightModeGet();
		switch (mode) {
		case AUTONOMOUS:
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
				BLUE_LED, RED_LED);
			break;
		case MANUAL:
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
				BLUE_LED, GREEN_LED);
			break;
		case ASSISTED:
			GPIOPinWrite(GPIO_PORTF_BASE, RED_LED | GREEN_LED |
				BLUE_LED, BLUE_LED);
			break;
		}
		//SysCtlDelay(SYSCLOCK / 10);
	}
	return 0;
}
