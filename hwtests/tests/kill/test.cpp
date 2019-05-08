#include "TestFunction.hpp"
#include <stdint.h>
#include <stdlib.h>
#include "driverlib/sysctl.h"
#include "LED.h"
#include "time_util.h"

#include "ProgramState.hpp"
#include "messaging/DataLink.hpp"
#include "DataLinkHw.hpp"
#include "LED.h"
#include "time_util.h"
#include "EEPROM.h"

#include "messaging/RingBuffer.hpp"
#include "messaging/emergency_t.h"
#include "messaging/feedback_t.h"

#include "runnables/DjiRunnable.hpp"
#include "servoIn.hpp"
#include "rc.hpp"
#include "RcProfiles.hpp"





void TestFunction::run()
{

	// Set up the necessary prerequisites for the testcase
	Config_LED();
	Config_EEPROM();
	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);

	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE); 						// Chose timer4 until encapsulated
	servoIn_attachPin();


	//Initialize the kill RcController
	RcController kill(Maav::hitec, Maav::hitecNumch);
	ProgramState pState(0, 0, 0, 0, 0, MANUAL, 0, 0, 0, &kill, 0, 0);

	for(;;)
	{

		// Get the duty cycle values for each stick
		float dutyXd = abs(pState.kill->dutyCycle(servoIn_getPulse(KILL_CHAN5), 0)); // pitch (vertical right stick)
//	    float dutyYd = abs(pState.kill->dutyCycle(servoIn_getPulse(KILL_CHAN2), 1)); // roll (horizontal right stick)
		float dutyFz = abs(pState.kill->dutyCycle(servoIn_getPulse(KILL_CHAN3), 2)); // thrust (vertical left stick)
//		float dutyYawd = abs(pState.kill->dutyCycle(servoIn_getPulse(KILL_CHAN4), 3)); // yaw rate (horizontal right stick)
		float dutyYd = 0;
		float dutyYawd = 0;
		// If the yaw is greater than 0.5, flash the active LEDs
		if (dutyYawd > 0.5) {

			// If duty cycle value for pitch is greater than 0.6, turn on the blue LED
			if (dutyXd > 0.6) {

				TurnOff_LED(BLUE_LED);                 //TurnOn_LED and TurnOff_LED do the opposite things

			}

			// If duty cycle value for pitch is greater than 0.5, turn on the red LED
			if (dutyYd > 0.5) {

				TurnOff_LED(RED_LED);

			}

			// If duty cycle value for pitch is greater than 0.5, turn on the green LED
			if (dutyFz > 0.5) {

				TurnOff_LED(GREEN_LED);

			}

			// Keep the LEDs on for a bit, then turn them all off and wait a bit (flash them)
			SysCtlDelay(SYSCLOCK / 3 / 2);
			TurnOn_LED(BLUE_LED);
			TurnOn_LED(RED_LED);
			TurnOn_LED(GREEN_LED);
			SysCtlDelay(SYSCLOCK / 3 / 2);

		// Otherwise, keep the LEDs on solid
		} else {

			// If duty cycle value for pitch is greater than 0.6, turn on the blue LED
			if (dutyXd > 0.6) {

				Toggle_LED(BLUE_LED, SYSCLOCK / 1000);

			}

			// If duty cycle value for pitch is greater than 0.5, turn on the red LED
			if (dutyYd > 0.5) {

				Toggle_LED(RED_LED, SYSCLOCK / 1000);

			}

			// If duty cycle value for pitch is greater than 0.5, turn on the green LED
			if (dutyFz > 0.5) {

				Toggle_LED(GREEN_LED, SYSCLOCK / 1000);

			}

		}

	}

}
