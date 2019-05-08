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

#include "runnables/ImuRunnable.hpp"
#include "servoIn.hpp"




void TestFunction::run()
{

	// Set up the necessary prerequisites for the testcase
	Config_LED();
	Config_EEPROM();
	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);

	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE); 						// Chose timer4 until encapsulated
	servoIn_attachPin();


	//Initialize the Imu (and make an almost empty ProgramState) and ImuRunnable
	Imu imu;
	ProgramState pState(0, &imu, 0, 0, 0, MANUAL, 0, 0, 0, 0, 0, 0);
	ImuRunnable imuRunnable(&pState);

	for(;;)
	{

		imuRunnable.run();

		// Take the absolute value of the acceleration on each axis
		float AccX = abs(imu.getAccX());
		float AccY = abs(imu.getAccY());
		float AccZ = abs(imu.getAccZ());

		// If acceleration on the X axis is greatest, display the blue LED
		if (AccX > AccY && AccX > AccZ) {

			Toggle_LED(BLUE_LED, SYSCLOCK / 1000);

		// If acceleration on the Y axis is greatest, display the red LED
		} else if (AccY > AccX && AccY > AccZ) {

			Toggle_LED(RED_LED, SYSCLOCK / 1000);

		// If acceleration on the Z axis is greatest, display the green LED
		} else if (AccZ > AccX && AccZ > AccY) {

			Toggle_LED(GREEN_LED, SYSCLOCK / 1000);

		}

	}


}
