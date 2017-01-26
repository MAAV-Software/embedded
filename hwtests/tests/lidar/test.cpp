#include "TestFunction.hpp"
#include <stdint.h>
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
#include "messaging/gains_t.h"
#include "messaging/raw_pose_t.h"
#include "messaging/setpt_t.h"

#include "Lidar.hpp"
#include "Px4.hpp"

#include "runnables/I2CRunnable.hpp"
#include "servoIn.hpp"




void TestFunction::run()
{
	Config_LED();
	Config_EEPROM();
	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);

	servoIn_init(SYSCTL_PERIPH_TIMER4, TIMER4_BASE); 						// Chose timer4 until encapsulated
	servoIn_attachPin();


	//Initialize the Lidar and dependencies (e.g. I2C)
	Lidar lidar;
	Px4 px4;
	ProgramState pState(0, 0, &px4, &lidar, 0, 0, MANUAL,
		       0, 0,0, 0, 0, 0);
	I2CRunnable i2cRunnable(&pState);

	for(;;)
	{

		i2cRunnable.run();
		float val=lidar.getDist();		//Get values from Lidar
		if (val>1){

			Toggle_LED(BLUE_LED, SYSCLOCK / 1000);

		}

	}


}
