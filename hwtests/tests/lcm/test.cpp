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


void TestFunction::run()
{
	Config_LED();
	Config_EEPROM();
	time_init(SYSCTL_PERIPH_TIMER1, SYSCLOCK, TIMER1_BASE, INT_TIMER1A);

	//Set up a DataLink Object
	DataLink dl(DataLinkUartSend);

	//Configure UART
	// THIS IS FOR BOTH 2015 AND 2016 SIGNAL BOARDS
	DataLinkUartConfig(SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_GPIOA, GPIO_PA0_U0RX,
					   GPIO_PA1_U0TX, GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_PIN_1,
					   UART0_BASE, INT_UART0);

	int32_t lastSetptTime = 0;
	setpt_t setpt;

	for(;;)
	{
		// Process incoming buffers
		while (DL_RBUFF.unread() > 0) // send any received bytes to DataLink for parsing
			dl.processRecv(DL_RBUFF.pop());

		// grab current message
		setpt = dl.getSetptMsg();

		// if we got a new message, blink the LED
		// new message is determined by INCREASE in timestamp
		if (setpt.utime > lastSetptTime)
		{
			// copy the new time
		    lastSetptTime = setpt.utime;
		    // blink the LED
		    Toggle_LED(BLUE_LED, SYSCLOCK / 1000);
		}
		
		feedback_t msg;
		msg.flags = setpt.flags;
		msg.pitch = msg.roll = msg.yaw = 0;
		msg.x[0] = 10;  msg.x[1] = 1; msg.x[2] = 0;
		msg.y[0] = 20;  msg.y[1] = 0; msg.y[2] = 0;
		msg.z[0] = 1.5; msg.z[1] = 0; msg.z[2] = 0;
		msg.utime = millis();
		dl.send(&msg);

	}


}
