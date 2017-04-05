#include <stdint.h>
#include "runnables/DataLinkRunnable.hpp"
#include "ProgramState.hpp"
#include "messaging/DataLink.hpp"
#include "DataLinkHw.hpp"
#include "LED.h"
#include "time_util.h"
#include "EEPROM.h"

#include "messaging/RingBuffer.hpp"
#include "messaging/emergency_t.h"
#include "messaging/feedback_t.h"
#include "messaging/dji_t.h"

#include "PPM.h"

#include "LED.h"

DataLinkRunnable::DataLinkRunnable(ProgramState *pState)
    : ps(pState)
{
	// THIS IS FOR BOTH 2015 AND 2016 SIGNAL BOARDS
	DataLinkUartConfig(SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_GPIOA, GPIO_PA0_U0RX,
					   GPIO_PA1_U0TX, GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_PIN_1,
					   UART0_BASE, INT_UART0);
}

void DataLinkRunnable::run()
{
	while (DL_RBUFF.unread() > 0) // send any received bytes to DataLink for parsing
		ps->dLink->processRecv(DL_RBUFF.pop());

	dji_t dji = ps->dLink->getDjiMsg();
	ps->vehicle->setCtrlInput(dji.roll, dji.pitch,
			dji.yaw, dji.thrust);

//	char buf[200];
//	uint32_t len = snprintf(buf, sizeof(buf),
//							"\n\nSetpt: %f %f %f %f %d %d %d\n\n",
//							setpt.x, setpt.y, setpt.z, setpt.yaw, setpt.flags, setpt.utime, millis());
//	ps->sdcard->write(buf, len);

//	switchesUpdate(ps->sw);
//	if (ps->sw[2].readState)
//	{
//		uint32_t curTime = millis();
//		while ((millis() - curTime) < 1000)
//		{
//			PPM_setPulse(0, 153372);    // Chan 1 - max
//			PPM_setPulse(1, 86588);    	// Chan 2 - min
//			PPM_setPulse(2, 71602);		// Chan 3 - min
//			PPM_setPulse(3, 152888);    // Chan 4 - max
//		}
//
//		PPM_setPulse(0, 120364);    // Chan 1 - mid
//		PPM_setPulse(1, 119735);    // Chan 2 - mid
//		PPM_setPulse(2, 100000);    // Chan 3
//		PPM_setPulse(3, 119784);    // Chan 4 - mid
//	}

//	// This is for test only!
//	feedback_t msg;
//	msg.flags = setpt.flags;
//	msg.pitch = msg.roll = msg.yaw = 0;
//	msg.x[0] = 10;  msg.x[1] = 1; msg.x[2] = 0;
//	msg.y[0] = 20;  msg.y[1] = 0; msg.y[2] = 0;
//	msg.z[0] = 1.5; msg.z[1] = 0; msg.z[2] = 0;
//	msg.utime = millis();
//	ps->dLink->send(&msg);


//	emergency_t msg;
//	msg.status = 0;
//	ps->dLink->send(&msg);
//	for (int8_t i = 0; i < 3; ++i)
//	{
//		msg.status = i;
//		ps->dLink->send(&msg);
//	}
}
