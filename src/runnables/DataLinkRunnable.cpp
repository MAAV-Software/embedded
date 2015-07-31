#include <stdint.h>
#include "runnables/DataLinkRunnable.hpp"
#include "ProgramState.hpp"
#include "messaging/DataLink.hpp"
#include "DataLinkHw.hpp"
#include "LED.h"
#include "time_util.h"

#include "messaging/RingBuffer.hpp"
#include "messaging/emergency_t.h"
#include "messaging/feedback_t.h"
#include "messaging/gains_t.h"
#include "messaging/raw_pose_t.h"
#include "messaging/setpt_t.h"

#include "LED.h"

static char buf[256];

DataLinkRunnable::DataLinkRunnable(ProgramState *pState) : ps(pState)
{
	DataLinkUartConfig(SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_GPIOA, GPIO_PA0_U0RX,
					   GPIO_PA1_U0TX, GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_PIN_1,
					   UART0_BASE, INT_UART0);
}

void DataLinkRunnable::run()
{
	/*
	while (DL_RBUFF.unread() > 0) ps->dLink->processRecv(DL_RBUFF.pop());

	setpt_t setpt = ps->dLink->getSetptMsg();
	uint32_t len = snprintf(buf, sizeof(buf),
							"\nSetpt: %f %f %f %f %d %d %d %d\n",
							setpt.x, setpt.y, setpt.z, setpt.yaw, setpt.flags,
							setpt.setptType, setpt.utime, millis());
	ps->sdcard->write(buf, len);
	*/
	while (DL_RBUFF.unread() > 0)
	{
		uint32_t len = snprintf(buf, sizeof(buf), "%c\t", (char)DL_RBUFF.pop());
		ps->sdcard->write(buf, len);
	}
	uint8_t sendBuf[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '\n', '\r'};
	DataLinkUartSend(sendBuf, sizeof(sendBuf));
}
