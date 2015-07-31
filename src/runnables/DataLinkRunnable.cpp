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
	//char buf[256];
	char sendBuff[258];
	uint8_t idx = 0;
	while (DL_RBUFF.unread() > 0)
	{
		uint8_t byte = DL_RBUFF.pop();
		//uint32_t len = snprintf(buf, sizeof(buf), "%c\t", (char)byte);
		//ps->sdcard->write(buf, len);
		if (idx < 256) sendBuff[idx++] = (char)byte;
	}
	while (idx < 256) sendBuff[idx++] = '0';
	sendBuff[256] = '\n';
	sendBuff[257] = '\r';
	DataLinkUartSend((uint8_t*)sendBuff, sizeof(sendBuff));
}
