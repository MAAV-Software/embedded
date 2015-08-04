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
#include "messaging/gains_t.h"
#include "messaging/raw_pose_t.h"
#include "messaging/setpt_t.h"

#include "LED.h"

DataLinkRunnable::DataLinkRunnable(ProgramState *pState)
    : ps(pState), lastSetptTime(0), lastGainsTime(0), lastRawPoseTime(0)
{
	DataLinkUartConfig(SYSCTL_PERIPH_UART0, SYSCTL_PERIPH_GPIOA, GPIO_PA0_U0RX,
					   GPIO_PA1_U0TX, GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_PIN_1,
					   UART0_BASE, INT_UART0);
}

void DataLinkRunnable::run()
{
	while (DL_RBUFF.unread() > 0) // send any received bytes to DataLink for parsing
		ps->dLink->processRecv(DL_RBUFF.pop());

	// grab current messages
	setpt = ps->dLink->getSetptMsg();
	gains = ps->dLink->getGainsMsg();

//	char buf[200];
//	uint32_t len = snprintf(buf, sizeof(buf),
//							"\n\nSetpt: %f %f %f %f %d %d %d\n\n",
//							setpt.x, setpt.y, setpt.z, setpt.yaw, setpt.flags, setpt.utime, millis());
//	ps->sdcard->write(buf, len);

	// Logic for dealing with messages depends on the last time they were received
	// Note that the raw_pose_t message is handled in the CtrlRunnable rather than
	// in here to keep it with the other sensor data calls for the filter.
	if (setpt.utime > lastSetptTime)
	{
	    lastSetptTime = setpt.utime;
	    //Toggle_LED(BLUE_LED, SYSCLOCK / 1000); // for debug only

	    //updateVehicleSetpt();
	}

	if (gains.utime > lastGainsTime) // update vehicle gains
	{
	    lastGainsTime = gains.utime;
	    //updateVehicleGains();
	}


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

void DataLinkRunnable::updateVehicleGains()
{
    // set Vehicle gains
    float valueGains[NUM_DOFS][NUM_PID_GAINS];
    float rateGains[NUM_DOFS][NUM_PID_GAINS];
    for (uint8_t i = 0; i < NUM_PID_GAINS; ++i)
    {
        valueGains[X_AXIS][i] = gains.xGains[i];
        valueGains[Y_AXIS][i] = gains.yGains[i];
        valueGains[Z_AXIS][i] = gains.zGains[i];
        valueGains[YAW][i]    = gains.yawGains[i];

        rateGains[X_AXIS][i]  = gains.xGains[i + 3];
        rateGains[Y_AXIS][i]  = gains.yGains[i + 3];
        rateGains[Z_AXIS][i]  = gains.zGains[i + 3];
    }

    ps->vehicle->setGains(valueGains, rateGains);

    // Log gains to EEPROM
    float pidEeprom[NUM_FLOAT];
    for (uint8_t i = 0; i < 3; ++i)
    {
        pidEeprom[i]      = gains.xGains[i];
        pidEeprom[i + 3]  = gains.yGains[i];
        pidEeprom[i + 6]  = gains.zGains[i];
        pidEeprom[i + 9]  = gains.yawGains[i];
        pidEeprom[i + 12] = gains.xGains[i + 3];
        pidEeprom[i + 15] = gains.yGains[i + 3];
        pidEeprom[i + 18] = gains.zGains[i + 3];
        pidEeprom[i + 21] = 0;
    }
    Write_PID_EEPROM(pidEeprom);
}

void DataLinkRunnable::updateVehicleSetpt()
{
    if ((ps->mode != AUTONOMOUS)
            || (setpt.flags == (int8_t)SETPT_T_IDLE))
    {
        return; // in these cases, we don't want to set vehicle setpt
        // IDLE will be taked care of in DJI runnable and rate setpts in CtrlRunnable
        // done by RC and not this message
    }

    float time = (float)millis() / 1000.0f;
    float spArr[NUM_DOFS][NUM_DOF_STATES];
    for (uint8_t i = 0; i < NUM_DOFS; ++i)
    {
        for (uint8_t j = 0; j < NUM_DOF_STATES - 1; ++j)
        {
            spArr[i][j] = 0;
        }
        spArr[i][DOF_TIME] = time;
    }

    switch (setpt.flags)
    {
        case SETPT_T_POSE:
            spArr[X_AXIS][DOF_VAL] = setpt.x;
            spArr[Y_AXIS][DOF_VAL] = setpt.y;
            spArr[Z_AXIS][DOF_VAL] = setpt.z;
            spArr[YAW][DOF_VAL]    = setpt.yaw;
            break;
        case SETPT_T_TAKEOFF:
            spArr[X_AXIS][DOF_VAL] = ps->feedback->x[FEEDBACK_T_VAL];
            spArr[Y_AXIS][DOF_VAL] = ps->feedback->y[FEEDBACK_T_VAL];
            spArr[Z_AXIS][DOF_VAL] = 1.5f;
            spArr[YAW][DOF_VAL]    = ps->feedback->yaw;
            break;
        case SETPT_T_LAND:
            spArr[X_AXIS][DOF_VAL] = ps->feedback->x[FEEDBACK_T_VAL];
            spArr[Y_AXIS][DOF_VAL] = ps->feedback->y[FEEDBACK_T_VAL];
            spArr[Z_AXIS][DOF_VAL] = 0.0f;
            spArr[YAW][DOF_VAL]    = ps->feedback->yaw;
            break;
        default:
            return;
    }

    ps->vehicle->setSetpt(spArr, AUTONOMOUS);
}
