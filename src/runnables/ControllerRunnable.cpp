#include "runnables/ControllerRunnable.hpp"
#include "servoIn.hpp"
#include "PPM.h"
#include "rc.hpp"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"

ControllerRunnable::ControllerRunnable(Vehicle* v)
	: vehicle(v) {}

ControllerRunnable::~ControllerRunnable() { }

void ControllerRunnable::run()
{
	uint32_t yChannel;
	uint32_t xChannel;
	uint32_t zChannel;
	uint32_t yawChannel;
	//switch (vehicle->getFlightMode())
	switch(0)
	{
		case AUTONOMOUS:
			break;
		case ASSISTED:
			yChannel = servoIn_getPulse(RC_CHAN1);
			xChannel = servoIn_getPulse(RC_CHAN2);
			yawChannel = servoIn_getPulse(RC_CHAN4);

//			zChannel = servoIn_getPulse(RC_CHAN3);

			break;
		case MANUAL:
			yChannel = servoIn_getPulse(RC_CHAN1);
			xChannel = servoIn_getPulse(RC_CHAN2);
			zChannel = servoIn_getPulse(RC_CHAN3);
			yawChannel = servoIn_getPulse(RC_CHAN4);
			break;
		default:
			break;
	}

	PPM_setPulse(0, yChannel);	// Y Accel
	PPM_setPulse(1, xChannel);	// X Accel
	PPM_setPulse(2, zChannel);	// Z Accel
	PPM_setPulse(3, yawChannel);	// Yaw Rate
}

