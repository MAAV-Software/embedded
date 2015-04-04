#include "runnables/ControllerRunnable.hpp"
#include "PPM.h"
#include "servoIn.h"

ControllerRunnable::ControllerRunnable(VehicleState* state)
: _state(state) {}

ControllerRunnable::~ControllerRunnable() { }

void ControllerRunnable::run() {
	uint32_t yChannel;
	uint32_t xChannel;
	uint32_t zChannel;
	uint32_t yawChannel;
	switch (_state->mode) {
		case AUTONOMOUS:
		case ASSISTED:
//			float height = _state.height;
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

