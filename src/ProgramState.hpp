#ifndef PROGRAMSTATE_HPP_
#define PROGRAMSTATE_HPP_

#include "Vehicle.hpp"
#include "Imu.hpp"
#include "Px4.hpp"
#include "Lidar.hpp"
#include "FlightMode.hpp"
#include "SdCard.hpp"
#include "messaging/DataLink.hpp"
#include "messaging/feedback_t.h"
#include "Battery.hpp"
#include "RcController.hpp"
#include "RcOutput.hpp"
#include "Switch.hpp"

struct ProgramState
{
	Vehicle *vehicle;
	Imu 	*imu;
	Px4 	*px4;
	Lidar 	*lidar;
	SdCard	*sdcard;
	Battery *battery;
	FlightMode mode;
	DataLink *dLink;
	ThreeSwitch *sw;
	feedback_t *feedback;
	RcController *pilot;
	RcController *kill;
	RcOutput *djiout;

	ProgramState(Vehicle *v, Imu *i, Px4 *px, Lidar *l, SdCard *s, Battery *b,
				 FlightMode m, DataLink *dl, ThreeSwitch *ls, feedback_t *fb,
				 RcController *k, RcController *pi, RcOutput *d);

	bool isAssistedXYPassThrough() const { return sw[0].getReadValue() == 0; }
};

#endif /* PROGRAMSTATE_HPP_ */
