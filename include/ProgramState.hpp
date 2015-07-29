#ifndef PROGRAMSTATE_HPP_
#define PROGRAMSTATE_HPP_

#include "Vehicle.hpp"
#include "Imu.hpp"
#include "Px4.hpp"
#include "Lidar.hpp"
#include "FlightMode.hpp"
#include "SdCard.hpp"
#include "messaging/DataLink.hpp"
#include "switch.h"

struct ProgramState
{
	Vehicle *vehicle;
	Imu 	*imu;
	Px4 	*px4;
	Lidar 	*lidar;
	SdCard	*sdcard;
	FlightMode mode;
	DataLink *dLink;
	SwitchData_t *sw;

	ProgramState(Vehicle *v, Imu *i, Px4 *p, Lidar *l, SdCard *s, FlightMode m,
				 DataLink *dl, SwitchData_t *ls);
};

#endif /* PROGRAMSTATE_HPP_ */
