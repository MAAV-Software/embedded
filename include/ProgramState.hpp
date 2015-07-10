#ifndef PROGRAMSTATE_HPP_
#define PROGRAMSTATE_HPP_

#include "Vehicle.hpp"
#include "Imu.hpp"
#include "Px4.hpp"
#include "Lidar.hpp"
#include "FlightMode.hpp"

struct ProgramState
{
	Vehicle *vehicle;
	Imu 	*imu;
	Px4 	*px4;
	Lidar 	*lidar;
	FlightMode mode;

	ProgramState(Vehicle *v, Imu *i, Px4 *p, Lidar *l, FlightMode m);
};

#endif /* PROGRAMSTATE_HPP_ */
