#ifndef PROGRAMSTATE_HPP_
#define PROGRAMSTATE_HPP_

#include "Vehicle.hpp"
#include "Imu.hpp"
#include "Px4.hpp"
#include "Lidar.hpp"

struct ProgramState
{
	Vehicle *vehicle;
	Imu 	*imu;
	Px4 	*px4;
	Lidar 	*lidar;

	ProgramState(Vehicle *v, Imu *i, Px4 *p, Lidar *l);
};

#endif /* PROGRAMSTATE_HPP_ */
