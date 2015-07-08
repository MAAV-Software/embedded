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

	ProgramState(Vehicle *V = NULL, Imu *I = NULL, Px4 *P = NULL, Lidar *L = NULL)
	:vehicle(V),imu(I),px4(P),lidar(L){};

};

#endif /* PROGRAMSTATE_HPP_ */
