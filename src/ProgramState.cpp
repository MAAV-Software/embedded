#include "ProgramState.hpp"
#include "Vehicle.hpp"
#include "Imu.hpp"
#include "Px4.hpp"
#include "Lidar.hpp"
#include "FlightMode.hpp"

ProgramState::ProgramState(Vehicle *v, Imu *i, Px4 *p, Lidar *l, FlightMode m)
	: vehicle(v), imu(i), px4(p), lidar(l), mode(m) {}
