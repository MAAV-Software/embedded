#include "ProgramState.hpp"
#include "Vehicle.hpp"
#include "Imu.hpp"
#include "Px4.hpp"
#include "Lidar.hpp"

ProgramState::ProgramState(Vehicle *v, Imu *i, Px4 *p, Lidar *l)
	: vehicle(v), imu(i), px4(p), lidar(l) {}
