#include "ProgramState.hpp"
#include "Vehicle.hpp"
#include "Imu.hpp"
#include "Px4.hpp"
#include "Lidar.hpp"
#include "SdCard.hpp"
#include "FlightMode.hpp"

ProgramState::ProgramState(Vehicle *v, Imu *i, Px4 *p, Lidar *l, SdCard *s,
						   FlightMode m, DataLink *dl, SwitchData_t *ls)
	: vehicle(v), imu(i), px4(p), lidar(l), sdcard(s), mode(m), dLink(dl), sw(ls) {}
