#include "ProgramState.hpp"
#include "Vehicle.hpp"
#include "Imu.hpp"
#include "Px4.hpp"
#include "Lidar.hpp"
#include "SdCard.hpp"
#include "FlightMode.hpp"

ProgramState::ProgramState(Vehicle *v, Imu *i, Px4 *p, Lidar *l, SdCard *s,
						   Battery *b, FlightMode m, DataLink *dl, SwitchData_t *ls,
						   feedback_t *fb)
	: vehicle(v), imu(i), px4(p), lidar(l), sdcard(s), battery(b), mode(m), 
	  dLink(dl), sw(ls), feedback(fb) {}
