#include "ProgramState.hpp"
#include "Vehicle.hpp"
#include "Imu.hpp"
#include "Px4.hpp"
#include "Lidar.hpp"
#include "SdCard.hpp"
#include "FlightMode.hpp"

ProgramState::ProgramState(Vehicle *v, Imu *i, Px4 *px, Lidar *l, SdCard *s,
        Battery *b, FlightMode m, DataLink *dl, ThreeSwitch *ls,
        feedback_t *fb, RcController *k, RcController *pi, RcOutput *d)
	: vehicle(v), imu(i), px4(px), lidar(l), sdcard(s), battery(b), mode(m),
	  dLink(dl), sw(ls), feedback(fb), kill(k), pilot(pi), djiout(d) {}
