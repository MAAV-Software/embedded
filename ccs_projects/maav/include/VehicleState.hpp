#ifndef VEHICLE_STATE_HPP_
#define VEHICLE_STATE_HPP_

#ifdef __cplusplus
extern "C" {
#endif

#include "flight_mode.h"

// state of the vehicle
struct VehicleState {
	FLIGHT_MODE mode;
	float height; // in meters
};

#ifdef __cplusplus
}
#endif

#endif /* VEHICLE_STATE_HPP_ */
