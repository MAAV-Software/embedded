#ifndef FLIGHTMODE_HPP_
#define FLIGHTMODE_HPP_

#include <stdint.h>

// Enum type definition of the FlightMode
// AUTONOMOUS - Fully autonomous control of the vehicle, when the Atom commands
//		   	    the Tiva with setpoints for X, Y, Z, and Yaw
// ASSISTED   - RC Pilot Controller used to give Xdot, Ydot, Z, and YawRate setpoints
// 				and the controller uses IMU and velocity feedback to stabilize flight 
// 				(a computer-aided manual mode)
// Manual	  - Pure manual RC flight where the RC Pilot controller signals are sent
//				directly to the DJI
typedef enum _FlightMode { AUTONOMOUS, MANUAL, ASSISTED } FlightMode;

const uint8_t CHANGE_ONLY_IF_EXCEEDS = 5; //about a tenth of a second

#endif /* FLIGHTMODE_HPP_ */
