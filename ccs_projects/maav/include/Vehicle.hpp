#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include "Dof.hpp"

// Enums for array indecies
enum xyzhEnum {X_AXIS, Y_AXIS, Z_AXIS, YAW};
enum rpEnum {ROLL, PITCH};

// enum for Flight Mode
typedef enum { AUTONOMOUS, MANUAL, ASSISTED } FlightMode;

class Vehicle
{
public:
	// public member variables
	FlightMode mode;

	/// Controller Specific Members
	float djiRoll;		// Roll for DJI
	float djiPitch;		// Pitch for DJI
	float djiYawDot;	// Yaw rate for DJI
	float djiForceZ;	// Thrust for DJI
	float mass;			// Mass of the Vehicle
	Dof xyzh[4]; 		// DOF classes for xyzh

	/// State Estimator Specific Enums
	// declare EKF instance here

	// Constructors
	Vehicle();

	Vehicle(float valueGains[4][3], float rateGains[4][3],
			float stateBounds[4], float velCaps[4], float rpCaps[2],
			float UvalPosThresh[4], float UvalNegThresh[4],
			uint8_t flags[4], FlightMode modeInit, float massInit);

	// Destructor
	~Vehicle();

	// Assigns state feedback to the individual DOF structs within quad_ctrl
	void setState(float state[], float t);

	// Assigns setpoints based on the controller mode
	void setSetpt(float setpt[], float t);

	// Assigns gains to the DOFs within this Vehicle
	void setGains(float gains[4][3], float rateGains[4][3]);

	// Executes all PID control for all DOFs in qc based on ctrl_mode
	void runPID();

	// Calculates Roll, Pitch, Z Force, and Yaw Rate to send to the DJI
	void calcDJIValues();

private:
	// private controller variables
	float rpLimits[2];   // max roll and pitch limit

	// pre-calculated trig values for YAW for one execution loop
	// (so we don't have to recalculate across many different functions
	// that are executed per iteration of the control loop)
	float preYawSin;
	float preYawCos;

	// private controller functions
	void runValuePID(); // runs Value PIDs on DOF

	void runRatePID(); // runs Rate PIDs on DOF
};

#endif /* VEHICLE_HPP_ */
