#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include "Dof.hpp"
#include "FlightMode.hpp"

// Enums for array indecies
enum xyzhEnum {X_AXIS, Y_AXIS, Z_AXIS, YAW};
enum rpEnum {ROLL, PITCH};

// enum for Flight Mode
//typedef enum { AUTONOMOUS, MANUAL, ASSISTED } FlightMode;

class Vehicle
{
public:
	// Constructors
	Vehicle();

	Vehicle(const float valueGains[4][3], const float rateGains[4][3],
			const float stateBounds[4], const float velCaps[4],
			const float rpCaps[2], const float UvalPosThresh[4],
			const float UvalNegThresh[4], const uint8_t flags[4],
			const FlightMode modeInit, const float massInit);

	// Destructor
	~Vehicle();

	// Assigns state feedback to the individual DOF structs within the vehicle
	void setState(const float state[], const float t);

	// Assigns setpoints based on the controller mode
	void setSetpt(const float setpt[], const float t);

	// Assigns gains to the DOFs within this Vehicle
	void setGains(const float gains[4][3], const float rateGains[4][3]);

	// Executes all PID control for all DOFs in qc based on ctrl_mode
	void runPID();

	// Calculates Roll, Pitch, Z Force, and Yaw Rate to send to the DJI
	void calcDJIValues();

	// Set the EKF sensor input
	void setEkfSensors();

	// Runs the EKF prediciton step
	void EkfPredict();

	// Runs the EKF correction step, this should also automatically udpdate the
	// states in Dof as well.
	void EkfCorrect();

	// Sets the Flight Mode of the controller
	void setFlightMode(const FlightMode modeIn);

	// Gets the Flight Mode
	FlightMode getFlightMode() const;

	// The following getter functions return the controller values that will be
	// sent to the DJI
	float getDjiRoll() const;
	float getDjiPitch() const;
	float getDjiYawDot() const;
	float getDjiForceZ() const;

	// Returns a pointer to the array of 4 Dofs within the Vehicle so when we
	// do logging, we can use them directly.
	const Dof* getDofs() const;

private:

	// public member variables
	FlightMode mode;

	/// Controller Specific Members
	Dof xyzh[4]; 		// DOF classes for xyzh

	float djiRoll;		// Roll for DJI
	float djiPitch;		// Pitch for DJI
	float djiYawDot;	// Yaw rate for DJI
	float djiForceZ;	// Thrust for DJI
	float mass;			// Mass of the Vehicle

	// private controller variables
	float rpLimits[2];   // max roll and pitch limit

	// pre-calculated trig values for YAW for one execution loop
	// (so we don't have to recalculate across many different functions
	// that are executed per iteration of the control loop)
	float preYawSin;
	float preYawCos;

	/// State Estimator Specific Enums
	// declare EKF instance here



	// private controller functions
	void runValuePID(); // runs Value PIDs on DOF

	void runRatePID(); // runs Rate PIDs on DOF
};

#endif /* VEHICLE_HPP_ */
