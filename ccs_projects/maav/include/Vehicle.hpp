#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include "Dof.hpp"
#include "FlightMode.hpp"


// Defines for array sizes
#define NUM_DOFS	4
#define NUM_ANGLES	2

// Enums for array indecies
enum xyzhEnum {X_AXIS = 0, Y_AXIS, Z_AXIS, YAW};
enum rpEnum {ROLL = 0, PITCH};

class Vehicle
{
public:
	// Constructors
	Vehicle();

	Vehicle(const float valueGains[NUM_DOFS][NUM_GAINS],
			const float rateGains[NUM_DOFS][NUM_GAINS],
			const float stateBounds[NUM_DOFS],
			const float velCaps[NUM_DOFS],
			const float rpCaps[NUM_ANGLES],
			const float UvalPosThresh[NUM_DOFS],
			const float UvalNegThresh[NUM_DOFS],
			const uint8_t flags[NUM_DOFS],
			const FlightMode modeInit,
			const float massInit);

	// Destructor
	~Vehicle();

	// Assigns state feedback to the individual DOF structs within the vehicle
	void setState(const float state[], const float t);

	// Assigns setpoints based on the controller mode
	void setSetpt(const float setpt[], const float t);

	// Assigns gains to the DOFs within this Vehicle
	void setGains(const float gains[NUM_DOFS][NUM_GAINS],
				  const float rateGains[NUM_DOFS][NUM_GAINS]);

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
	Dof xyzh[NUM_DOFS]; 		// DOF classes for xyzh

	float djiRoll;		// Roll for DJI
	float djiPitch;		// Pitch for DJI
	float djiYawDot;	// Yaw rate for DJI
	float djiForceZ;	// Thrust for DJI
	float mass;			// Mass of the Vehicle

	// private controller variables
	float rpLimits[NUM_ANGLES];   // max roll and pitch limit

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
