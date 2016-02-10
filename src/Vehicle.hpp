#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include "Dji.hpp"
#include "Dof.hpp"
#include "Pid.hpp"
#include "kalman/KalmanFilter.hpp"
#include "FlightMode.hpp"
#include "CtrlLogs.hpp"

// Defines for array sizes
#define NUM_DOFS	4
#define NUM_ANGLES	2

// Enums for array indecies
enum xyzhEnum {X_AXIS, Y_AXIS, Z_AXIS, YAW};
enum rpEnum {ROLL, PITCH};

class Vehicle
{
public:
	// Public Methods
	// Constructors
	//Vehicle(){};
	Vehicle(const float valueGains[NUM_DOFS][NUM_PID_GAINS],
			const float rateGains[NUM_DOFS][NUM_PID_GAINS]);

	Vehicle(const float states[NUM_DOFS][NUM_DOF_STATES],
			const float setpts[NUM_DOFS][NUM_DOF_STATES],
			const float valueGains[NUM_DOFS][NUM_PID_GAINS],
			const float rateGains[NUM_DOFS][NUM_PID_GAINS],
			const uint8_t valueFlags[NUM_DOFS],
			const uint8_t rateFlags[NUM_DOFS],
			const float inertias[NUM_DOFS],
			const float stateBounds[NUM_DOFS],
			const float rateUpLims[NUM_DOFS],
			const float rateLwLims[NUM_DOFS],
			const float accelUpLims[NUM_DOFS],
			const float accelLwLims[NUM_DOFS],
			const float valueStateLpCoeffs[NUM_DOFS],
			const float valueErrorLpCoeffs[NUM_DOFS],
			const float rateStateLpCoeffs[NUM_DOFS],
			const float rateErrorLpCoeffs[NUM_DOFS],
			const float totalMass,
			const float initTime,
			const float rpLims[NUM_ANGLES],
			/* 24/11/2015 changed by hlx*/
			const float ekfInitState[6],
			const float ekfInitP[36],
			float ekfQ[46],
			float ekfNoCamR[9],
			float ekfWithCamR[25]);
	
	// Destructor (will need to free memory allocation for EKF)
	~Vehicle();
	
	// Assigns setpoints based on the controller mode
	void setSetpt(const float setpt[NUM_DOFS][NUM_DOF_STATES], 
				  const FlightMode mode);
	
	// Assigns gains to the DOFs within this Vehicle
	void setGains(const float valueGains[NUM_DOFS][NUM_PID_GAINS],
				  const float rateGains[NUM_DOFS][NUM_PID_GAINS]);
	
	// Executes all PID control for all DOFs based on flight mode and calcs DJI vals
	void runCtrl(const FlightMode mode);
	
	// updates sensor inputs, runs the EKF, and updates the states in the DOFs
	// times are seconds
	void runFilter(const float rotationMatrix[9], float yaw,
			float imuX, float imuY, float imuZ, float currTime,
			float lidar, float lidarTime,
			float px4X, float px4Y, float px4Time,
			float cameraX, float cameraY, float cameraTime);
	
	// returns the DJI values needed to send to it
	Dji getDjiVals() const;

	// Logging function
	void prepareLog(VehicleLog &vlog, PidLog plogs[NUM_DOFS][2]);

	void setDofStates(const float state[NUM_DOFS][NUM_DOF_STATES]);

private:
	// Controller specific Members
	Dof dofs[NUM_DOFS]; 		// DOF classes for xyzh
	Dji dji;					// DJI struct of values
	float mass;					// Mass of the Vehicle
	float rpLimits[NUM_ANGLES];	// max roll and pitch limit

	float preYawSin;			// precaculated sin and cos (in next line) of current vehicle yaw
	float preYawCos;
	
	// State Estimator Specific Members	
	KalmanFilter kalmanFilter;
	float lastPredictTime;
	float lastLidarTime;
	float lastPx4Time;
	float lastCameraTime;
	float lastLidarArenaZ;
	
	// Calculates Roll, Pitch, Z Force, and Yaw Rate to send to the DJI
	void calcDJIValues(const FlightMode mode);

	// Updates the states of the DOFS
	//void setDofStates(const float state[NUM_DOFS][NUM_DOF_STATES]);
};

#endif /* VEHICLE_HPP_ */
