#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include "Dji.hpp"
#include "Dof.hpp"
#include "Pid.hpp"
#include "kalman/ExtendedKalmanFilter.hpp"
#include "kalman/KalmanFunctions.hpp"
#include "FlightMode.hpp"

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
	Vehicle();
	Vehicle(const float states[NUM_DOFS][NUM_DOF_STATES],
				 const float setpts[NUM_DOFS][NUM_DOF_STATES],
				 const float valueGains[NUM_DOFS][NUM_PID_GAINS],
				 const float rateGains[NUM_DOFS][NUM_PID_GAINS],
				 const float valueFlags[NUM_DOFS],
				 const float rateFlags[NUM_DOFS],
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
				 const float rpLims[NUM_ANGLES]);
	
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
	void runFilter(const float x, const float y, const float z,
				   const float xdot, const float ydot, const float roll, 
				   const float pitch, const float yaw, const float time);
	
	// returns the DJI values needed to send to it
	Dji getDjiVals() const;

	// Logging function
	void prepareLog(VehicleLog &vlog, PidLogs plogs[NUM_DOFS][2]);

private:
	// Controller specific Members
	Dof dofs[NUM_DOFS]; 		// DOF classes for xyzh
	Dji dji;					// DJI struct of values
	float mass;					// Mass of the Vehicle
	float rpLimits[NUM_ANGLES];	// max roll and pitch limit
	float lastPredTime; 		// time of the last ekf predict
	float time;					// current time
	
	// State Estimator Specific Members	
	ExtendedKalmanFilter *ekf;
	arm_matrix_instance_f32 controlInputMat;
	arm_matrix_instance_f32 sensorMeasurementMat;
	
	// Calculates Roll, Pitch, Z Force, and Yaw Rate to send to the DJI
	void calcDJIValues();

	// Updates the states of the DOFS
	void setDofStates(const float state[NUM_DOFS][NUM_DOF_STATES])
};

#endif /* VEHICLE_HPP_ */
