#ifndef VEHICLE_HPP_
#define VEHICLE_HPP_

#include "Dji.hpp"
#include "Dof.hpp"
#include "Pid.hpp"
#include "FlightMode.hpp"

// Defines for array sizes
#define NUM_DOFS	4
#define NUM_ANGLES	2
#define ROTMAT_SIZE 9

// Enums for array indecies
enum xyzhEnum {X_AXIS, Y_AXIS, Z_AXIS, YAW};
enum rpEnum {ROLL, PITCH};

class Vehicle
{
public:
	// Public Methods
	//TODO ADD EKF TO CONSTRUCTORS
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
				 const float valueErrorLpCoeffS[NUM_DOFS],
				 const float rateStateLpCoeffs[NUM_DOFS],
				 const float rateErrorLpCoeffs[NUM_DOFS],
				 const float totalMass,
				 const float initTime,
				 const float rpLims[NUM_ANGLES],
				 const float initRotMat[ROTMAT_SIZE]);
	
	// TODO Implement this when the State Estimator is ready
	// Destructor (will need to free memory allocation for EKF)
	//~Vehicle();
	
	// Assigns state feedback to the individual DOF structs within the vehicle
	void setState(const float state[NUM_DOFS][NUM_DOF_STATES], 
				  const float rotation[ROTMAT_SIZE]);
					
	// Assigns setpoints based on the controller mode
	void setSetpt(const float setpt[NUM_DOFS][NUM_DOF_STATES], 
				  FlightMode mode);
	
	// Assigns gains to the DOFs within this Vehicle
	void setGains(const float valueGains[NUM_DOFS][NUM_PID_GAINS],
				  const float rateGains[NUM_DOFS][NUM_PID_GAINS]);
	
	// Executes all PID control for all DOFs in qc based on ctrl_mode
	void runCtrl(FlightMode mode);
	

// TODO Implement these when the state estimator is ready	
	// Set the EKF sensor input
//	void setEkfSensors(const float rotation[ROTMAT_SIZE], const float t);
	// Runs the EKF prediciton step
//	void EkfPredict(float t);
	// Runs the EKF correction step, this should also automatically udpdate the
	// states in Dof as well.
//	void EkfCorrect(float t);
	
	
	Dji getDjiVals() const;

	// Logging function
	void prepareLog();

private:
	// Controller specific Members
	Dof dofs[NUM_DOFS]; 		// DOF classes for xyzh
	Dji dji;					// DJI struct of values
	float mass;					// Mass of the Vehicle
	float rpLimits[NUM_ANGLES];	// max roll and pitch limit
	float rotMat[ROTMAT_SIZE];	// Rotation matrix from sensor feedback
	float time;					// Timestamp for the rotation Matrix

	// Calculates Roll, Pitch, Z Force, and Yaw Rate to send to the DJI
	void calcDJIValues();

	// State Estimator Specific Members
	// TODO Add EKF here	
};

#endif /* VEHICLE_HPP_ */
