#ifndef QUADCTRL_HPP_
#define QUADCTRL_HPP_

/*
 * quad_ctrl.hpp
 *
 * Executing PID outer-loop control of quadrotor with four
 * degrees of freedom to control (X, Y, Z, Yaw). This contains
 * the function declarations.
 *
 *  Created on: July 17, 2014
 *      Author: Sajan Patel, Matt Karashin
 */

#include "Dof.hpp"

// Enums for control mode
typedef enum ctrlModeEnum {AUTON_CTRL, RC_CTRL} ctrlMode;

// Enums for array indecies
enum xyzhEnum {X_AXIS, Y_AXIS, Z_AXIS, YAW};
enum rpEnum {ROLL, PITCH};

// Quad Control Class
class QuadCtrl {
public:
	float mass; // quadrotor's mass
	ctrlMode mode; // controller mode
	// Final values to send to the DJI
	float djiRoll;				// Roll (DJI input A)
	float djiPitch;			// Pitch (DJI input E)
	float djiYawDot;			// Yaw Rate (DJI input R)
	float djiForceZ;			// Z Force/Thrust (DJI input T)

	//public member functions
	// Default Constructor
	QuadCtrl();

	// Constructor with intial arguments
	QuadCtrl(float valueGains[4][3], float rateGains[4][3],
			 float stateBounds[4], float velCaps[4], float rpCaps[2],
			 float UvalPosThresh[4], float UvalNegThresh[4],
			 uint8_t flags[4], ctrlMode modeInit, float massInit);

	// Destructor
	~QuadCtrl() {}

	// Assigns state feedback to the individual DOF structs within quad_ctrl
	void setQuadState(float* state, float t);

	// Assigns setpoints based on the controller mode
	void setQuadSetpt(float* setpt, float t);

	// Executes all PID control for all DOFs in qc based on ctrl_mode
	void runPID();

	// Calculates Roll, Pitch, Z Force, and Yaw Rate to send to the DJI
	void calcDJIValues();

private:
	float rpLimits[2];   // max roll and pitch limits
	// pre-calculated trig values for YAW for one execution loop
	// (so we don't have to recalculate across many different functions
	// that are executed per iteration of the control loop)
	float preYawSin;
	float preYawCos;
	Dof xyzh[4]; // DOF classes for

	// private member functions
	// Executes position->velocity PID control and related functions
	void runValuePID();

	// Executes velocity->force PID control and related functions
	void runRatePID();
};

#endif // QUADCTRL_HPP
