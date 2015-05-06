#ifndef QUADCTRL_HPP
#define QUADCTRL_HPP

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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "Dof.hpp"

// Enums for control mode
typedef enum ctrlModeEnum {AUTON_CTRL, RC_CTRL} ctrlMode;

// Enums for array indecies
enum xyzhEnum {X_AXIS, Y_AXIS, Z_AXIS, YAW};
enum rpEnum {ROLL, PITCH};

// Quad Control Class
class QuadCtrl
{
private:
	float _rateLimits[4]; // max rate limits for X, Y, Z, and Yaw
	float _rpLimits[2];   // max roll and pitch limits
	// pre-calculated trig values for YAW for one execution loop
	// (so we don't have to recalculate across many different functions
	// that are executed per iteration of the control loop)
	float _preYawSin;
	float _preYawCos;
	Dof _xyzh[4]; // DOF classes for

	// private member functions
	// Executes position->velocity PID control and related functions
	void runValuePID();

	// Executes velocity->force PID control and related functions
	void runRatePID();

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

	// Todo: Constructor with intial arguments

	// Destructor
	~QuadCtrl();

	// Assigns state feedback to the individual DOF structs within quad_ctrl
	void setState(float state[], float t);

	// Assigns setpoints based on the controller mode
	void setSetpt(float setpt[], float t);

	// Executes all PID control for all DOFs in qc based on ctrl_mode
	void runPID();

	// Calculates Roll, Pitch, Z Force, and Yaw Rate to send to the DJI
	void calcDJIValues();
};

#endif // QUADCTRL_HPP
