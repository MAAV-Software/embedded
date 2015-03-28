/*
 * quad_ctrl.h
 *
 * Executing PID outer-loop control of quadrotor with four
 * degrees of freedom to control (X, Y, Z, Yaw). This contains
 * the function declarations.
 *
 *  Created on: July 17, 2014
 *      Author: Sajan Patel, Matt Karashin
 */

#ifndef QUAD_CTRL_H_
#define QUAD_CTRL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "dof.h"

// Enums for control mode
typedef enum ctrl_mode_enum {AUTON_CTRL, RC_CTRL} ctrl_mode_t;

// Enums for array indecies
enum xyhz_enum {X_AXIS, Y_AXIS, Z_AXIS, YAW};
enum rp_enum {ROLL, PITCH};
/*
 * quad_ctrl_t Struct representing a quadrotor, it's controllable degrees of
 * freedom, and related controller properties.
 */
typedef struct _quad_ctrl_t
{
	dof_t xyzh[4];				// [X, Y, Z, Heading(Yaw)] DOF structs

	// Ctrl flags, limits, yaw-frame integrals
	ctrl_mode_t ctrl_mode;			// controler mode
	float vel_cap[4];        	// max velocity in X, Y, Z, and Yaw
	float rp_cap[2];         	// max roll and pitch setpoints

	// Intertial properties of this quadrotor
    float mass;             	// mass of the quadrotor

    // pre-calculated trig values for YAW for one execution loop
    // (so we don't have to recalculate across many different functions
    // that are executed per iteration of the control loop)
	float pre_yaw_sin;
	float pre_yaw_cos;

	// Final values to send to the DJI
	float dji_roll;				// Roll (DJI input A)
	float dji_pitch;			// Pitch (DJI input E)
	float dji_yawDot;			// Yaw Rate (DJI input R)
	float dji_forceZ;			// Z Force/Thrust (DJI input T)
} quad_ctrl_t;

// Creates a new quad_ctrl struct in dynamic memory
quad_ctrl_t* qc_create();

// Initializes the quad_ctrl struct with mass, inertia, gains, etc
void qc_init(quad_ctrl_t *qc);

// Assigns state feedback to the individual DOF structs within quad_ctrl
void qc_setState(quad_ctrl_t *qc, float state[], float t);

// Assigns setpoints based on the controller mode
void qc_setSetpt(quad_ctrl_t *qc, float setpt[], float t);

// Executes all PID control for all DOFs in qc based on ctrl_mode
void qc_runPID(quad_ctrl_t *qc);

// Executes position->velocity PID control and related functions
void qc_runValuePID(quad_ctrl_t *qc);

// Executes velocity->force PID control and related functions
void qc_runRatePID(quad_ctrl_t *qc);

// Calculates Roll, Pitch, Z Force, and Yaw Rate to send to the DJI
void qc_setDJIValues(quad_ctrl_t *qc);

// Cleans up the dynamic memory used to create the quad_ctrl_t struct qc.
void qc_destroy(quad_ctrl_t *qc);

#ifdef __cplusplus
}
#endif

#endif /* QUAD_CTRL_H_ */
