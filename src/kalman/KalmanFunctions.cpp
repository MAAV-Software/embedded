/*
 * KalmanFunctions.cpp
 *
 *  Created on: Jul 2, 2015
 *      Author: clark
 */

#include "kalman/KalmanFunctions.hpp"
#include <cassert>

using namespace std;

// TODO: optimize the calls to sin/cos

void systemDeltaState(const arm_matrix_instance_f32* currState,
					  const arm_matrix_instance_f32* controlInput,
					  const float mass,
					  const float sinR,
					  const float cosR,
					  const float sinP,
					  const float cosP,
					  const float sinY,
					  const float cosY,
					  arm_matrix_instance_f32* deltaState) 
{
    assert(currState->numRows == 6);
    assert(currState->numCols == 1);
    assert(controlInput->numRows == 4);
    assert(controlInput->numCols == 1);
    assert(deltaState->numRows == 6);
    assert(deltaState->numCols == 1);

    //const float gravity = 9.81;

    deltaState->pData[0] = currState->pData[3]; // dx = xdot
    deltaState->pData[1] = currState->pData[4]; // dy = ydot
    deltaState->pData[2] = currState->pData[5]; // dz = zdot
    deltaState->pData[3] = controlInput->pData[0] / mass * ((cosY * sinP * cosR) + (sinY * sinR));
    deltaState->pData[4] = controlInput->pData[0] / mass * ((sinY * sinP * cosR) - (cosY * sinR));
    deltaState->pData[5] = (controlInput->pData[0] / mass * (cosP * cosR));
    // don't include gravity here becuase u[0] (body frame Fz control input) already has mg offset for vehicle
    //deltaState->pData[5] = (controlInput->pData[0] / mass * (cosP * cosR)) + gravity;

/*** Old code ***/
//	// sanity checks
//	assert(currState->numRows == 9);
//	assert(currState->numCols == 1);
//	assert(controlInput->numRows == 4);
//	assert(controlInput->numCols == 1);
//	assert(deltaState->numRows == 9);
//	assert(deltaState->numCols == 1);
//
//	deltaState->pData[0] = currState->pData[3]; // dx = xdot
//	deltaState->pData[1] = currState->pData[4]; // dy = ydot
//	deltaState->pData[2] = currState->pData[5]; // dz = zdot
//	deltaState->pData[3] = -controlInput->pData[0]
//						   * arm_sin_f32(currState->pData[7]) / mass;
//	deltaState->pData[4] = controlInput->pData[0]
//						   * arm_sin_f32(currState->pData[6])
//						   * arm_cos_f32(currState->pData[7]) / mass;
//	deltaState->pData[5] = controlInput->pData[0]
//						   * arm_cos_f32(currState->pData[6])
//						   * arm_cos_f32(currState->pData[7]) / mass;
//	deltaState->pData[6] = 0;
//	deltaState->pData[7] = 0;
//	deltaState->pData[8] = controlInput->pData[3];
}

void systemGetJacobian(const arm_matrix_instance_f32* currState,
					   const arm_matrix_instance_f32* controlInput,
					   arm_matrix_instance_f32* jacobian) 
{
	assert(currState->numRows == 6);
	assert(currState->numCols == 1);
	assert(currState->numRows == 6);
	assert(currState->numCols == 1);
	assert(controlInput->numRows == 4);
	assert(controlInput->numCols == 1);
	assert(jacobian->numRows == 6);
	assert(jacobian->numCols == 6);

	memset(jacobian->pData, 0, sizeof(float) * jacobian->numRows * jacobian->numCols);

	jacobian->pData[3]  = 1;
	jacobian->pData[10] = 1;
	jacobian->pData[17] = 1;

//	// sanity checks
//	assert(currState->numRows == 9);
//	assert(currState->numCols == 1);
//	assert(controlInput->numRows == 4);
//	assert(controlInput->numCols == 1);
//	assert(jacobian->numRows == 9);
//	assert(jacobian->numCols == 9);
//
//	memset(jacobian->pData, 0, sizeof(float) * jacobian->numRows * jacobian->numCols);
//	jacobian->pData[3]  = 1;
//	jacobian->pData[13] = 1;
//	jacobian->pData[23] = 1;
//	jacobian->pData[34] = (-controlInput->pData[0] / mass) * arm_cos_f32(currState->pData[7]);
//	jacobian->pData[42] = (-controlInput->pData[0] / mass) * arm_cos_f32(currState->pData[6])
//						  * arm_cos_f32(currState->pData[7]);
//	jacobian->pData[43] = (-controlInput->pData[0] / mass) * arm_sin_f32(currState->pData[6])
//						  * arm_sin_f32(currState->pData[7]);
//	jacobian->pData[50] = (-controlInput->pData[0] / mass) * arm_sin_f32(currState->pData[6])
//						  * arm_cos_f32(currState->pData[7]);
//	jacobian->pData[51] = (-controlInput->pData[0] / mass) * arm_cos_f32(currState->pData[6])
//						  * arm_sin_f32(currState->pData[7]);
}

void sensorPredict(const arm_matrix_instance_f32* currState, 
				   arm_matrix_instance_f32* sensor) 
{
	assert(currState->numRows == 6);
	assert(currState->numCols == 1);
	assert(sensor->numRows == 3);
	assert(sensor->numCols == 1);

	// sensor indecies are: z, xdot, ydot
	sensor->pData[0] = currState->pData[2];
	sensor->pData[1] = currState->pData[3];
	sensor->pData[2] = currState->pData[4];

//	// sanity checks
//	assert(currState->numRows == 9);
//	assert(currState->numCols == 1);
//	assert(sensor->numRows == 6);
//	assert(sensor->numCols == 1);
//
//	sensor->pData[0] = currState->pData[6];
//	sensor->pData[1] = currState->pData[7];
//	sensor->pData[2] = currState->pData[8];
//	sensor->pData[3] = currState->pData[3] * arm_cos_f32(currState->pData[8]);
//	sensor->pData[4] = currState->pData[3] * arm_sin_f32(currState->pData[8]);
//	sensor->pData[5] = currState->pData[2]
//					   / (arm_cos_f32(currState->pData[6]) * arm_cos_f32(currState->pData[7]));
}

void sensorGetJacobian(const arm_matrix_instance_f32* currState,
					   arm_matrix_instance_f32* jacobian) 
{
	assert(currState->numRows == 6);
	assert(currState->numCols == 1);
	assert(jacobian->numRows == 3);
	assert(jacobian->numCols == 6);
	memset(jacobian->pData, 0, sizeof(float) * jacobian->numRows * jacobian->numCols);

	jacobian->pData[2]  = 1;
	jacobian->pData[9]  = 1;
	jacobian->pData[16] = 1;

//	// sanity checks
//	assert(currState->numRows == 9);
//	assert(currState->numCols == 1);
//	assert(jacobian->numRows == 6);
//	assert(jacobian->numCols == 9);
//
//	memset(jacobian->pData, 0, sizeof(float) * jacobian->numRows * jacobian->numCols);
//	jacobian->pData[6]  = 1;
//	jacobian->pData[16] = 1;
//	jacobian->pData[26] = 1;
//	jacobian->pData[30] = arm_cos_f32(currState->pData[8]);
//	jacobian->pData[35] = -currState->pData[3] * arm_sin_f32(currState->pData[8]);
//	jacobian->pData[39] = arm_sin_f32(currState->pData[8]);
//	jacobian->pData[44] = currState->pData[3] * arm_cos_f32(currState->pData[8]);
//	jacobian->pData[47] = 1 / (arm_cos_f32(currState->pData[6]) * arm_cos_f32(currState->pData[7]));
//	jacobian->pData[51] = (currState->pData[2] * arm_sin_f32(currState->pData[6]))
//						  / (arm_cos_f32(currState->pData[6]) * arm_cos_f32(currState->pData[6])
//						     * arm_cos_f32(currState->pData[7]));
//	jacobian->pData[52] = (currState->pData[2] * arm_sin_f32(currState->pData[7]))
//						  / (arm_cos_f32(currState->pData[6]) * arm_cos_f32(currState->pData[7])
//							 * arm_cos_f32(currState->pData[7]));
}


void sensorPredictWithCam(const arm_matrix_instance_f32* currState,
				   	   	  arm_matrix_instance_f32* sensor)
{
	assert(currState->numRows == 6);
	assert(currState->numCols == 1);
	assert(sensor->numRows == 5);
	assert(sensor->numCols == 1);

	// sensor is now x, y, z, xdot, ydot,
	sensor->pData[0] = currState->pData[0];
	sensor->pData[1] = currState->pData[1];
	sensor->pData[2] = currState->pData[2];
	sensor->pData[3] = currState->pData[3];
	sensor->pData[4] = currState->pData[4];
}

void sensorGetJacobianWithCam(const arm_matrix_instance_f32* currState,
					   	   	  arm_matrix_instance_f32* jacobian)
{
	assert(currState->numRows == 6);
	assert(currState->numCols == 1);
	assert(jacobian->numRows == 5);
	assert(jacobian->numCols == 6);

	memset(jacobian->pData, 0, sizeof(float) * jacobian->numRows * jacobian->numCols);

	jacobian->pData[0]  = 1;
	jacobian->pData[7]  = 1;
	jacobian->pData[14] = 1;
	jacobian->pData[21] = 1;
	jacobian->pData[28] = 1;
}
