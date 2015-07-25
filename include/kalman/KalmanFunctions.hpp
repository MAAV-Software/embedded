/*
 * KalmanFunctions.hpp
 *
 *  Created on: Jul 1, 2015
 *      Author: clark
 */

#ifndef KALMANFUNCTIONS_HPP_
#define KALMANFUNCTIONS_HPP_

#include "arm_math.h"

void systemDeltaState(const arm_matrix_instance_f32* currState,
					  const arm_matrix_instance_f32* controlInput,
					  const float mass,
					  arm_matrix_instance_f32* deltaState);

void systemGetJacobian(const arm_matrix_instance_f32* currState,
					   const arm_matrix_instance_f32* controlInput,
					   const float mass,
					   arm_matrix_instance_f32* jacobian);

void sensorPredict(const arm_matrix_instance_f32* currState,
				   arm_matrix_instance_f32* sensor);

void sensorGetJacobian(const arm_matrix_instance_f32* currState,
					   arm_matrix_instance_f32* jacobian);

void sensorPredictWithCam(const arm_matrix_instance_f32* currState,
				   arm_matrix_instance_f32* sensor);

void sensorGetJacobianWithCam(const arm_matrix_instance_f32* currState,
					   arm_matrix_instance_f32* jacobian);

#endif /* KALMANFUNCTIONS_HPP_ */
