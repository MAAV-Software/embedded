/*
 * KalmanFunctions.hpp
 *
 *  Created on: Jul 1, 2015
 *      Author: clark
 */

#ifndef KALMANFUNCTIONS_HPP_
#define KALMANFUNCTIONS_HPP_

/* temporary hack to let us use our own implementation of a subset of arm_math.h */
#ifdef LINUX
#include "cmeigen.hpp"
#else
#include "arm_math.h"
#endif

void systemDeltaState(const arm_matrix_instance_f32* currState,
					  const arm_matrix_instance_f32* controlInput,
					  const float mass,
					  const float sinR,
					  const float cosR,
					  const float sinP,
					  const float cosP,
					  const float sinY,
					  const float cosY,
					  arm_matrix_instance_f32* deltaState);

void systemGetJacobian(const arm_matrix_instance_f32* currState,
					   const arm_matrix_instance_f32* controlInput,
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