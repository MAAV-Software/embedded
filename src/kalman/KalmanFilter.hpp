#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
#endif

#ifdef LINUX
#include "cmeigen.hpp"
#else
#include "arm_math.h"
#endif

#include<stdlib.h>
#include <stdint.h>

class KalmanFilter
{
public:
	/*TODO arguments for like all of these*/
	KalmanFilter();

	~KalmanFilter();

	/*
	 * Run the predict step whenever we want updated state info,
	 * but don't have new sensor data.
	 */
	void predict(const arm_matrix_instance_f32 u, float delta_t);

	/*
	 * Run when we get new lidar info so we can correct where we are.
	 */
	void correctLidar(const arm_matrix_instance_f32& z);

	void correctPx4(const arm_matrix_instance_f32& z);

	void correctCamera();

	/*
	 * Returns a pointer to the state data. Const because we do not want
	 * people messing with the data.
	 */
	const arm_matrix_instance_f32& getState() const;

	/*
	 * Returns pointer to the covariance data. Again, const because we
	 * don't want people to go around setting things.
	 */
	const arm_matrix_instance_f32& getCovar() const;

	/*
	 * Reset all of our data to zero.
	 */
	void reset();
private:
	//Make sure nobody tries to assign Kalman filters.
	void correct2(const arm_matrix_instance_f32& z, const bool Px4);
	KalmanFilter& operator=(const KalmanFilter& a);
	KalmanFilter(const KalmanFilter& b);
	int n; //size of the state vector
	int u; //size of control input
	int l; //size of lidar input
	int p; //size of Px4 input
	int c; //size of camera input
	arm_matrix_instance_f32 state; //aka x
	arm_matrix_instance_f32 P; //covariances
	//float* last_camera;
	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 B;
	arm_matrix_instance_f32 Q; //noise matrices
	arm_matrix_instance_f32 R_lidar;
	arm_matrix_instance_f32 R_Px4;
	//arm_matrix_instance_f32 R_camera;
	arm_matrix_instance_f32 H_lidar;
	arm_matrix_instance_f32 H_Px4;
	//arm_matrix_instance_f32 H_camera;
	arm_matrix_instance_f32 inter_nby1;
	arm_matrix_instance_f32 inter_nbyn;
	arm_matrix_instance_f32 inter_another_nbyn;
	arm_matrix_instance_f32 inter_H;
	arm_matrix_instance_f32 inter_2by1;
	arm_matrix_instance_f32 inter_nby2;
	arm_matrix_instance_f32 inter_another_nby2;
	arm_matrix_instance_f32 inter_2byn;
	arm_matrix_instance_f32 inter_2by2;
	arm_matrix_instance_f32 inter_another_2by2;
};
