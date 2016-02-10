#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP
#endif

#ifdef LINUX
#include "cmeigen.hpp"
#else
#include "arm_math.h"
#endif

#include <cassert>
#include <cstdlib>
#include <stdint.h>


/**
 * @brief Returns the value of the matrix at the specified row and column
 *
 * @details Returns a value of the float at the spot given by the row and column which can be read or assigned as needed
 *
 * @param mat the matrix you want to get the element from
 * @param row the desired element's row
 * @param col the desired element's column
 */
float mat_noref_at(const arm_matrix_instance_f32& mat, uint16_t row, uint16_t col);

class KalmanFilter
{
public:
	KalmanFilter();

	~KalmanFilter();

	/*
	 * Run the predict step whenever we want updated state info,
	 * but don't have new sensor data.
	 */
	void predict(float xddot, float yddot, float zddot, float dt);

	/*
	 * Run when we get new lidar info so we can correct where we are.
	 */
	void correctLidar(const float z, const float zdot);

	void correctPx4(const float xdot, const float ydot);

	void correctCamera(const float x, const float y);

	/*
	 * Allows for setting Q and R matrices
	 */
	void setQ(float val1, float val2, float val3, float val4, float val5, float val6);

	void setR_lidar(float val1, float val2);

	void setR_Px4(float val1, float val2);

	void setR_camera(float val1, float val2);

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
    enum CorrectionType { LIDAR, PX4, CAMERA };
	void correct2(const arm_matrix_instance_f32& z, const CorrectionType sensor);
	
	//Make sure nobody tries to assign Kalman filters.
	KalmanFilter& operator=(const KalmanFilter& a);
	KalmanFilter(const KalmanFilter& b);

	const uint16_t n_size; //size of the state vector
	const uint16_t u_size; //size of control input
	const uint16_t l_size; //size of lidar input
	const uint16_t p_size; //size of Px4 input
	const uint16_t c_size; //size of camera input

	arm_matrix_instance_f32 state; //aka x
	arm_matrix_instance_f32 P; //covariances
	arm_matrix_instance_f32 A;
	arm_matrix_instance_f32 B;
	arm_matrix_instance_f32 Q; //noise matrices
	arm_matrix_instance_f32 u;
    
    arm_matrix_instance_f32 R_lidar;
	arm_matrix_instance_f32 R_Px4;
	arm_matrix_instance_f32 R_camera;
	arm_matrix_instance_f32 H_lidar;
	arm_matrix_instance_f32 H_Px4;
	arm_matrix_instance_f32 H_camera;
    arm_matrix_instance_f32 z_2by1;//for the Px4, camera, and lidar
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
