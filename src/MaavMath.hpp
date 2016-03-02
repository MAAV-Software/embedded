#ifndef MAAV_MATH_HPP_
#define MAAV_MATH_HPP_ 

#include <stdlib.h>
#include <assert.h>

#ifdef LINUX
#include "cmeigen.hpp"
#else
#include "arm_math.h"
#endif

namespace MaavMath {

const float Gravity = 9.80665f;
const float Pi = 3.14159265358979323846264338327950288419716939937510582097494f;
const float Pi_Over_Two = Pi / 2.0f;

void applyTransRotMatrix(const float rotationMatrix[9],
	float x_in, float y_in, float z_in,
	float& x_out, float& y_out, float& z_out);

bool floatClose(float f1, float f2, float threshold);


/**
 * @brief Returns the value of the matrix at the specified row and column
 *
 * @details Returns a value of the float at the spot given by the row and column which can be read or assigned as needed
 *
 * @param mat the matrix you want to get the element from
 * @param row the desired element's row
 * @param col the desired element's column
 */
float mat_at(const arm_matrix_instance_f32& mat, uint16_t row, uint16_t col);

/** 
 * @brief Simple function for initializing a matrix
 *
 * @details Basically just calls arm_mat_init_f32, initializing memory as necessary
 *
 * @param mat a pointer to the matrix being initialized
 * @param rows number of rows in the desired matrix
 * @param cols number of cols in the desired matrix
 */
void mat_init(arm_matrix_instance_f32* mat, uint16_t rows, uint16_t cols);

void mat_destroy(arm_matrix_instance_f32 &mat);

/**
 * @brief Returns a reference to the value of the matrix at the specified row and column
 *
 * @details Returns a reference to the float at the spot given by the row and column which can be read or assigned as needed
 *
 * @param mat the matrix you want to get the element from
 * @param row the desired element's row
 * @param col the desired element's column
 */
float& mat_at(arm_matrix_instance_f32& mat, uint16_t row, uint16_t col);

float mat_at(const arm_matrix_instance_f32& mat, uint16_t row, uint16_t col);

void mat_fill(arm_matrix_instance_f32& mat, float toFill);

void mat_copy(const arm_matrix_instance_f32 &src, arm_matrix_instance_f32 &dest);

}

#endif /* MAAV_MATH_HPP_ */
