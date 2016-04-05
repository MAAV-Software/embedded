#include "MaavMath.hpp"
#include <cmath>

namespace MaavMath {

void applyTransRotMatrix(const float rotationMatrix[9],
	float x_in, float y_in, float z_in,
	float& x_out, float& y_out, float& z_out) {

	x_out = x_in * rotationMatrix[0] + y_in * rotationMatrix[3] + z_in * rotationMatrix[6];
	y_out = x_in * rotationMatrix[1] + y_in * rotationMatrix[4] + z_in * rotationMatrix[7];
	z_out = x_in * rotationMatrix[2] + y_in * rotationMatrix[5] + z_in * rotationMatrix[8];
}


bool floatClose(float f1, float f2, float threshold) {
	return std::fabs(f1 - f2) < threshold;
}

/** 
 * @brief Simple function for initializing a matrix
 *
 * @details Basically just calls arm_mat_init_f32, initializing memory as necessary
 *
 * @param mat a pointer to the matrix being initialized
 * @param rows number of rows in the desired matrix
 * @param cols number of cols in the desired matrix
 */
void mat_init(arm_matrix_instance_f32* mat, uint16_t rows, uint16_t cols)
{
	arm_mat_init_f32(mat, rows, cols, (float*)calloc((rows * cols), sizeof(float)));
}

void mat_destroy(arm_matrix_instance_f32 &mat) 
{
    free(mat.pData);
    mat.pData = NULL;
}

/**
 * @brief Returns a reference to the value of the matrix at the specified row and column
 *
 * @details Returns a reference to the float at the spot given by the row and column which can be read or assigned as needed
 *
 * @param mat the matrix you want to get the element from
 * @param row the desired element's row
 * @param col the desired element's column
 */
float& mat_at(arm_matrix_instance_f32& mat, uint16_t row, uint16_t col)
{
	return mat.pData[row * mat.numCols + col];
}

float mat_at(const arm_matrix_instance_f32& mat, uint16_t row, uint16_t col)
{
	return mat.pData[row * mat.numCols + col];
}

void mat_fill(arm_matrix_instance_f32& mat, float toFill)
{
	for (uint16_t i = 0; i < mat.numRows * mat.numCols; i++) 
		mat.pData[i] = toFill;
}

void mat_copy(const arm_matrix_instance_f32 &src, arm_matrix_instance_f32 &dest) {
    #ifdef LINUX
        assert(src.numCols == dest.numCols && src.numRows == dest.numRows);
    #endif
    for(uint16_t i = 0; i < src.numRows * src.numCols; ++i) {
        dest.pData[i] = src.pData[i];
    }
}

// Maps x which is in the range of [fromLow, fromHigh] into the range of
// [toLow, toHigh] and returns the result.
float map(float x, float fromLow, float fromHigh, float toLow, float toHigh)
{
	return (((x - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow)) + toLow;
}

}
