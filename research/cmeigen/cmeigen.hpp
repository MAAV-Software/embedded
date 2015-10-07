// Eigen Wrapper to Emulate CMSIS
#ifndef CMEIGEN_HPP_
#define CMEIGEN_HPP_

#include <stdint.h>
//#include <cstdint>
//#include <cfloat>
//#include <Eigen/Dense>

typedef float float32_t;

enum arm_status 
{
	ARM_MATH_SUCCESS, 
	ARM_MATH_ARGUMENT_ERROR, 
	ARM_MATH_LENGTH_ERROR,
	ARM_MATH_SIZE_MISMATCH,
	ARM_MATH_NANINF,
	ARM_MATH_SINGULAR,
	ARM_MATH_TEST_FAILURE
};

typedef struct
{
	uint16_t numRows;     // number of rows of the matrix.
	uint16_t numCols;     // number of columns of the matrix.
	float32_t *pData;     // points to the data of the matrix.
} arm_matrix_instance_f32;

arm_status arm_mat_add_f32(const arm_matrix_instance_f32* pSrcA,
		const arm_matrix_instance_f32* pSrcB,
		arm_matrix_instance_f32* pDst);

arm_status arm_mat_cmplx_mult_f32(const arm_matrix_instance_f32* pSrcA,
		const arm_matrix_instance_f32* pSrcB,
		arm_matrix_instance_f32* pDst);


void arm_mat_init_f32(arm_matrix_instance_f32* S,
		uint16_t nRows,
		uint16_t nColumns,
		float32_t* pData);

arm_status arm_mat_inverse_f32 (const arm_matrix_instance_f32* pSrc, 
		arm_matrix_instance_f32* pDst);

arm_status arm_mat_mult_f32 (const arm_matrix_instance_f32* pSrcA, 
		const arm_matrix_instance_f32* pSrcB, 
		arm_matrix_instance_f32 *pDst);

arm_status arm_mat_scale_f32 (const arm_matrix_instance_f32* pSrc, 
		float32_t scale, 
		arm_matrix_instance_f32* pDst);


arm_status arm_mat_sub_f32(const arm_matrix_instance_f32* pSrcA, 
		const arm_matrix_instance_f32* pSrcB, 
		arm_matrix_instance_f32* pDst);

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32* pSrc, 
		arm_matrix_instance_f32* pDst);

float32_t arm_sin_f32 (float32_t x);

float32_t arm_cos_f32 (float32_t x);

arm_status arm_sqrt_f32 (float32_t in, float32_t *pOut);

#endif //CMEIGEN_HPP
