#include <cmath>
//#include <cstdint>
#include <stdint.h>
#include <Eigen/Dense>
#include "cmeigen.hpp"

using namespace Eigen;
using namespace std;

arm_status arm_mat_add_f32(const arm_matrix_instance_f32* pSrcA,
		const arm_matrix_instance_f32* pSrcB,
		arm_matrix_instance_f32* pDst)
{
	// Check sizing
	if(pSrcA->numRows != pSrcB->numRows 
			|| pSrcA->numCols != pSrcB->numCols
			|| pSrcA->numRows != pDst->numRows 
			|| pSrcA->numCols != pDst->numCols)
	{
		return ARM_MATH_SIZE_MISMATCH;
	}
	// Convert to Eigen matrix
	MatrixXf A(pSrcA->numRows, pSrcA->numCols);
	MatrixXf B(pSrcB->numRows, pSrcB->numCols);
	for(int i = 0; i < pSrcA->numRows; ++i)
	{
		for(int j = 0; j < pSrcA->numCols; ++j)
		{
			A(i,j) = pSrcA->pData[pSrcA->numCols * i + j];
			B(i,j) = pSrcB->pData[pSrcA->numCols * i + j];
		}
	}
	// Perform the addition
	MatrixXf R = A + B;
	// Convert back
	for(int i = 0; i < pSrcA->numRows; ++i)
	{
		for(int j = 0; j < pSrcA->numCols; ++j)
		{
			pDst->pData[pSrcA->numCols * i + j] = R(i,j);
		}
	}
	return ARM_MATH_SUCCESS;
}

arm_status arm_mat_cmplx_mult_f32(const arm_matrix_instance_f32* pSrcA,
		const arm_matrix_instance_f32* pSrcB,
		arm_matrix_instance_f32* pDst)
{
	// Check sizing
	if(pSrcA->numCols != pSrcB->numRows
			|| pSrcA->numRows != pDst->numRows 
			|| pSrcB->numCols != pDst->numCols)
	{
		return ARM_MATH_SIZE_MISMATCH;
	}
	// Convert to Eigen matrix
	MatrixXf A(pSrcA->numRows, pSrcA->numCols);
	MatrixXf B(pSrcB->numRows, pSrcB->numCols);
	for(int i = 0; i < pSrcA->numRows; ++i)
	{
		for(int j = 0; j < pSrcA->numCols; ++j)
		{
			A(i,j) = pSrcA->pData[pSrcA->numCols * i + j];
		}
	}
	for(int i = 0; i < pSrcB->numRows; ++i)
	{
		for(int j = 0; j < pSrcB->numCols; ++j)
		{
			B(i,j) = pSrcB->pData[pSrcB->numCols * i + j];
		}
	}
	// Perform the multiplication
	MatrixXf R = A * B;
	// Convert back
	for(int i = 0; i < pDst->numRows; ++i)
	{
		for(int j = 0; j < pDst->numCols; ++j)
		{
			pDst->pData[pDst->numCols * i + j] = R(i,j);
		}
	}
	return ARM_MATH_SUCCESS;
}


void arm_mat_init_f32(arm_matrix_instance_f32* S,
		uint16_t nRows,
		uint16_t nColumns,
		float32_t* pData)
{
	S->numRows = nRows;
	S->numCols = nColumns;
	S->pData = pData;
}

arm_status arm_mat_inverse_f32 (const arm_matrix_instance_f32* pSrc, 
		arm_matrix_instance_f32* pDst)
{
	// Check sizing
	if(pSrc->numRows != pSrc->numCols
			|| pSrc->numRows != pDst->numRows
			|| pSrc->numCols != pDst->numCols)
	{
		return ARM_MATH_SIZE_MISMATCH;
	}
	// Convert to Eigen Matrix
	MatrixXf A(pSrc->numRows, pSrc->numCols);
	for(int i = 0; i < pSrc->numRows; ++i)
	{
		for(int j = 0; j < pSrc->numCols; ++j)
		{
			A(i,j) = pSrc->pData[pSrc->numCols * i + j];
		}
	}
	// Perform the inverse
	MatrixXf R = A.inverse();
	// Convert back
	for(int i = 0; i < pSrc->numRows; ++i)
	{
		for(int j = 0; j < pSrc->numCols; ++j)
		{
			pDst->pData[pSrc->numCols * i + j] = R(i,j);
		}
	}
	return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32 (const arm_matrix_instance_f32* pSrcA, 
		const arm_matrix_instance_f32* pSrcB, 
		arm_matrix_instance_f32 *pDst)
{
	// Eigen doesn't care
	return arm_mat_cmplx_mult_f32(pSrcA, pSrcB, pDst);
}

arm_status arm_mat_scale_f32 (const arm_matrix_instance_f32* pSrc, 
		float32_t scale, 
		arm_matrix_instance_f32* pDst)
{
	// Check sizing
	if(pSrc->numRows != pDst->numRows || pSrc->numCols != pDst->numCols)
	{
		return ARM_MATH_SIZE_MISMATCH;
	}
	// Scale is trivial
	for(int i = 0; i < pSrc->numRows; ++i)
	{
		for(int j = 0; j < pSrc->numCols; ++j)
		{
			pDst->pData[pSrc->numCols * i +j] = 
				pSrc->pData[pSrc->numCols * i + j] * scale;
		}
	}
	return ARM_MATH_SUCCESS;
}


arm_status arm_mat_sub_f32(const arm_matrix_instance_f32* pSrcA, 
		const arm_matrix_instance_f32* pSrcB, 
		arm_matrix_instance_f32* pDst)
{
	// Check sizing
	if(pSrcA->numRows != pSrcB->numRows 
			|| pSrcA->numCols != pSrcB->numCols
			|| pSrcA->numRows != pDst->numRows 
			|| pSrcA->numCols != pDst->numCols)
	{
		return ARM_MATH_SIZE_MISMATCH;
	}
	// Convert to Eigen matrix
	MatrixXf A(pSrcA->numRows, pSrcA->numCols);
	MatrixXf B(pSrcB->numRows, pSrcB->numCols);
	for(int i = 0; i < pSrcA->numRows; ++i)
	{
		for(int j = 0; j < pSrcA->numCols; ++j)
		{
			A(i,j) = pSrcA->pData[pSrcA->numCols * i + j];
			B(i,j) = pSrcB->pData[pSrcA->numCols * i + j];
		}
	}
	// Perform the addition
	MatrixXf R = A - B;
	// Convert back
	for(int i = 0; i < pSrcA->numRows; ++i)
	{
		for(int j = 0; j < pSrcA->numCols; ++j)
		{
			pDst->pData[pSrcA->numCols * i + j] = R(i,j);
		}
	}
	return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32* pSrc, 
		arm_matrix_instance_f32* pDst)
{
	// Check sizing
	if(pSrc->numRows != pDst->numCols || pSrc->numCols != pDst->numRows)
	{
		return ARM_MATH_SIZE_MISMATCH;
	}
	// Convert to Eigen Matrix
	MatrixXf A(pSrc->numRows, pSrc->numCols);
	for(int i = 0; i < pSrc->numRows; ++i)
	{
		for(int j = 0; j < pSrc->numCols; ++j)
		{
			A(i,j) = pSrc->pData[pSrc->numCols * i + j];
		}
	}
	// Perform the inverse
	MatrixXf R = A.transpose();
	// Convert back
	for(int i = 0; i < pDst->numRows; ++i)
	{
		for(int j = 0; j < pDst->numCols; ++j)
		{
			pDst->pData[pDst->numRows * i + j] = R(i,j);
		}
	}
	return ARM_MATH_SUCCESS;
}

float32_t arm_sin_f32 (float32_t x)
{
	return sin(x);
}

float32_t arm_cos_f32 (float32_t x)
{
	return cos(x);
}

arm_status arm_sqrt_f32 (float32_t in, float32_t *pOut)
{
	if(in < 0) return ARM_MATH_ARGUMENT_ERROR;
	*pOut = sqrt(in);
	return ARM_MATH_SUCCESS;
}


