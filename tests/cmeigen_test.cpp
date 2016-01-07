#include <cassert>
#include <cstring>
#include "cmeigen.hpp"

using namespace std;

// 0 1 2
// 3 4 5
// 6 7 8

// Uses all the functions
void test_compile()
{
	arm_status test;
	arm_matrix_instance_f32 a;
	float32_t aData[9];
	arm_matrix_instance_f32 b;
	float32_t bData[9];
	arm_matrix_instance_f32 c;
	float32_t cData[9];
	arm_mat_init_f32(&a, 3, 3, aData);
	arm_mat_init_f32(&b, 3, 3, bData);
	arm_mat_init_f32(&c, 3, 3, cData);
	test = arm_mat_add_f32(&a, &b, &c);
	arm_mat_scale_f32(&a, 3, &c);
	arm_mat_sub_f32(&a, &b, &c);
	arm_mat_mult_f32(&a, &b, &c);
	arm_mat_cmplx_mult_f32(&a, &b, &c);
	arm_mat_trans_f32(&a, &c);
	aData[0] = 1;
	aData[4] = 1;
	aData[8] = 1;
	arm_mat_inverse_f32(&a, &c);
}

void test_add()
{
	//setup
	arm_matrix_instance_f32 a;
	float32_t aData[9];
	arm_matrix_instance_f32 b;
	float32_t bData[9];
	arm_matrix_instance_f32 c;
	float32_t cData[9];
	arm_matrix_instance_f32 d;
	float32_t dData[6];
	aData[0] = 1;
	aData[4] = 1;
	aData[8] = 1;
	bData[0] = 1;
	bData[4] = 1;
	bData[8] = 1;
	arm_mat_init_f32(&a, 3, 3, aData);
	arm_mat_init_f32(&b, 3, 3, bData);
	arm_mat_init_f32(&c, 3, 3, cData);
	arm_mat_init_f32(&d, 2, 3, dData);
	//test I3 + I3 = 2I3
	assert(arm_mat_add_f32(&a, &b, &c) == ARM_MATH_SUCCESS);
	assert(cData[0] == 2);
	assert(cData[4] == 2);
	assert(cData[8] == 2);
	//test size mismatch
	assert(arm_mat_add_f32(&d, &b, &c) == ARM_MATH_SIZE_MISMATCH);
	assert(arm_mat_add_f32(&a, &d, &c) == ARM_MATH_SIZE_MISMATCH);
	assert(arm_mat_add_f32(&a, &b, &d) == ARM_MATH_SIZE_MISMATCH);
}

void test_scale()
{
	//setup
	arm_matrix_instance_f32 a;
	float32_t aData[9];
	arm_matrix_instance_f32 c;
	float32_t cData[9];
	aData[0] = 1;
	aData[4] = 1;
	aData[8] = 1;
	arm_mat_init_f32(&a, 3, 3, aData);
	arm_mat_init_f32(&c, 3, 3, cData);
	//test 2*I3 = 2I3
	assert(arm_mat_scale_f32(&a, 2, &c) == ARM_MATH_SUCCESS);
	assert(cData[0] == 2);
	assert(cData[4] == 2);
	assert(cData[8] == 2);
}

void test_mult()
{
	//setup
	arm_matrix_instance_f32 a;
	float32_t aData[9]; memset(aData, 0.0, sizeof(float[9]));
	arm_matrix_instance_f32 b;
	float32_t bData[9]; memset(bData, 0.0, sizeof(float[9]));
	arm_matrix_instance_f32 c;
	float32_t cData[9]; memset(cData, 0.0, sizeof(float[9]));
	arm_matrix_instance_f32 d;
	float32_t dData[6]; memset(dData, 0.0, sizeof(float[9]));
	arm_matrix_instance_f32 e;
	float32_t eData[6]; memset(eData, 0.0, sizeof(float[9]));
	aData[0] = 2;
	aData[4] = 2;
	aData[8] = 2;
	bData[0] = -1;
	bData[4] = -1;
	bData[8] = -1;
	dData[0] = 1;
	arm_mat_init_f32(&a, 3, 3, aData);
	arm_mat_init_f32(&b, 3, 3, bData);
	arm_mat_init_f32(&c, 3, 3, cData);
	arm_mat_init_f32(&d, 3, 2, dData);
	arm_mat_init_f32(&e, 3, 2, eData);
	//test -I3 * 2*I3 = -2I3
	assert(arm_mat_mult_f32(&a, &b, &c) == ARM_MATH_SUCCESS);
	assert(cData[0] == -2);
	assert(cData[4] == -2);
	assert(cData[8] == -2);
	//test size mismatch
	assert(arm_mat_mult_f32(&d, &b, &c) == ARM_MATH_SIZE_MISMATCH);
	assert(arm_mat_mult_f32(&a, &d, &e) == ARM_MATH_SUCCESS);
	assert(arm_mat_mult_f32(&a, &b, &d) == ARM_MATH_SIZE_MISMATCH);
}

void test_inv()
{
	//setup
	arm_matrix_instance_f32 a;
	float32_t aData[9];
	arm_matrix_instance_f32 c;
	float32_t cData[9];
	arm_matrix_instance_f32 d;
	float32_t dData[6];
	aData[0] = 7;
	aData[4] = 1;
	aData[8] = 1;
	arm_mat_init_f32(&a, 3, 3, aData);
	arm_mat_init_f32(&c, 3, 3, cData);
	arm_mat_init_f32(&d, 3, 2, cData);
	//test inv
	assert(arm_mat_inverse_f32(&a, &c) == ARM_MATH_SUCCESS);
	assert(cData[0] = 1.0/7.0);
	assert(cData[4] = 1);
	assert(cData[8] = 1);
	//test size mismatch
	assert(arm_mat_inverse_f32(&a, &d) == ARM_MATH_SIZE_MISMATCH);
	assert(arm_mat_inverse_f32(&d, &a) == ARM_MATH_SIZE_MISMATCH);
	assert(arm_mat_inverse_f32(&d, &d) == ARM_MATH_SIZE_MISMATCH);
}

void test_trans()
{
	//setup
	arm_matrix_instance_f32 a;
	float32_t aData[6];
	arm_matrix_instance_f32 c;
	float32_t cData[6];
	arm_matrix_instance_f32 d;
	float32_t dData[9];
	aData[0] = 1;
	aData[1] = 2;
	aData[4] = 3;
	arm_mat_init_f32(&a, 2, 3, aData);
	arm_mat_init_f32(&c, 3, 2, cData);
	arm_mat_init_f32(&d, 3, 3, cData);
	//test inv
	assert(arm_mat_trans_f32(&a, &c) == ARM_MATH_SUCCESS);
	assert(cData[0] = 1);
	assert(cData[1] = 3);
	assert(cData[4] = 2);
	//test size mismatch
	assert(arm_mat_trans_f32(&a, &d) == ARM_MATH_SIZE_MISMATCH);
	assert(arm_mat_trans_f32(&a, &a) == ARM_MATH_SIZE_MISMATCH);
	assert(arm_mat_trans_f32(&d, &d) == ARM_MATH_SUCCESS);
}

int main(void)
{
	test_add();
	test_scale();
	test_mult();
	test_inv();
	test_trans();
	return 0;
}
