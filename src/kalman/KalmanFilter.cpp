#include "kalman/KalmanFilter.hpp"

using namespace std;

/** @brief Simple function for initializing a matrix
 *
 */
inline static void MAT_INIT(arm_matrix_instance_f32* mat, size_t rows, size_t cols)
{
	//arm_mat_init_f32(mat, (uint16_t)rows, (uint16_t)cols, (float*)malloc(sizeof(float) * rows * cols));
	arm_mat_init_f32(mat, (uint16_t)rows, (uint16_t)cols, (float*)calloc((rows * cols), sizeof(float)));
}

inline static float& mat_at(arm_matrix_instance_f32& mat, int row, int col) {
	return mat.pData[row * mat.numCols + col];
}

inline static void mat_fill(arm_matrix_instance_f32& mat, float toFill) {
	for(int i = 0; i < mat.numRows * mat.numCols; i++) {
		mat.pData[i] = toFill;
	}
}

KalmanFilter::KalmanFilter() :
	n(6),
	u(3),
	l(2),
	p(2),
	c(3) {
	MAT_INIT(&state, n, 1);
	mat_fill(state, 0);
	MAT_INIT(&P, n, n);
	mat_fill(P, 0);
	MAT_INIT(&A, n, n);
	mat_fill(A, 0);
	mat_at(A, 0, 0) = 1;
	mat_at(A, 1, 1) = 1;
	mat_at(A, 2, 2) = 1;
	mat_at(A, 3, 3) = 1;
	mat_at(A, 4, 4) = 1;
	mat_at(A, 5, 5) = 1;
	MAT_INIT(&B, n, u);
	mat_fill(B, 0);
	MAT_INIT(&Q, n, n);
	mat_fill(Q, 0);
	MAT_INIT(&R_lidar, l, l);
	mat_fill(R_lidar, 0);
	MAT_INIT(&R_Px4, p, p);
	mat_fill(R_Px4, 0);
	//MAT_INIT(&R_camera, c, c);
	//mat_fill(R_camera, 0);
	MAT_INIT(&H_lidar, l, n);
	mat_fill(H_lidar, 0);
	mat_at(H_lidar, 0, 4) = 1;
	mat_at(H_lidar, 1, 5) = 1;
	MAT_INIT(&H_Px4, p, n);
	mat_fill(H_Px4, 0);
	mat_at(H_lidar, 0, 1) = 1;
	mat_at(H_lidar, 0, 4) = 1;
	//MAT_INIT(&H_camera, c, n);
	//mat_fill(H_camera, 0);
	MAT_INIT(&inter_nby1, n, 1); //used as temporary variables
	mat_fill(inter_nby1, 0);
	MAT_INIT(&inter_nbyn, n, n);
	mat_fill(inter_nbyn, 0);
	MAT_INIT(&inter_another_nbyn, n, n);
	mat_fill(inter_another_nbyn, 0);
	MAT_INIT(&inter_2by1, 2, 1);
	mat_fill(inter_2by1, 0);
	MAT_INIT(&inter_nby2, n, 2);
	mat_fill(inter_nby2, 0);
	MAT_INIT(&inter_another_nby2, n, 2);
	mat_fill(inter_another_nby2, 0);
	MAT_INIT(&inter_2byn, 2, n);
	mat_fill(inter_2byn, 0);
	MAT_INIT(&inter_2by2, 2, 2);
	mat_fill(inter_2by2, 0);
	MAT_INIT(&inter_another_2by2, 2, 2);
	mat_fill(inter_another_2by2, 0);
}

KalmanFilter::~KalmanFilter() {
	free(state.pData);
	free(P.pData);
	free(A.pData);
	free(B.pData);
	free(Q.pData);
	free(R_lidar.pData);
	free(R_Px4.pData);
	//free(R_camera.pData);
	free(H_lidar.pData);
	free(H_Px4.pData);
	//free(H_camera.pData);
	free(inter_nby1.pData);
	free(inter_nbyn.pData);
	free(inter_another_nbyn.pData);
	free(inter_2by1.pData);
	free(inter_nby2.pData);
	free(inter_another_nby2.pData);
	free(inter_2byn.pData);
	free(inter_2by2.pData);
	free(inter_another_2by2.pData);
}

void KalmanFilter::predict(const arm_matrix_instance_f32 u, float delta_t) {
	//Put delta_t in the appropriate spot in A and B
	//note: right now this assumes A is 6x6 and B is 6x3
	mat_at(A, 0, 1) = delta_t;
	mat_at(A, 2, 3) = delta_t;
	mat_at(A, 4, 5) = delta_t;
	mat_at(B, 1, 0) = delta_t;
	mat_at(B, 3, 1) = delta_t;
	mat_at(B, 5, 2) = delta_t;

	//predict step for state
	arm_mat_mult_f32(&A, &state, &inter_nby1);
	state = inter_nby1;
	arm_mat_mult_f32(&B, &u, &inter_nby1);
	arm_mat_add_f32(&state, &inter_nby1, &state);

	//predict step for P
	arm_mat_mult_f32(&A, &P, &inter_nbyn);
	arm_mat_trans_f32(&A, &inter_another_nbyn);
	arm_mat_mult_f32(&inter_nbyn, &inter_another_nbyn, &P);
	arm_mat_add_f32(&P, &Q, &P);
}

void KalmanFilter::correct2(const arm_matrix_instance_f32& z, const bool Px4) {

	arm_matrix_instance_f32* H;
	arm_matrix_instance_f32* R;
	if(Px4)
	{
		H = &H_Px4;
		R = &R_Px4;
	}
	else
	{
		H = &H_lidar;
		R = &R_lidar;
	}

	//H * x
	arm_mat_mult_f32(H, &state, &inter_2by1);
	//y = z - H * x
	arm_mat_sub_f32(&z, &inter_2by1, &inter_2by1);

	//H * P
	arm_mat_mult_f32(H, &P, &inter_2byn);
	//H transpose
	arm_mat_trans_f32(H, &inter_nby2);
	//H * P * H transpose
	arm_mat_mult_f32(&inter_2byn, &inter_nby2, &inter_2by2);
	//H * P * H transpose + R
	arm_mat_add_f32(&inter_2by2, R, &inter_2by2);
	//(H * P * H transpose + R)^(-1)
	arm_mat_inverse_f32(&inter_2by2, &inter_another_2by2);
	//P * H transpose
	arm_mat_mult_f32(&P, &inter_nby2, &inter_another_nby2);
	//K = P * H transpose * (H * P * H transpose + R)^(-1)
	arm_mat_mult_f32(&inter_another_nby2, &inter_another_2by2, &inter_nby2);

	//K * y
	arm_mat_mult_f32(&inter_nby2, &inter_2by1, &inter_nby1);
	//x+ = x + K * y
	arm_mat_add_f32(&state, &inter_nby1, &state);

	//K * H
	arm_mat_mult_f32(&inter_nby2, H, &inter_nbyn);
	//use A for I because it is already very close
	mat_at(A, 0, 1) = 0;
	mat_at(A, 2, 3) = 0;
	mat_at(A, 4, 5) = 0;
	//now it's a 6x6 identity
	//I - K * H
	arm_mat_sub_f32(&A, &inter_nbyn, &inter_another_nbyn);
	//(I - K * H) * P
	arm_mat_mult_f32(&inter_another_nbyn, &P, &inter_nbyn);
	//P+ = (I - K * H) * P
	P = inter_nbyn;
}

void KalmanFilter::correctPx4(const arm_matrix_instance_f32& z) {
	correct2(z, true);
}

void KalmanFilter::correctLidar(const arm_matrix_instance_f32& z) {
	correct2(z, false);
}

//Probably will be implemented later
void KalmanFilter::correctCamera() {

}

const arm_matrix_instance_f32& KalmanFilter::getState() const {
	return state;
}

const arm_matrix_instance_f32& KalmanFilter::getCovar() const {
	return P;
}

void KalmanFilter::reset() {


}
