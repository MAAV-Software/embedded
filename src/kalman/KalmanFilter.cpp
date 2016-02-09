#include "kalman/KalmanFilter.hpp"
using namespace std;

/** 
 * @brief Simple function for initializing a matrix
 *
 * @details Basically just calls arm_mat_init_f32, initializing memory as necessary
 *
 * @param mat a pointer to the matrix being initialized
 * @param rows number of rows in the desired matrix
 * @param cols number of cols in the desired matrix
 */
inline static void mat_init(arm_matrix_instance_f32* mat, uint16_t rows, uint16_t cols)
{
	arm_mat_init_f32(mat, rows, cols, (float*)calloc((rows * cols), sizeof(float)));
}

inline static void mat_destroy(arm_matrix_instance_f32 &mat) 
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
inline static float& mat_at(arm_matrix_instance_f32& mat, uint16_t row, uint16_t col)
{
	return mat.pData[row * mat.numCols + col];
}

float mat_noref_at(const arm_matrix_instance_f32& mat, uint16_t row, uint16_t col)
{
	return mat.pData[row * mat.numCols + col];
}

inline static void mat_fill(arm_matrix_instance_f32& mat, float toFill)
{
	for (uint16_t i = 0; i < mat.numRows * mat.numCols; i++) 
		mat.pData[i] = toFill;
}

KalmanFilter::KalmanFilter() :
	n_size(6),
	u_size(3),
	l_size(2),
	p_size(2),
	c_size(3) 
{
    //initialize state matrix and set values to 0
	mat_init(&state, n_size, 1);
	mat_fill(state, 0);

    //initialize covariances matrix and set them to 0 too
	mat_init(&P, n_size, n_size);
	mat_fill(P, 0);

    //A = diag(1, 1, 1, 1, 1, 1)
    //note that A will have changes made to it within some of the functions
	mat_init(&A, n_size, n_size);
	mat_fill(A, 0);
	mat_at(A, 0, 0) = 1;
	mat_at(A, 1, 1) = 1;
	mat_at(A, 2, 2) = 1;
	mat_at(A, 3, 3) = 1;
	mat_at(A, 4, 4) = 1;
	mat_at(A, 5, 5) = 1;

    //initialize B and set values to 0
	mat_init(&B, n_size, u_size);
	mat_fill(B, 0);

    //initialize and zero Q
	mat_init(&Q, n_size, n_size);
	mat_fill(Q, 0);

    //initialize and zero R for the lidar
	mat_init(&R_lidar, l_size, l_size);
	mat_fill(R_lidar, 0);

    //initialize and zero R for the Px4
	mat_init(&R_Px4, p_size, p_size);
	mat_fill(R_Px4, 0);

    //initialize and zero R for the camera
	mat_init(&R_camera, c_size, c_size);
	mat_fill(R_camera, 0);

    //H_lidar = [ 0 0 0 0 1 0; 0 0 0 0 0 1 ]
	mat_init(&H_lidar, l_size, n_size);
	mat_fill(H_lidar, 0);
	mat_at(H_lidar, 0, 4) = 1;
	mat_at(H_lidar, 1, 5) = 1;

    //H_Px4 = [ 0 1 0 0 0 0; 0 0 0 1 0 0 ]
	mat_init(&H_Px4, p_size, n_size);
	mat_fill(H_Px4, 0);
	mat_at(H_lidar, 0, 1) = 1;
	mat_at(H_lidar, 0, 4) = 1;

    //H_camera = [ 1 0 0 0 0 0; 0 0 1 0 0 0 ] TODO correct?
	mat_init(&H_camera, c_size, n_size);
	mat_fill(H_camera, 0);
    mat_at(H_camera, 0, 0) = 1;
    mat_at(H_camera, 1, 2) = 1;

    //initialize z matrices
    mat_init(&z_2by1, 2, 1);

    //initialize a bunch of intermediate matrices TODO no need to bother filling these, right?
	mat_init(&inter_nby1, n_size, 1); //used as temporary variables
	mat_init(&inter_nbyn, n_size, n_size);
	mat_init(&inter_another_nbyn, n_size, n_size);
	mat_init(&inter_2by1, 2, 1);
	mat_init(&inter_nby2, n_size, 2);
	mat_init(&inter_another_nby2, n_size, 2);
	mat_init(&inter_2byn, 2, n_size);
	mat_init(&inter_2by2, 2, 2);
	mat_init(&inter_another_2by2, 2, 2);
}

KalmanFilter::~KalmanFilter()
{
	mat_destroy(state);
	mat_destroy(P);
	mat_destroy(A);
	mat_destroy(B);
	mat_destroy(Q);
	mat_destroy(R_lidar);
	mat_destroy(R_Px4);
	mat_destroy(R_camera);
	mat_destroy(H_lidar);
	mat_destroy(H_Px4);
	mat_destroy(H_camera);
    mat_destroy(z_2by1);
	mat_destroy(inter_nby1);
	mat_destroy(inter_nbyn);
	mat_destroy(inter_another_nbyn);
	mat_destroy(inter_2by1);
	mat_destroy(inter_nby2);
	mat_destroy(inter_another_nby2);
	mat_destroy(inter_2byn);
	mat_destroy(inter_2by2);
	mat_destroy(inter_another_2by2);
}

void KalmanFilter::predict(const arm_matrix_instance_f32 &u, float dt)
{
	//Put dt in the appropriate spot in A and B
	//note: right now this assumes A is 6x6 and B is 6x3
	mat_at(A, 0, 1) = dt;
	mat_at(A, 2, 3) = dt;
	mat_at(A, 4, 5) = dt;
	mat_at(B, 1, 0) = dt;
	mat_at(B, 3, 1) = dt;
	mat_at(B, 5, 2) = dt;

	//predict step for state
	arm_mat_mult_f32(&A, &state, &inter_nby1); // A * x
	state = inter_nby1; // x = A * x
	arm_mat_mult_f32(&B, &u, &inter_nby1); //B * u
	arm_mat_add_f32(&state, &inter_nby1, &state);//x = A * x + B * u

	//predict step for P
	arm_mat_mult_f32(&A, &P, &inter_nbyn); //A * P
	arm_mat_trans_f32(&A, &inter_another_nbyn); //A^T
	arm_mat_mult_f32(&inter_nbyn, &inter_another_nbyn, &P); //P = A * P * A^T
	arm_mat_add_f32(&P, &Q, &P); //P = A * P * A^T + Q
}

void KalmanFilter::correct2(const arm_matrix_instance_f32& z, const CorrectionType sensor)
{
	arm_matrix_instance_f32* H;
	arm_matrix_instance_f32* R;

    switch (sensor) 
	{
        case LIDAR:
            H = &H_lidar;
            R = &R_lidar;
            break;
        case PX4:
            H = &H_Px4;
            R = &R_Px4;
            break;
        case CAMERA:
            H = &H_camera;
            R = &R_camera;
            break;
        default:
            //Really, we probably want to throw some sort of error here. Not cure exactly how that should happen
            break;
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

void KalmanFilter::correctPx4(const float xdot, const float ydot)
{
    //z_2by1 = [ xdot; ydot ]
    mat_at(z_2by1, 0, 0) = xdot;
    mat_at(z_2by1, 1, 0) = ydot;
	correct2(z_2by1, PX4);
}

void KalmanFilter::correctLidar(const float z, const float zdot)
{
    //z_2by1 = [ z; zdot ]
    mat_at(z_2by1, 0, 0) = z;
    mat_at(z_2by1, 1, 0) = zdot;
	correct2(z_2by1, LIDAR);
}

void KalmanFilter::correctCamera(const float x, const float y)
{
    //z_2by1 = [ x; y ]
    mat_at(z_2by1, 0, 0) = x;
    mat_at(z_2by1, 1, 0) = y;
    correct2(z_2by1, CAMERA);
}

const arm_matrix_instance_f32& KalmanFilter::getState() const 
{
	return state;
}

const arm_matrix_instance_f32& KalmanFilter::getCovar() const 
{
	return P;
}

void KalmanFilter::reset()
{
	mat_fill(state, 0);
	mat_fill(P, 0);
}
