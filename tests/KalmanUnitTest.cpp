#define BOOST_TEST_MODULE "KalmanFilterTest"
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include "kalman/KalmanFilter.hpp"
#include "cmeigen.hpp"

struct Fixture
{
	KalmanFilter kf;
	
	// sensor feedback (or ctrl input in Imu's case)
	float xcam;
	float ycam;
	float xdotPx4;
	float ydotPx4;
	float zLidar;
	float zdotLidar;
	float xddotImu;
	float yddotImu;
	float zddotImu;
	float dt;
	
	Fixture()
	{
		dt = 0.010;
		xcam = 5.7;
		ycam = 15.5;
		xdotPx4 = 0.5;
		ydotPx4 = 1.0;
		zLidar = 1.5;
		zdotLidar = 0.01;
		xddotImu = 0.01;
		yddotImu = 1.0;
		zddotImu = 0.001;
	}
};

BOOST_AUTO_TEST_CASE(ctorTest)
{
	Fixture f;
	const arm_matrix_instance_f32& x = f.kf.getState();
	const arm_matrix_instance_f32& P = f.kf.getCovar();
`    
    for (uint16_t r = 0; r < x.numRows; ++r)
    {
        for (uint16_t c = 0; c < x.numCols; ++c)
            BOOST_CHECK_CLOSE(mat_noref_at(x, r, c), 0.0, 0.0001);
    }
    
    for (uint16_t r = 0; r < P.numRows; ++r)
    {
        for (uint16_t c = 0; c < P.numCols; ++c)
            BOOST_CHECK_CLOSE(mat_noref_at(P, r, c), 0.0, 0.0001);
    }
}


BOOST_AUTO_TEST_CASE(predictTest)
{
	Fixture f;
	
	const arm_matrix_instance_f32& x = f.kf.getState();
	const arm_matrix_instance_f32& P = f.kf.getCovar();
}

/*
BOOST_AUTO_TEST_CASE(resetTest)
{
	Fixture f;
	const arm_matrix_instance_f32& xBefore = f.kf.getState();
	const arm_matrix_instance_f32& PBefore = f.kf.getCovar();
	
	// use same prediction stuff here

	f.kf.reset();
	const arm_matrix_instance_f32& xAfter = f.kf.getState();
	const arm_matrix_instance_f32& PAfter = f.kf.getCovar();
}

BOOST_AUTO_TEST_CASE(correctPx4Test)
{
	Fixture f;

	const arm_matrix_instance_f32& x = f.kf.getState();
	const arm_matrix_instance_f32& P = f.kf.getCovar();
}

BOOST_AUTO_TEST_CASE(correctLidarTest)
{
	Fixture f;

	const arm_matrix_instance_f32& x = f.kf.getState();
	const arm_matrix_instance_f32& P = f.kf.getCovar();
}

BOOST_AUTO_TEST_CASE(correctCameraTest)
{
	Fixture f;

	const arm_matrix_instance_f32& x = f.kf.getState();
	const arm_matrix_instance_f32& P = f.kf.getCovar();
}
*/
